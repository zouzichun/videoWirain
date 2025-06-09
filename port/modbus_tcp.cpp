#include "modbus_tcp.h"
#include <QByteArray>
#include <QDebug>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <QThread>

ModbusTcp::ModbusTcp(QWidget *parent) :
    Port(parent),
    tcp_ip("192.168.1.111"),
    tcp_port(502) {
    m_modbustcp = new QModbusTcpClient(this);
    
    // 连接状态变化和错误信号
    connect(m_modbustcp, &QModbusClient::stateChanged, this, &ModbusTcp::handleStateChanged);
    connect(m_modbustcp, &QModbusClient::errorOccurred, this, &ModbusTcp::handleErrorOccurred);
}

ModbusTcp::~ModbusTcp(){
    if (m_modbustcp) {
        m_modbustcp->disconnectDevice();
        delete m_modbustcp;
    }
}


// 处理状态变化
void ModbusTcp::handleStateChanged(QModbusDevice::State state)
{
    if (state == QModbusDevice::ConnectedState) {
        // ui->connectButton->setText("断开连接");
        // ui->statusLabel->setText("已连接到服务器");
        spdlog::info("已连接到服务器");
    } else {
        // ui->connectButton->setText("连接");
        // ui->statusLabel->setText("未连接");
        spdlog::info("未连接");
    }
}

// 处理错误
void ModbusTcp::handleErrorOccurred(QModbusDevice::Error error)
{
    if (error == QModbusDevice::NoError)
        return;
    
    // ui->statusLabel->setText(tr("错误: %1").arg(m_modbustcp->errorString()));
    spdlog::info("错误: {}", m_modbustcp->errorString().toStdString());
}


int ModbusTcp::startPort(const ConfigData &configData) {
    if (!m_modbustcp)
        return -1;

    if (m_modbustcp->state() == QModbusDevice::ConnectedState) {
        spdlog::info("already started, disconnect modbus {}:{}", tcp_ip.toStdString().c_str(), tcp_port);
        m_modbustcp->disconnectDevice();
    } else {
        tcp_ip = configData.modbusTcpIp;
        tcp_port = configData.modbusTcpPort;
        spdlog::info("try start modbus {}:{}, timeout {}, retry {}",
            tcp_ip.toStdString().c_str(), tcp_port,
            configData.modbusTimeout, configData.modbusNumRetry);

        m_modbustcp->setConnectionParameter(QModbusDevice::NetworkAddressParameter, tcp_ip);
        m_modbustcp->setConnectionParameter(QModbusDevice::NetworkPortParameter, tcp_port);
        m_modbustcp->setTimeout(configData.modbusTimeout);
        m_modbustcp->setNumberOfRetries(configData.modbusNumRetry);
        if (!m_modbustcp->connectDevice()) {
            spdlog::info("connect modbus  {}:{} failed!", tcp_ip.toStdString().c_str(), tcp_port);
            return -1;
        }
        else {
            if (QIODevice *device = m_modbustcp->device()) {  // 获取底层设备
                if (QTcpSocket *socket = qobject_cast<QTcpSocket*>(device)) {
                    // 设置 TCP_NODELAY (禁用 Nagle 算法)
                    socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
                    qDebug() << "Nagle algorithm disabled";
                }
            }
            spdlog::info("connect to modbus {}:{} succeeded!", tcp_ip.toStdString().c_str(), tcp_port);
            
            return 0;
        }
    }

    return 0;
}
    
void ModbusTcp::closePort() {
    spdlog::info("stop modbus {}:{}", tcp_ip.toStdString().c_str(), tcp_port);
    m_modbustcp->disconnectDevice();
}

void ModbusTcp::handleDataReady(float & val) {

}

bool ModbusTcp::readModbusData(int startAdd, int numbers, int &val) {
//    qDebug("ConnectedState %d", m_modbustcp->state());
    if (!m_modbustcp || m_modbustcp->state() != QModbusDevice::ConnectedState)
        return false;
    QModbusDataUnit readUnit(QModbusDataUnit::InputRegisters, startAdd, numbers);

    if (auto *reply = m_modbustcp->sendReadRequest(readUnit, 1)) {
        if (!reply->isFinished()) {
            connect(reply, &QModbusReply::finished, this, [=]() {
                if (reply->error() == QModbusDevice::NoError) {
                    const QModbusDataUnit unit = reply->result();
                    auto valueList = unit.values();
                    int nSize = valueList.size();
                    if(nSize == 2) {
                        quint16 uData16[2] = {0};
                        uData16[0] = valueList[0];
                        uData16[1] = valueList[1];

                        uint32_t resultNum = uData16[1];
                        resultNum = resultNum << 16;
                        resultNum |= uData16[0];

                        // Debug("read register h: %x, l: %x, val %x", resultNum >> 16, resultNum & 0xffff, resultNum);
                        int val = 0;
                        memcpy(&val, &resultNum, sizeof(int));
                        {
                            std::lock_guard<std::mutex> lg(mtx);
                            rdy_data = val;
                            rdy_flag = true;
                        }
                        emit signal_UpdateReadData(val);
                        // spdlog::info("read register h: {:#x}, l: {:#x}, val {}", resultNum >> 16, resultNum & 0xffff, val);
                    } else {
//                        spdlog::info("保持寄存器返回数据错误,个数: {}", nSize);
                    }
                } else {
//                    spdlog::info("read failed! {}", reply->errorString().toStdString().c_str());
                }
                reply->deleteLater();
            });
        }
    } else {
        spdlog::info("读取请求失败");
        return false;
    }

    return true;
}

//对modbus设备各寄存器写入数据
//typeNum:1_线圈 2_保持 (这两类寄存器可读可写,其余的只读)
bool ModbusTcp::writeModbusData(int startAdd, int numbers, float val)
{
    if(m_modbustcp->state() != QModbusDevice::ConnectedState) {
        return false;
    }

     QModbusDataUnit writeUnit;
     writeUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters,startAdd,2);
    quint16 uData16[2] = {0};
    uint32_t tval;
    memcpy(&tval, &val, sizeof(float));
    uData16[0] = tval & 0xffff;
    uData16[1] = (tval >> 16) & 0xffff;
     writeUnit.setValue(0,uData16[0]);
     writeUnit.setValue(1,uData16[1]);
//    spdlog::info("write addr {:#x} data h: {:#x}, l: {:#x}, val {:.2f}", startAdd, uData16[1], uData16[0], val);

//    QModbusRequest request(QModbusRequest::WriteSingleRegister,
//        QByteArray::fromRawData(reinterpret_cast<const char*>(&uData16),
//                       sizeof(uData16)));

    if(auto *reply = m_modbustcp->sendWriteRequest(writeUnit, 1)) {
        if(!reply->isFinished()) {
            connect(reply,&QModbusReply::finished,this,[=]() {
                if(reply->error() == QModbusDevice::NoError) {
//                    spdlog::info("write addr {}, numbers {} val {:.2f} success", startAdd, numbers, val);
                } else {
                    spdlog::info("write failed: {}", reply->error());
                }
                reply->deleteLater();
            });
        }
    } else {
        spdlog::info("write failed! {}" ,m_modbustcp->errorString().toStdString().c_str());
        return false;
    }
    return true;
}

//对modbus设备各寄存器写入数据
//typeNum:1_线圈 2_保持 (这两类寄存器可读可写,其余的只读)
bool ModbusTcp::writeModbusData(int startAdd, int numbers, int val)
{
    if(m_modbustcp->state() != QModbusDevice::ConnectedState) {
        return false;
    }

     QModbusDataUnit writeUnit;
     writeUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters,startAdd,2);
    quint16 uData16[2] = {0};
    uint32_t tval;
    memcpy(&tval, &val, sizeof(int));
    uData16[0] = tval & 0xffff;
    uData16[1] = (tval >> 16) & 0xffff;
     writeUnit.setValue(0,uData16[0]);
     writeUnit.setValue(1,uData16[1]);
//    spdlog::info("write addr {:#x} data h: {:#x}, l: {:#x}, val {:.2f}", startAdd, uData16[1], uData16[0], val);

//    QModbusRequest request(QModbusRequest::WriteSingleRegister,
//        QByteArray::fromRawData(reinterpret_cast<const char*>(&uData16),
//                       sizeof(uData16)));

    if(auto *reply = m_modbustcp->sendWriteRequest(writeUnit, 1)) {
        if(!reply->isFinished()) {
            connect(reply,&QModbusReply::finished,this,[=]() {
                if(reply->error() == QModbusDevice::NoError) {
//                    spdlog::info("write addr {}, numbers {} val {:.2f} success", startAdd, numbers, val);
                } else {
                    spdlog::info("write failed: {}", reply->error());
                }
                reply->deleteLater();
            });
        }
    } else {
        spdlog::info("write failed! {}" ,m_modbustcp->errorString().toStdString().c_str());
        return false;
    }
    return true;
}

bool ModbusTcp::waitDataReady() {
    return true;
}
