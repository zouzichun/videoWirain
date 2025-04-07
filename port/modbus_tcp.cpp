#include "modbus_tcp.h"
#include <QByteArray>
#include <QDebug>

ModbusTcp::ModbusTcp(QWidget *parent) :
    Port(parent),
    tcp_ip("192.168.20.32"),
    tcp_port(520) {
    m_modbustcp = new QModbusTcpClient();
}

ModbusTcp::~ModbusTcp(){
    if (m_modbustcp)
        delete m_modbustcp;
}

int ModbusTcp::startPort(const ConfigData &configData) {
    if (!m_modbustcp)
        return -1;

    if (m_modbustcp->state() != QModbusDevice::ConnectedState) {
        tcp_ip = configData.modbusTcpIp;
        tcp_port = configData.modbusTcpPort;
        qDebug("try start modbus %s:%d", tcp_ip.toStdString().c_str(), tcp_port);
        m_modbustcp->setConnectionParameter(QModbusDevice::NetworkAddressParameter,tcp_ip);
        m_modbustcp->setConnectionParameter(QModbusDevice::NetworkPortParameter,tcp_port);
        if (!m_modbustcp->connectDevice()) {
            qDebug("连接modbus设备 %s:%d失败", tcp_ip.toStdString().c_str(), tcp_port);
            return -1;
        }
        else {
            qDebug("成功连接到modbs设备%s:%d", tcp_ip.toStdString().c_str(), tcp_port);
            qDebug("modbus status %d", m_modbustcp->state());
            return 0;
        }
    } else {
        qDebug("port %s:%d is already started!", tcp_ip.toStdString().c_str(), tcp_port);
    }

    return 0;
}
    
void ModbusTcp::closePort() {
    qDebug("stop modbus", tcp_ip.toStdString().c_str(), tcp_port);
    m_modbustcp->disconnectDevice();
}

bool ModbusTcp::readModbusData(int typeNum,int startAdd, quint16 numbers) {
    //读取modbus设备各寄存器数据
    //typeNum:1_线圈 2_离散输入 3_保持 4_输入
    if(m_modbustcp->state() != QModbusDevice::ConnectedState)
    {
        qDebug("modbus status %d", m_modbustcp->state());
        return false;
    }

    //确定寄存器类型
    QModbusDataUnit ReadUnit;
    if(typeNum == 1)
    {
        ReadUnit = QModbusDataUnit(QModbusDataUnit::Coils,startAdd,numbers);
    }
    else if(typeNum == 2)
    {
        ReadUnit = QModbusDataUnit(QModbusDataUnit::DiscreteInputs,startAdd,numbers);
    }
    else if(typeNum == 3)
    {
        ReadUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters,startAdd,numbers);
    }
    else if(typeNum == 4)
    {
        ReadUnit = QModbusDataUnit(QModbusDataUnit::InputRegisters,startAdd,numbers);
    }
    else
    {
        qDebug("读取寄存器类型错误");
        return false;
    }
    qDebug("readModbusData typeNum: %d, start addr %d, numbers %d", typeNum, startAdd, numbers);

    //多读
    if(auto *reply = m_modbustcp->sendReadRequest(ReadUnit,1)) {
        if(!reply->isFinished()) {
            if((typeNum == 1) || (typeNum == 2)) {
                QObject::connect(reply,&QModbusReply::finished,this,&ModbusTcp::slot_readReadyCoils);   //读取线圈
            }
            if((typeNum == 3) || (typeNum == 4)) {
                QObject::connect(reply,&QModbusReply::finished,this,&ModbusTcp::slot_readReadyRegisters);   //读取寄存器
            }
            //reply->deleteLater();
            return true;
        } else {
            reply->deleteLater();
            return false;
        }
    } else {
        qDebug("读取错误: %s", m_modbustcp->errorString().toStdString().c_str());
        return false;
    }

    return false;
}

//对modbus设备各寄存器写入数据
//typeNum:1_线圈 2_保持 (这两类寄存器可读可写,其余的只读)
bool ModbusTcp::writeModbusData(int typeNum,int startAdd, uint32_t writeNum)
{
    if(m_modbustcp->state() != QModbusDevice::ConnectedState) {
        return false;
    }

    //确定寄存器类型
    QModbusDataUnit writeUnit;
    if(typeNum == 1) {
        writeUnit = QModbusDataUnit(QModbusDataUnit::Coils,startAdd,1);   //写入一个数据
        writeUnit.setValue(0,writeNum);

        //单写
        //bool ok;
        //quint16 hexData = writeData.toInt(&ok,16);   //转16进制
    } else if(typeNum == 2) {
        writeUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters,startAdd,2);   //写入两个数据
        quint16 uData16[2] = {0};
        uData16[0] = writeNum & 0xffff;
        uData16[1] = (writeNum >> 16) & 0xffff;
        writeUnit.setValue(0,uData16[0]);
        writeUnit.setValue(1,uData16[1]);
        qDebug("write data h: %x, l: %x, val %f", uData16[1], uData16[0], static_cast<float>(writeNum));
        //LOGDEBUG<<"uData16[0]:"<<uData16[0]<<"   uData16[1]:"<<uData16[1]<<"   writeNum:"<<writeNum;
    } else {
        qDebug("写入寄存器类型错误");
        return false;
    }
    //LOGDEBUG<<"writeModbusData typeNum:"<<typeNum<<"   writeNum:"<<writeNum;
    if(auto *reply = m_modbustcp->sendWriteRequest(writeUnit,1)) {
        if(!reply->isFinished()) {
            connect(reply,&QModbusReply::finished,this,[reply]() {
                if(reply->error() == QModbusDevice::NoError) {
                    reply->deleteLater();
                    return true;
                } else {
                    qDebug("写入返回错误: %s", reply->error());
                    reply->deleteLater();
                    return false;
                }
            });
        } else {
            reply->deleteLater();
            return false;
        }
    } else {
        qDebug("写入错误: %s" ,m_modbustcp->errorString().toStdString().c_str());
        return false;
    }
    return true;
}


//监听TCP连接的状态,若状态发生改变,发出对应的信号
void ModbusTcp::slot_stateChanged()
{
    if(m_modbustcp->state() == QModbusDevice::ConnectedState)
    {
        emit signal_stateChanged(true);
    }
    else if(m_modbustcp->state() == QModbusDevice::UnconnectedState)
    {
        emit signal_stateChanged(false);
    }
}

uint8_t Coils_Bufer[2000];

//接收到读取线圈/离散输入寄存器请求后执行的槽函数
void ModbusTcp::slot_readReadyCoils() {
    QVector<quint16> vAllData;
    QModbusReply *reply = qobject_cast<QModbusReply *>(sender());
    if(!reply) {
        qDebug("读取线圈/离散输入寄存器错误");
        return;
    }
    if(reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();
        vAllData = unit.values();
        // qDebug("read ceil %d", vAllData);
        for(uint16_t i=0; i< unit.valueCount();  i++) {
            uint16_t res=unit.value(i);			//一个一个读
            Coils_Bufer[i] = static_cast<uint8_t>(res);
            //读完将数据存储起来  Coils_Bufer[i] 自定的数组 用来存放数据
        }
        emit signal_readCoils(vAllData);
    } else {
        qDebug("线圈/离散输入寄存器回复错误: %s", reply->error());
    }
    reply->deleteLater();
}

//接收到读取保持/输入寄存器请求后执行的槽函数
void ModbusTcp::slot_readReadyRegisters()
{
    QModbusReply *reply = qobject_cast<QModbusReply *>(sender());
    if(!reply) {
        qDebug("读取保持/输入寄存器错误");
        return;
    }
    if(reply->error() == QModbusDevice::NoError) {
        const QModbusDataUnit unit = reply->result();
        auto valueList = unit.values();
        int nSize = valueList.size();
        if(nSize == 2) {
            quint16 uData16[2] = {0};
            uData16[0] = valueList[0];
            uData16[1] = valueList[1];
            int resultNum = uData16[0] | (uData16[1] << 16);

            QByteArray val;
            val.resize(4);
            memcpy(&resultNum, val.data(), 4);

            //LOGDEBUG<<"uData16[0]:"<<uData16[0]<<"   uData16[1]:"<<uData16[1]<<"   resultNum:"<<resultNum;
            qDebug("read register h: %x, l: %x, val %f", resultNum >> 16, resultNum & 0xffff, *reinterpret_cast<float *>(val.data()));
            emit signal_readRegisters(resultNum);
        } else {
            qDebug("保持寄存器返回数据错误,个数: %d", nSize);
        }
    } else {
        qDebug("保持/输入寄存器回复错误: %s", reply->error());
    }
    reply->deleteLater();
}
