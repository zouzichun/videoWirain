#include "modbus.h"
#include <QByteArray>
#include <QDebug>

ModbusPort::ModbusPort(QWidget *parent) :
    Port(parent),
    buad_rate(9600) {
}

ModbusPort::~ModbusPort(){
}

int ModbusPort::startPort(const ConfigData &configData) {
    qDebug("try start mobus %s, %d", configData.modbusName.toStdString().c_str(), configData.modbusRate);
    // m_serial.setPortName(configData.modbusName);
    // m_serial.setBaudRate(configData.modbusRate);
    // m_serial.setDataBits(QSerialPort::Data8);
    // m_serial.setParity(QSerialPort::NoParity);
    // m_serial.setStopBits(QSerialPort::TwoStop);

    // if (!m_serial.open(QIODevice::ReadWrite)) {
    //     qDebug("open modbus serial port failed!");
    //     return -1;
    // }

    m_master.setConnectionParameter(QModbusDevice::SerialPortNameParameter, configData.modbusName);
    m_master.setConnectionParameter(QModbusDevice::SerialBaudRateParameter, configData.modbusRate);
    m_master.setConnectionParameter(QModbusDevice::SerialDataBitsParameter, QSerialPort::Data8);
    m_master.setConnectionParameter(QModbusDevice::SerialParityParameter, QSerialPort::NoParity);
    m_master.setConnectionParameter(QModbusDevice::SerialStopBitsParameter, QSerialPort::TwoStop);

   if (!m_master.connectDevice()) {
       qDebug("connect device failed!");
       return -1;
   }

    return 0;
}
    
void ModbusPort::closePort() {
    m_master.disconnectDevice();
    m_serial.close();
}

bool ModbusPort::readModbusData(int typeNum,int startAdd, quint16 numbers) {
    qDebug("modbus send read message!");
    //读取modbus设备各寄存器数据
    //typeNum:1_线圈 2_离散输入 3_保持 4_输入
    if(m_master.state() != QModbusDevice::ConnectedState)
    {
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
    qDebug("readModbusData typeNum: %d", typeNum);

    //多读
    if(auto *reply = m_master.sendReadRequest(ReadUnit,1))
    {
        if(!reply->isFinished())
        {
            if((typeNum == 1) || (typeNum == 2))
            {
                QObject::connect(reply,&QModbusReply::finished,this,&ModbusPort::slot_readReadyCoils);   //读取线圈
            }
            if((typeNum == 3) || (typeNum == 4))
            {
                QObject::connect(reply,&QModbusReply::finished,this,&ModbusPort::slot_readReadyRegisters);   //读取寄存器
            }
            //reply->deleteLater();
            return true;
        }
        else
        {
            reply->deleteLater();
            return false;
        }
    }
    else
    {
        qDebug("读取错误: %s", m_master.errorString().toStdString().c_str());
        return false;
    }

    return false;
}

//对modbus设备各寄存器写入数据
//typeNum:1_线圈 2_保持 (这两类寄存器可读可写,其余的只读)
bool ModbusPort::writeModbusData(int typeNum,int startAdd, uint32_t writeNum)
{
    if(m_master.state() != QModbusDevice::ConnectedState)
    {
        return false;
    }

    //确定寄存器类型
    QModbusDataUnit writeUnit;
    if(typeNum == 1)
    {
        writeUnit = QModbusDataUnit(QModbusDataUnit::Coils,startAdd,1);   //写入一个数据
        writeUnit.setValue(0,writeNum);

        //单写
        //bool ok;
        //quint16 hexData = writeData.toInt(&ok,16);   //转16进制
    }
    else if(typeNum == 2)
    {
        writeUnit = QModbusDataUnit(QModbusDataUnit::HoldingRegisters,startAdd,2);   //写入两个数据
        quint16 uData16[2] = {0};
        uData16[0] = writeNum & 0xffff;
        uData16[1] = (writeNum >> 16) & 0xffff;
        writeUnit.setValue(0,uData16[0]);
        writeUnit.setValue(1,uData16[1]);
        qDebug("write data h: %x, l: %x, val %f", uData16[1], uData16[0], static_cast<float>(writeNum));
        //LOGDEBUG<<"uData16[0]:"<<uData16[0]<<"   uData16[1]:"<<uData16[1]<<"   writeNum:"<<writeNum;
    }
    else
    {
        qDebug("写入寄存器类型错误");
        return false;
    }
    //LOGDEBUG<<"writeModbusData typeNum:"<<typeNum<<"   writeNum:"<<writeNum;
    if(auto *reply = m_master.sendWriteRequest(writeUnit,1))
    {
        if(!reply->isFinished())
        {
            connect(reply,&QModbusReply::finished,this,[reply]()
            {
                if(reply->error() == QModbusDevice::NoError)
                {
                    reply->deleteLater();
                    return true;
                }
                else
                {
                    qDebug("写入返回错误: %s", reply->error());
                    reply->deleteLater();
                    return false;
                }
            });
        }
        else
        {
            reply->deleteLater();
            return false;
        }
    }
    else
    {
        qDebug("写入错误: %s" ,m_master.errorString().toStdString().c_str());
        return false;
    }
    return true;
}


//监听TCP连接的状态,若状态发生改变,发出对应的信号
void ModbusPort::slot_stateChanged()
{
    qDebug("myClient.state()");
    if(m_master.state() == QModbusDevice::ConnectedState)
    {
        emit signal_stateChanged(true);
    }
    else if(m_master.state() == QModbusDevice::UnconnectedState)
    {
        emit signal_stateChanged(false);
    }
}

//接收到读取线圈/离散输入寄存器请求后执行的槽函数
void ModbusPort::slot_readReadyCoils()
{
    QVector<quint16> vAllData;
    QModbusReply *reply = qobject_cast<QModbusReply *>(sender());
    if(!reply)
    {
        qDebug("读取线圈/离散输入寄存器错误");
        return;
    }
    if(reply->error() == QModbusDevice::NoError)
    {
        const QModbusDataUnit unit = reply->result();
        vAllData = unit.values();
        qDebug("read ceil %d", vAllData);
        emit signal_readCoils(vAllData);
    }
    else
    {
        qDebug("线圈/离散输入寄存器回复错误: %s", reply->error());
    }
    reply->deleteLater();
}

//接收到读取保持/输入寄存器请求后执行的槽函数
void ModbusPort::slot_readReadyRegisters()
{
    QModbusReply *reply = qobject_cast<QModbusReply *>(sender());
    if(!reply)
    {
        qDebug("读取保持/输入寄存器错误");
        return;
    }
    if(reply->error() == QModbusDevice::NoError)
    {
        const QModbusDataUnit unit = reply->result();
        auto valueList = unit.values();
        int nSize = valueList.size();
        if(nSize == 2)
        {
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
        }
        else
        {
            qDebug("保持寄存器返回数据错误,个数: %d", nSize);
        }
    }
    else
    {
        qDebug("保持/输入寄存器回复错误: %s", reply->error());
    }
    reply->deleteLater();
}