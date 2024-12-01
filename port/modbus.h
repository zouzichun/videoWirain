#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>
#include <QByteArray>
#include <QTimer>
#include <QSerialPort>
#include <QtSerialBus>
#include <QModbusDataUnit>
#include <QModbusClient>
#include <QModbusDevice>
 
#include <iostream>
#include <functional>
#include "port.h"

struct ModbusInfo
{
    quint32 buadrate;
};

class ModbusPort : public Port
{
    Q_OBJECT
public:
    explicit ModbusPort(QWidget *parent = nullptr);
    ~ModbusPort();
 
    virtual int startPort(const ConfigData &configData);
    virtual void closePort();
    // virtual int sendMsg(int idx);

    virtual bool readModbusData(int typeNum,int startAdd,quint16 numbers);
    virtual bool writeModbusData(int typeNum,int startAdd,uint32_t writeNum);
 
 signals:
    void signal_stateChanged(bool flag);
    void signal_readCoils(QVector<quint16> vAllData);
    void signal_readRegisters(int resultNum);

private slots:
    void slot_stateChanged();
    void slot_readReadyCoils();
    void slot_readReadyRegisters();

private:
    QString m_server_name;
    int buad_rate;
    QSerialPort m_serial;
    QModbusRtuSerialMaster m_master;
};


#endif // _MODBUS_H_
