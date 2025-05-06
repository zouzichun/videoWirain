#ifndef _MODBUS_TCP_H_
#define _MODBUS_TCP_H_

#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>
#include <QByteArray>
#include <QTimer>
#include <QModbusDataUnit>
#include <QModbusTcpClient>
 
#include <iostream>
#include <functional>
#include "port.h"

class ModbusTcp : public Port
{
    Q_OBJECT
public:
    explicit ModbusTcp(QWidget *parent = nullptr);
    ~ModbusTcp();

    virtual int startPort(const ConfigData &configData);
    virtual void closePort();
    // virtual int sendMsg(int idx);

    virtual bool readModbusData(int startAdd, int numbers, float &val);
    virtual bool writeModbusData(int startAdd, int numbers, float val);
    virtual bool waitDataReady();

 signals:
    // void signal_stateChanged(bool flag);
    void signal_UpdateReadData(float val);

private slots:
    void handleStateChanged(QModbusDevice::State state);
    void handleErrorOccurred(QModbusDevice::Error error);
    void handleDataReady(float & val);

private:
    QString tcp_ip;
    uint32_t tcp_port;
    QModbusTcpClient * m_modbustcp;
};

#endif // _MODBUS_TCP_H_
