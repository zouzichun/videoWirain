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

    virtual bool readModbusData(int typeNum,int startAdd,quint16 numbers);
    bool writeModbusData(int typeNum,int startAdd,uint32_t writeNum) {
                return true;
    };
    virtual bool writeModbusData(int typeNum,int startAdd, float write_val);
    virtual bool waitDataReady();

    volatile bool rdy_flag;
    volatile float rdy_data;

 signals:
    void signal_stateChanged(bool flag);
    void signal_readCoils(QVector<quint16> vAllData);
    void signal_readRegisters(int resultNum);

private slots:
    void slot_stateChanged();
    void slot_readReadyCoils();
    void slot_readReadyRegisters();

private:
    std::mutex mtx;
    QString tcp_ip;
    uint32_t tcp_port;
    QModbusTcpClient * m_modbustcp;
};

#endif // _MODBUS_TCP_H_
