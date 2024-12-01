#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <QObject>
#include <QTcpServer>
#include <QString>
#include <QThread>
#include <QVector>
#include <QTcpSocket>
#include <QByteArray>
#include <QTimer>
#include <QTime>
#include <QObject>
#include <QMutex>
#include <QWaitCondition>

#include <iostream>
#include <functional>

#include "comdata.h"
#include <QSerialPort>
#include <queue>

class QSerialPort;
class QTimer;
class QMessageBox;

class UartBuffQueue : public QObject {
    Q_OBJECT

public:
    UartBuffQueue() = default;
    virtual ~UartBuffQueue() = default;

    bool empty() {
        std::lock_guard<std::mutex> lock(mtx);
        return cnt == 0;
    };

    int size() {
        std::lock_guard<std::mutex> lock(mtx);
        return cnt;
    };

    void push(const QByteArray in_val) {
        std::lock_guard<std::mutex> lock(mtx);
        cnt++;
        rx_queue.push(in_val);
    };

    const QByteArray pop() {
        std::lock_guard<std::mutex> lock(mtx);
        if (cnt > 0) {
            const QByteArray tt = rx_queue.front();
            rx_queue.pop();
            cnt--;
            return tt;
        }
        return QByteArray();
    };

    const QByteArray front() {
        std::lock_guard<std::mutex> lock(mtx);
        return rx_queue.front();
    };

    const QByteArray back() {
        std::lock_guard<std::mutex> lock(mtx);
        return rx_queue.back();
    };

    void clear() {
        std::lock_guard<std::mutex> lock(mtx);
        if (cnt != 0) {
            std::queue<QByteArray> tq;
            rx_queue.swap(tq);
            cnt = 0;
        }
    };

private:
    std::mutex mtx = {};
    volatile int cnt = 0;
    std::queue<QByteArray> rx_queue = {};
};

class SerialPort;
class TxProcessThread : public QThread
{
    Q_OBJECT

public:
    explicit TxProcessThread(QObject *parent = nullptr);
    ~TxProcessThread();

public slots:
    void sendProcess(SerialPort * serial, const QByteArray & val);

signals:
    void error(const QString &s);

private:
    void run() override;

    QMutex m_mutex;
    QWaitCondition m_cond;
    bool m_quit = false;
    SerialPort * m_serial = nullptr;
    QByteArray m_send_val;
};

//class RxProcessThread : public QThread
//{
//    Q_OBJECT

//public:
//    explicit RxProcessThread(QObject *parent = nullptr);
//    ~RxProcessThread();

//    void process();

//signals:
//    void error(const QString &s);

//private:
//    void run() override;

//    QMutex m_mutex;
//    QWaitCondition m_cond;
//    bool m_quit = false;
//};

class SerialPort : public QObject
{
    Q_OBJECT
public:
    explicit SerialPort();
    virtual ~SerialPort();

    int startPort(ConfigData &configData);
    void closePort();
    void sendMsg(const QByteArray &data);
    void waitMsg(QByteArray &r_data);
    void sendMsgSync(const QByteArray &data);

    void motorDrive(uint8_t motor_id, uint8_t dir, uint16_t speed, uint32_t pulse_num);
    void motorDrive(uint8_t motor_id, uint8_t dir, uint16_t speed);
    void motorStop(uint8_t motor_id);

    qint64 sendCount = 0;
    qint64 recvCount = 0;
    qint64 m_bytesToWrite = 0;
    bool isOpened = false;
    QSerialPort* m_serial = nullptr;
    QTimer * m_timer = nullptr;

    TxProcessThread * m_tx_thread = nullptr;

    QMutex m_mutex = {};
    QWaitCondition m_cond = {};

public slots:
    void sendMsgWait(const QByteArray &data);
//    void slotRecvPort();
    void handleBytesWritten(qint64 bytes);
    void sendTimeout();
};

#endif // SERIAL_PORT_H
