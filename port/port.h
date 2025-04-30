#ifndef PORT_H
#define PORT_H

#include <QObject>
#include <QMutex>
#include "comdata.h"
#include <QHostAddress>
#include <memory>
#include <mutex>

class QTcpSocket;
class QTimer;
class QMessageBox;

#define BUFF_SIZE 4096
class BufferManager {
public:
    BufferManager(unsigned int buf_cnt) :
    buff_cnts_max(buf_cnt) {
        assert(buff_cnts_max != 0);
        for (unsigned int idx = 0; idx < buff_cnts_max; idx++) {
//            buff.push_back(QByteArray(0));
        }
    }

    BufferManager() :
    buff_cnts_max(8) {
        for (unsigned int idx = 0; idx < buff_cnts_max; idx++) {
//            buff.push_back(QByteArray(0));
        }
    }

    ~BufferManager() {
        for (unsigned int idx = 0; idx < buff_cnts_max; idx++) {
        }
    }

    QByteArray & GetBuffPtr(int pos) {
        assert(static_cast<unsigned int>(pos) < buff_cnts_max);
        return buff[pos];
    }

    int GetAvaliableBuff() {
        std::lock_guard<std::mutex> guard(buff_mtx);
        for (unsigned int idx = 0; idx < buff_cnts_max; idx++) {
            if ((buff_status & (1 << idx)) == 0) {
                buff_status |= 1 << idx;
                return static_cast<int>(idx);
            }
        }
        return -1;
    }

    void ReleaseBuff(int idx) {
        std::lock_guard<std::mutex> guard(buff_mtx);
        buff_status &= ~(1 << idx);
    }

    unsigned int GetBuffStatus() {
        return buff_status;
    }

    int TaskAvaliable() {
        std::lock_guard<std::mutex> guard(buff_mtx);
        int buff_idx = -1;
        for (unsigned int pos = 0; pos < buff_cnts_max; pos++) {
            if ((buff_status & (1 << pos)) != 0) {
                buff_idx = static_cast<int>(pos);
            }
        }

        return buff_idx;
    }

    void IncCounter() {
        std::lock_guard<std::mutex> guard(buff_mtx);
        counter++;
    }

    void ClrCounter() {
        std::lock_guard<std::mutex> guard(buff_mtx);
        counter = 0;
    }

    int GetCounter() {
        return counter;
    }

private:
    QByteArray buff[8];
    unsigned int buff_cnts_max = 8;
    unsigned int buff_status = 0;
    std::vector<unsigned int> task;
    int counter = 0;
    std::mutex buff_mtx;
};

class Port: public QObject
{
    Q_OBJECT
public:
    explicit Port(QWidget *parent = nullptr);
    virtual ~Port();

    virtual int startPort(const ConfigData &configData);
    virtual void closePort();
    virtual int sendMsg(int idx);

    int formFrame(unsigned char command, unsigned int size, void * buff);
    void parseMsg(int idx);

    virtual bool readModbusData(int startAdd, int numbers, float &val) = 0;
    virtual bool writeModbusData(int startAdd, int numbers, float val) = 0;
    virtual bool waitDataReady() {return true;};
    void ClearFlagData();
    void thd_msleep(uint32_t ms);
    
    QWidget* pWidget;
    // QMutex  mtx;
    std::mutex mtx;
    QTimer* mIoTimer;
    QMessageBox* msgBoxIoTimeout;
    bool isOpened = false;

    volatile bool rdy_flag;
    volatile float rdy_data;

    std::shared_ptr<BufferManager> send_buff_mgr;
    std::shared_ptr<BufferManager> recv_buff_mgr;

private:
    unsigned short copyAndCrc16(unsigned char * src_buff, char * dst_buff, unsigned int size);

public slots:
    void onIoTimeout();

signals:
    void receivedSignal(int buff_idx);
};

#endif // PORT_H
