#include "serial_port.h"
#include <queue>

UartBuffQueue uart_rx_queue;
UartBuffQueue uart_tx_queue;

volatile int input_pulse[3] = {};
volatile int curr_position[3] = {};
volatile quint16 encoder_val[3] = {};

SerialPort::SerialPort(): QObject()
{
    uart_tx_queue.clear();
    uart_rx_queue.clear();
    m_serial = new QSerialPort();
    m_timer = new QTimer(this);
    m_tx_thread = new TxProcessThread(this);
    // connect(m_timer, &QTimer::timeout,this,&SerialPort::sendTimeout);
    connect(m_tx_thread, &QThread::finished, this, &QObject::deleteLater);
    // connect(m_serial, &QSerialPort::readyRead, this, &SerialPort::slotRecvPort);
    connect(m_serial, &QSerialPort::bytesWritten, this, &SerialPort::handleBytesWritten);
    m_tx_thread->start();
}

SerialPort::~SerialPort()
{
    closePort();
    // disconnect(m_timer, &QTimer::timeout,this,&SerialPort::sendTimeout);
    // disconnect(m_serial, &QSerialPort::readyRead, this, &SerialPort::slotRecvPort);
    disconnect(m_serial, &QSerialPort::bytesWritten, this, &SerialPort::handleBytesWritten);

    m_tx_thread->quit();
    m_tx_thread->wait();
    delete m_serial;
    delete m_timer;
}

void SerialPort::sendTimeout()
{
}

int SerialPort::startPort(ConfigData &configData)
{
    if(isOpened) {
        qDebug()<<"serial port is already opened, Close first!";
        closePort();
    }

    sendCount = 0;
    recvCount = 0;
    m_serial->setPortName(configData.serialName);
    m_serial->setBaudRate(configData.baudRate);
    if (!m_serial->open(QIODevice::ReadWrite)) {
        qDebug()<<tr("Can't open %1, error is %2").arg(configData.serialName).arg(m_serial->errorString());//发送错误给协议层，协议层弹框显示
        return -1;
    }else {
        qDebug()<<"serial is opened true";
        m_serial->flush();
    }
    isOpened = true;
    return 0;
}

void SerialPort::closePort()
{
    m_serial->clear();
    m_serial->close();
    isOpened = false;
    qDebug()<<"close serial port";
}

void SerialPort::motorDrive(uint8_t motor_id, uint8_t dir, uint16_t speed, uint32_t pulse_num) {
    QByteArray ba = {};
    uint16_t cmd = (dir << 12) | (speed & 0xfff);
    ba.append(motor_id);
    ba.append('\xfd');
    ba.append((const char)((cmd & 0xff00) >> 8));
    ba.append((const char)(cmd & 0xff));
    ba.append('\x0');
    ba.append((const char)((pulse_num & 0xff0000) >> 16));
    ba.append((const char)((pulse_num & 0xff00) >> 8));
    ba.append((const char)(pulse_num & 0xff));
    ba.append('\x6b');
    sendMsgSync(ba);
}

void SerialPort::motorDrive(uint8_t motor_id, uint8_t dir, uint16_t speed) {
    QByteArray ba = {};
    uint16_t cmd = (dir << 12) | (speed & 0xfff);
    ba.append(motor_id);
    ba.append('\xf6');
    ba.append((const char)((cmd & 0xff00) >> 8));
    ba.append((const char)(cmd & 0xff));
    ba.append('\x0');
    ba.append('\x6b');
    sendMsgSync(ba);
}

void SerialPort::motorStop(uint8_t motor_id) {
    QByteArray ba = {};
    uint16_t cmd = (1 << 12);
    ba.append(motor_id);
    ba.append('\xf6');
    ba.append((const char)((cmd & 0xff00) >> 8));
    ba.append((const char)(cmd & 0xff));
    ba.append('\x0');
    ba.append('\x6b');
    sendMsgSync(ba);
}

void SerialPort::sendMsg(const QByteArray &data)
{
    if (isOpened) {
        const qint64 written = m_serial->write(data);
        if (written == data.size()) {
            m_bytesToWrite += written;
            QString str;
            str = QString("Serial Tx :(%1):").arg(data.count());
            for(int i=0; i<data.count(); i++)
             str += QString(" %1").arg((unsigned char)data.at(i),2,16,QLatin1Char('0'));
            qDebug()<<str;

            // m_timer->start(100);
        } else {
            const QString error = tr("Failed to write all data to port %1.\n"
                                    "Error: %2").arg(m_serial->portName(),
                                                    m_serial->errorString());
            qDebug() << error;
        }
    } else {
        qDebug()<<"Serial Port is not open";
    }
}

void SerialPort::sendMsgSync(const QByteArray &data)
{
    if (isOpened) {
        // qDebug()<< "send msg sync thd id: " << QThread::currentThreadId();
        const qint64 written = m_serial->write(data);
        if (written == data.size()) {
            m_bytesToWrite += written;
            if (m_serial->waitForBytesWritten(100)) {
                QString str;
                str = QString("Serial Tx :(%1):").arg(data.count());
                for(int i=0; i<data.count(); i++)
                 str += QString(" %1").arg((unsigned char)data.at(i),2,16,QLatin1Char('0'));
                qDebug()<<str;
            } else {
                const QString error = tr("Failed to write data to port IO %1.\n"
                                        "Error: %2").arg(m_serial->portName(),
                                                        m_serial->errorString());
                qDebug() << error;
            }
            // m_timer->start(100);
        } else {
            const QString error = tr("Failed to write all data to port %1.\n"
                                    "Error: %2").arg(m_serial->portName(),
                                                    m_serial->errorString());
            qDebug() << error;
        }
    } else {
        qDebug()<<"Serial Port is not open";
    }
}

void SerialPort::sendMsgWait(const QByteArray &data) {
    m_tx_thread->sendProcess(this, data);
}

void SerialPort::waitMsg(QByteArray &r_data)
{
    r_data.clear();
    if (isOpened) {
        if (m_serial->waitForReadyRead(300)) {
            r_data = m_serial->readAll();
            if(!r_data.isEmpty())
            {
                recvCount++;
                qDebug()<< "slot wait msg thd id: " << QThread::currentThreadId();
                QString str = QString("Serial RX wait message :(%1):").arg(r_data.count());
                for(int i=0; i<r_data.count(); i++)
                 str += QString(" %1").arg((unsigned char)r_data.at(i),2,16,QLatin1Char('0'));
                qDebug()<<str;
            }
        }
    }
}

void SerialPort::handleBytesWritten(qint64 bytes)
{
    m_bytesToWrite -= bytes;
    if (m_bytesToWrite == 0) {
        sendCount++;
        // m_timer->stop();
    }
}

//void SerialPort::slotRecvPort()
//{
//    qDebug()<< "slot recv thd id: " << QThread::currentThreadId();
//    QByteArray tmp = m_serial->readAll();
////    while (m_serial->waitForReadyRead(1))
////        tmp += m_serial->readAll();
//    if(!tmp.isEmpty())
//    {
//        recvCount++;
//    }
//}

TxProcessThread::TxProcessThread(QObject *parent) : QThread(parent),
    m_mutex(),
    m_cond(),
    m_quit(false),
    m_send_val() {
}

TxProcessThread::~TxProcessThread() {};

void TxProcessThread::sendProcess(SerialPort * serial, const QByteArray & val) {
    m_serial = serial;
    m_send_val = val;
    m_cond.wakeOne();
}

void TxProcessThread::run() {
    qDebug()<< "tx thd start id: " << QThread::currentThreadId();
    while (!m_quit) {
        m_mutex.lock();
        m_cond.wait(&m_mutex);

        if(m_send_val.count() != 0) {
            QByteArray tmp;
            if (m_send_val.size() == 3) {
                m_serial->sendMsgSync(m_send_val);
                m_serial->waitMsg(tmp);
                if (m_send_val[1] == '\x36') {
                    if (tmp.size() == 6) {
                        unsigned char idx = tmp[0] % 3;
                        unsigned int tt = ((unsigned int)tmp[1]) << 24;
                        tt |= ((unsigned int)tmp[2]) << 16;
                        tt |= ((unsigned int)tmp[3]) << 8;
                        tt |= ((unsigned int)tmp[4]);
                        curr_position[idx] = (int)(tt);
                    }
                } else if (m_send_val[1] == '\x33'){
                    if (tmp.size() == 6) {
                        unsigned char idx = tmp[0] % 3;
                        unsigned int tt = ((unsigned int)tmp[1]) << 24;
                        tt |= ((unsigned int)tmp[2]) << 16;
                        tt |= ((unsigned int)tmp[3]) << 8;
                        tt |= ((unsigned int)tmp[4]);
                        input_pulse[idx] = (int)(tt);
                    }
                } else if (m_send_val[1] == '\x30'){
                    if (tmp.size() == 4) {
                        unsigned char idx = tmp[0] % 3;
                        quint16 tt = ((quint16)tmp[1]) << 8;
                        tt |= ((quint16)tmp[2]);
                        encoder_val[idx] = tt;
                    }
                } else {

                }
            }
        }
        m_mutex.unlock();
    };
}


