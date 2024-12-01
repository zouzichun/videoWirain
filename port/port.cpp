#include "port.h"
#include "maindialog.h"
#include <QDebug>
#include <QMessageBox>
#include <QTimer>
#include <QtEndian>
#include "crcalgorithm.h"
#include <memory>

Port::Port(QWidget *parent): QObject(parent)
{
    pWidget =  parent;
    mIoTimer = new QTimer(parent);
    mIoTimer->setSingleShot(true);
    connect(mIoTimer, SIGNAL(timeout()), this, SLOT(onIoTimeout()));
    msgBoxIoTimeout = new QMessageBox(QMessageBox::Critical, "ERROR", "Wait Peply Timeout.");
    msgBoxIoTimeout->setModal(false);
    send_buff_mgr = std::make_shared<BufferManager>(8);
    recv_buff_mgr = std::make_shared<BufferManager>(16);
}
Port::~Port()
{
}

int Port::startPort(const ConfigData &configData)
{
    Q_UNUSED(configData)
    qDebug()<<"startPort() should not be called";
    return 0;
}

void Port::closePort()
{
    qDebug()<<"closePort() should not be called";
}

int Port::sendMsg(int idx)
{
    Q_UNUSED(idx)
    qDebug()<<"senMsg() should not be called";
    return 0;
}

void Port::onIoTimeout()
{
    qDebug()<<"Wait Peply TimeOut";
    //参考//QMessageBox::critical(mainWindow, tr("ERROR"), tr("I/O Timeout."));
    msgBoxIoTimeout->show();
//    mutex.lock();
//    recvData.clear();
//    mutex.unlock();
}

unsigned short Port::copyAndCrc16(unsigned char * src_buff, char * dst_buff, unsigned int size) {
    unsigned char  ucCRCHi = 0xFF;
    unsigned char  ucCRCLo = 0xFF;
    int  iIndex;
    dst_buff += 8;

    while(size--) {
        unsigned char tt = *(src_buff++);
        *dst_buff++ = tt;
        iIndex = ucCRCLo ^ tt;
        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }

    return (unsigned short)( ucCRCHi << 8 | ucCRCLo );
}

int Port::formFrame(unsigned char command, unsigned int size, void * buff)
{
    int buff_idx = send_buff_mgr->GetAvaliableBuff();
    if (buff_idx == -1) {
        qDebug() << "formFrame no valid send buffer!";
        return -1;
    }

    QByteArray & buf_ptr = send_buff_mgr->GetBuffPtr(buff_idx);
    buf_ptr.resize(size + 10);
    int pos = 0;
    buf_ptr[pos++] = 0x55;
    buf_ptr[pos++] = 0xaa;
    buf_ptr[pos++] = command;
    buf_ptr[pos++] = static_cast<char>(0);
    unsigned int size_bp = size;
    for(int idx = 0; idx < 4; idx++) {
        buf_ptr[pos++] = size & 0xff;
        size = size >> 8;
    }
    unsigned short crc_16 = 0;
    if (buff) {
        crc_16 = copyAndCrc16((unsigned char *)buff, buf_ptr.data(), size_bp);
    }
    pos += size_bp;
    buf_ptr[pos++] = crc_16 & 0xff;
    buf_ptr[pos++] = (crc_16 >> 8)& 0xff;

//    for(int idx = 0; idx < 8; idx++) {
//        qDebug("buf_ptr pos: %2x = %2x", idx, buf_ptr[idx]);
//    }
    
//    unsigned short crc = mbCrc16((unsigned char*)sendData.data(),sendData.size());//计算CRC
//    sendData.push_back(crc & 0x00FF);//MODBUS: CRC的相反，LSB在前MSB在后
//    sendData.push_back((crc >> 8) & 0x00FF);//填充CRC

    if(configData.isReplyTimeout)
    {
        if(!mIoTimer->isActive())
            mIoTimer->start(5000);
    }

    send_buff_mgr->IncCounter();
    qDebug() << "Msgformed count: " << send_buff_mgr->GetCounter();

    return 0;
}

void Port::parseMsg(int buff_idx)
{
    int index=0;
    QByteArray & pPortRecv = recv_buff_mgr->GetBuffPtr(buff_idx);

    if (pPortRecv.length() < 10) {
        qDebug("invalid recv buff len %d.", pPortRecv.length());
        recv_buff_mgr->ReleaseBuff(buff_idx);
        return;
    }

    for(index=0; index < pPortRecv.length() - 1; index++)
    {
        unsigned char sof = '0', sof1 = '0';
        if (index == 0) {
            sof = pPortRecv.at(index);
            sof1 = pPortRecv.at(index+1);
        } else {
            sof = sof1;
            sof1 = pPortRecv.at(index+1);
        }
        if((unsigned char)0x55 == sof &&
            (unsigned char)0xaa == sof1)
            break;
    }

    if(0 != index)
    {
        qDebug("start of frame not 0x55aa:%X, all size %d.", index, pPortRecv.length());
        pPortRecv.remove(0,index);
    }

    unsigned int data_len = 0;
    for (int pos = 7; pos >=4; pos--) {
        data_len = data_len << 8;
        data_len |= pPortRecv.at(pos);
    }

    int pos = pPortRecv.size() - 2;
    unsigned short r_crc = static_cast<unsigned char>(pPortRecv.at(pos + 1)) << 8 | static_cast<unsigned char>(pPortRecv.at(pos));

    unsigned short check_crc = mbCrc16(pPortRecv.data() + 8, data_len);
    if(r_crc != check_crc) {
        qCritical("rcrc 0x%4X mismatch with cal_crc 0x%4X", r_crc, check_crc);}
//    } else {
//        qDebug("rcrc check pass!");
//    }

    mIoTimer->stop();
    emit receivedSignal(buff_idx);
}
