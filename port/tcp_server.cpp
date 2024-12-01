#include "tcp_server.h"
#include <QByteArray>
#include <QHostAddress>
#include <QNetworkProxyFactory>
#include <QDebug>
#include <QTcpSocket>
#include "crcalgorithm.h"

void rcvFunc(QByteArray &buf, QTcpSocket *socket) {
    if (buf.length() < 10) {
        qDebug("invalid recv buff len %d.", buf.length());
        return;
    }

    int index = 0;
    for(index = 0; index < buf.length() - 1; index++)
    {
        unsigned char sof = '0', sof1 = '0';
        if (index == 0) {
            sof = buf.at(index);
            sof1 = buf.at(index+1);
        } else {
            sof = sof1;
            sof1 = buf.at(index+1);
        }
        if((unsigned char)0x55 == sof &&
            (unsigned char)0xaa == sof1)
            break;
    }

    if(0 != index)
    {
        qDebug("start of frame not 0x55aa:%X, all size %d.", index, buf.length());
        buf.remove(0,index);
    }

    if (buf.size() < 10) {
        qCritical("rcrc lenght %d invalid", buf.size());
        return;
    }

    unsigned int data_len = 0;
    for (int pos = 7; pos >=4; pos--) {
        data_len = data_len << 8;
        data_len |= buf.at(pos);
    }

    int pos = buf.size() - 2;
    unsigned short r_crc = static_cast<unsigned char>(buf.at(pos + 1)) << 8 | static_cast<unsigned char>(buf.at(pos));

    unsigned short check_crc = mbCrc16(buf.data() + 8, data_len);
    if(r_crc != check_crc) {
        qCritical("rcrc 0x%4X mismatch with cal_crc 0x%4X", r_crc, check_crc);}
//    } else {
//        qDebug("rcrc check pass!");
//    }
};

TcpServer::TcpServer(QString t_server_name,quint32 t_thread_cout,msgHandle t_func,QObject *parent) :
    QTcpServer(parent),
    m_server_name(t_server_name),
    m_thread_count(t_thread_cout),
    m_func(t_func)
{
    this->m_port=0; 
    for(quint32 index=0;index<this->m_thread_count;index++)
    {
        SocketInfo *tmp_info=new SocketInfo();
        tmp_info->socket_num=0;
        tmp_info->socket_thread.start();
        this->m_thread_vec.append(tmp_info);
    }
}

TcpServer::~TcpServer()
{
    if(this->isListening())
    {
        this->close();
    }
 
    int cout=this->m_thread_vec.size();
    for(int index=0;index<cout;index++)
    {
        this->m_thread_vec[index]->socket_thread.terminate();
        this->m_thread_vec[index]->socket_thread.wait();
        delete this->m_thread_vec[index];
        this->m_thread_vec[index]=nullptr;
    }
}

QString TcpServer::getServerName()
{
    return this->m_server_name;
}

quint16 TcpServer::getServerPort()
{
    return this->m_port;
}

void TcpServer::setMsgHandle(msgHandle t_func)
{
    this->m_func=t_func;
}

void TcpServer::startListen(quint16 t_port)
{
    if(this->isListening())
    {
        this->close();
    }
 
    if(this->listen(QHostAddress::Any,t_port))
    {
        qDebug()<<__FILE__<<":"<<__LINE__<<":"<<"listen successful:"<<t_port;
        this->m_port=t_port;
    }
    else
    {
        qDebug()<<__FILE__<<":"<<__LINE__<<":"<<"listen failed:"<<t_port;
    }
}

void TcpServer::stopListen()
{
    qDebug()<<__FILE__<<":"<<__LINE__<<":"<<"stop listen: "<<this->m_port;
    if(this->isListening())
    {
        this->close();
    }
}

void TcpServer::incomingConnection(qintptr socketDescriptor)
{
    qDebug()<<__FILE__<<":"<<__LINE__<<":"<<"new connection: "<<socketDescriptor;
    qint32 cut_id=this->getMinSocketThread();
 
    TcpSocket *tmp_soc=new TcpSocket(cut_id,socketDescriptor,this->m_func);
    connect(tmp_soc,SIGNAL(socketClose(qint32)),this,SLOT(socketClose(qint32)));
    tmp_soc->moveToThread(&(this->m_thread_vec[cut_id]->socket_thread));
    this->m_thread_vec[cut_id]->socket_num++;
}

void TcpServer::socketClose(qint32 t_id)
{
    this->m_thread_vec[t_id]->socket_num--;
}

qint32 TcpServer::getMinSocketThread()
{
    qint32 tmp_id=0;
    qint32 th_cout=this->m_thread_vec.size();
    for(qint32 index=0;index<th_cout;index++)
    {
        if(this->m_thread_vec[index]->socket_num < this->m_thread_vec[tmp_id]->socket_num)
        {
            tmp_id=index;
        }
    }
    return tmp_id;
}

static const qint32 TIMEOUT_VALUE=30000;
 
TcpSocket::TcpSocket(qint32 id,qintptr socketDescriptor,msgHandle t_msg_handle,QObject *parent) :
    QTcpSocket(parent),
    m_id(id),
    m_msg_handle(t_msg_handle)
{
    m_msg_timeout =new QTimer(this);
    connect(this->m_msg_timeout,SIGNAL(timeout()),this,SLOT(msgTimeout()));
    this->setSocketDescriptor(socketDescriptor);
    connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(socketErr(QAbstractSocket::SocketError)));
    connect(this,SIGNAL(disconnected()),this,SLOT(socketdisconnect()));
    connect(this, SIGNAL(readyRead()), this, SLOT(readMessage()));
 
    this->m_msg_timeout->start(TIMEOUT_VALUE);
}
 
TcpSocket::~TcpSocket()
{
    this->disconnectFromHost();
}
 
void TcpSocket::msgTimeout()
{
    qDebug() << __FILE__ <<":"<<__LINE__<< ":" << "socket connect free timeout";
    this->disconnectFromHost();
}
 
void TcpSocket::readMessage()
{
    this->m_msg_timeout->stop();
    QByteArray t_message=this->readAll();
    if(this->m_msg_handle!=nullptr)
    {
        m_msg_handle(t_message,this);
    }
 
    this->m_msg_timeout->start(TIMEOUT_VALUE);
}
 
void TcpSocket::socketErr(QAbstractSocket::SocketError tError)
{
    qDebug()<<__FILE__<<":"<<__LINE__<<":"<<"id: "<<this->socketDescriptor()<<"socket error: "<<this->errorString();
    switch (tError)
    {
    case QAbstractSocket::RemoteHostClosedError:
        this->disconnectFromHost();
        break;
    default:
        this->disconnectFromHost();
        break;
    }
}
 
void TcpSocket::socketdisconnect()
{
    emit socketClose(this->m_id);
    this->deleteLater();
}
