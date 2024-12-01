#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QObject>
#include <QTcpServer>
#include <QString>
#include <QThread>
#include <QVector>
#include <QTcpSocket>
#include <QByteArray>
#include <QTimer>
 
#include <iostream>
#include <functional>

typedef std::function<void (QByteArray &msg,QTcpSocket* sock)> msgHandle;

struct SocketInfo
{
    quint32 socket_num;
    QThread socket_thread;
};

class TcpServer : public QTcpServer
{
    Q_OBJECT
public:
    explicit TcpServer(QString t_server_name,quint32 t_thread_cout,msgHandle t_func=nullptr, QObject *parent = nullptr);
    ~TcpServer();
 
    QString getServerName();
    quint16 getServerPort();
 
public slots:
    void setMsgHandle(msgHandle t_func);
    void startListen(quint16 t_port);
    void stopListen();
    void socketClose(qint32 t_id);
 
protected:
    void incomingConnection(qintptr socketDescriptor);
    qint32 getMinSocketThread();
 
private:
    QString m_server_name;
    quint16 m_port;
 
    quint32 m_thread_count;
    QVector<SocketInfo*> m_thread_vec;
    msgHandle m_func;
};

class TcpSocket : public QTcpSocket
{
    Q_OBJECT
public:
    explicit TcpSocket(qint32 id,qintptr socketDescriptor,msgHandle t_msg_handle=nullptr,QObject *parent = nullptr);
    ~TcpSocket();

signals:
    void socketClose(qint32);

public slots:
    void msgTimeout();
    void readMessage();
    void socketErr(QAbstractSocket::SocketError tError);
    void socketdisconnect();

private:
    qint32 m_id;
    msgHandle m_msg_handle;
    QTimer *m_msg_timeout;
};

#endif // TCPSERVER_H
