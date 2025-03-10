#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <QThread>

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(QObject *parent = nullptr)
    {
        Q_UNUSED(parent)
    }

signals:
    void resultReady(const QVector<QVector<QString> > &result); //返回处理结果信号

public slots:
    void doWork(const QString &cmd) //接收页面命令槽函数
    {
        Q_UNUSED(cmd)
        // QVector< QVector<QString> > result;
        // for (int i = 0; i < 1000000; ++i)
        // {
        //     QVector<QString> tmp(10);
        //     tmp[0] = QString::number(i+1);
        //     tmp[1] = QString("test%1").arg(i+1);
        //     tmp[2] = tmp[1];
        //     tmp[7] = "1";
        //     tmp[8] = "";
        //     tmp[9] = "";
        //     result.push_back(tmp);
        // }
        // Sleep(1000*3); //模拟耗时操作
        emit resultReady(result);
    }

private:
};

class CDataClass : public QObject
{
    Q_OBJECT
public:
    explicit CDataClass(QObject *parent = nullptr)
    {
        Q_UNUSED(parent)
        Worker *worker = new Worker; //定义数据处理类
        worker->moveToThread(&mWorkerThread); //把数据处理类移到线程
        connect(&mWorkerThread, &QThread::finished, worker, &QObject::deleteLater);
        //定义信号槽，把命令发送到线程里面的槽函数
        connect(this, &CDataClass::operate, worker, &Worker::doWork);
        //定义信号槽，接收线程里面发送的结果
        connect(worker, &Worker::resultReady, this, &CDataClass::handleResults);
        mWorkerThread.start(); //开启线程
    }

    ~CDataClass()
    {
        mWorkerThread.quit();
        mWorkerThread.wait();
    }

signals:
    //把页面接受的命令，发送到线程里面的槽函数
    void operate(const QString &cmd);
    //把线程里面的处理结果返回给页面
    void operateResult(const QVector<QVector<QString> > &result);

public slots:
     void handleResults(const QVector<QVector<QString> > &result) //接受线程里面处理结果
     {
        emit operateResult(result);
     }

private:
    QThread mWorkerThread; //定义处理线程
};

