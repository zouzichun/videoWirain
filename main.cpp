#include "maindialog.h"
#include <QtDebug>
#include <QMutex>
#include <QFile>
#include <QDate>
#include <QDir>
#include <QApplication>
#include <QtWidgets/QTextBrowser>
#include <sstream>

std::shared_ptr<spdlog::logger> g_logger;
QTextBrowser *logOut = nullptr;
void LogMsgOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);
void spdLogInit();

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    spdLogInit();
    qInstallMessageHandler(LogMsgOutput);
    qRegisterMetaType<QTextCursor>("QTextCursor");
    MainDialog w;
    w.show();

    qDebug("main started!");
    int ret = a.exec();
    // w.saveConfigFile();
    return ret;
}


void spdLogInit() {
    std::vector<spdlog::sink_ptr> sinks;
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::level_enum::debug);
    console_sink->set_pattern("[%^%l%$][%H:%M:%S:%e][%n] %v");
    sinks.push_back(console_sink);

//    auto is_truncate_log_file = false;
//    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("debug_log.txt", is_truncate_log_file);
//    file_sink->set_level(spdlog::level::level_enum::debug);
//    file_sink->set_pattern("[%C-%m-%d %z %H:%M:%S:%e] %v");
//    sinks.push_back(file_sink);

    g_logger = std::make_shared<spdlog::logger>("server", begin(sinks), end(sinks));
    g_logger->set_level(spdlog::level::level_enum::debug);
    spdlog::register_logger(g_logger);
    spdlog::set_default_logger(g_logger);
}

void LogMsgOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    static QMutex mutex;
    mutex.lock();

    QByteArray localMsg = msg.toLocal8Bit();

    QString strMsg("");
    switch(type)
    {
        case QtDebugMsg:
            strMsg = QString("Debug:");
            break;
        case QtWarningMsg:
            strMsg = QString("Warning:");
            break;
        case QtCriticalMsg:
            strMsg = QString("Critical:");
            break;
        case QtFatalMsg:
            strMsg = QString("Fatal:");
            break;
        default:
            break;
    }
    QString strDateTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString strMessage = QString("%1  File:%2  Line:%3  Function:%4  DateTime:%5 Message:%6")
            .arg(strMsg).arg(context.file).arg(context.line).arg(context.function).arg(strDateTime).arg(localMsg.constData());
//    QFile file("log.txt");
//    file.open(QIODevice::ReadWrite | QIODevice::Append);
//    QTextStream stream(&file);
//    stream << strMessage << "\r\n";
    if (logOut) {
        logOut->append(QString(localMsg.constData()));
    }
    std::cout << QString(localMsg.constData()).toStdString() << std::endl;

//    file.flush();
//    file.close();
    mutex.unlock();
}
