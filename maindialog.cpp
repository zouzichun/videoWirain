#include "maindialog.h"
#include <ui_maindialog.h>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QTextCodec>
#include <QStyleFactory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;

extern volatile bool tx_rx_sync_flag;
extern volatile bool get_curr_position;
extern volatile bool get_input_pulse;
extern volatile bool get_encoder_val;
extern volatile int input_pulse[3];
extern volatile int curr_position[3];
extern quint16 encoder_val[3];

volatile int current_position_bias[3];
volatile int current_pulse_bias[3];
volatile quint16 current_encoder_bias[3];

ConfigData configData = defaultSetting;

extern QTextBrowser *logOut;
extern void rcvFunc(QByteArray &buf, QTcpSocket *socket);
extern std::vector<Lines> g_lines;

std::pair<double, double> X2_MACH;
std::pair<double, double> X1_MACH;
bool trigger_process = false;

extern bool debug_win_enable;
extern bool img_sw_status;

std::vector<cv::Point> roi_points;

#define TEST_CAMERA 1

MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::MainDialog),
    m_serial(nullptr),
    m_imgproc(nullptr),
    m_monitor_timer(nullptr)
{
    m_ui->setupUi(this);
    logOut = m_ui->txbRecv;

    //mvs camera
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    Qt::WindowFlags flags=Qt::Dialog;
    flags |=Qt::WindowMinMaxButtonsHint;
    flags |=Qt::WindowCloseButtonHint;
    setWindowFlags(flags);
    m_ui->txbRecv->document()->setMaximumBlockCount(2000);

    loadConfigFile();
    showUIConfigData(configData);

    // if (configData.netType == 3) {
    // m_port = new ModbusPort(parent);
    m_port = new ModbusTcp(parent);
    m_port->startPort(configData);
//     m_port->moveToThread(&mmodbusthd);
//     connect(&mmodbusthd, &QThread::finished, m_port, &QObject::deleteLater);

//    connect(this, SIGNAL(sendMsgWait(const QByteArray&)),
//                m_serial, SLOT(sendMsgWait(const QByteArray&)));

    m_monitor_timer =new QTimer(this);
    // connect(this->m_monitor_timer,SIGNAL(timeout()),this,SLOT(monitor_thread()));
    m_monitor_timer->stop();

    m_imgproc = new ImgProcess("image", 2048, 2048, true);
    m_imgproc->moveToThread(&mWorkerThread); //把数据处理类移到线程
    connect(&mWorkerThread, &QThread::finished, m_imgproc, &QObject::deleteLater);


    connect(m_imgproc, &ImgProcess::signal_refresh_delta, this, &MainDialog::calibration_refresh_delta, Qt::QueuedConnection); // 或 QueuedConnection
    connect(m_imgproc, &ImgProcess::signal_refresh_img, this, &MainDialog::main_img_refresh, Qt::QueuedConnection); // 或 QueuedConnection

    connect(this, &MainDialog::signal_auto_run, m_imgproc, &ImgProcess::AutoRunSlot, Qt::DirectConnection);
    connect(this, &MainDialog::signal_trigger, m_imgproc, &ImgProcess::TriggerSlot, Qt::DirectConnection);
    connect(m_imgproc, &ImgProcess::signal_read_modbus_data, this, &MainDialog::read_modbus_data, Qt::DirectConnection);
    
    #if TEST_CAMERA
        connect(this, &MainDialog::cameraStart, m_imgproc, &ImgProcess::CameraTest);
    #else
        connect(this, &MainDialog::cameraStart, m_imgproc, &ImgProcess::ImageTest);
    #endif

    imgw = new imgWindow();
    #if TEST_CAMERA
        connect(this, &MainDialog::cameraCalStart, m_imgproc, &ImgProcess::CameraCalTest, Qt::QueuedConnection);
    #else
        connect(this, &MainDialog::cameraCalStart, m_imgproc, &ImgProcess::ImageCalTest, Qt::QueuedConnection);
    #endif
    connect(m_imgproc, &ImgProcess::signal_refresh_cal_img, imgw, &imgWindow::calibration_refresh, Qt::QueuedConnection);

    m_hWnd = (void*)m_ui->painter->winId();
    connect(m_ui->modbus_delay,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->monitor_delay,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->motor_rho,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->x1_start,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->x2_start,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->x2_rho,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->y1_start,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->target_delta,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->fetch_delta,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->y_fetch_delta,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    connect(m_ui->y_target_delta,&QLineEdit::editingFinished,this,&MainDialog::on_cal_editingFinished);
    CameraInit();

    mWorkerThread.start();
//     mmodbusthd.start();

    emit signal_auto_run(m_ui->auto_run->checkState());

    qDebug()<< "MainDialog thd id: " << QThread::currentThreadId();
}

MainDialog::~MainDialog()
{
    delete m_ui;
    if (imgw)
        delete imgw;

    if (m_serial)
        delete m_serial;
    if (configData.netType == 3) {
        m_port->closePort();
        delete m_port;
    }

    for (auto v : m_cameras) {
        v.timer->stop();
        delete v.timer;

        if (v.handler) {
            v.handler->Close();
            delete v.handler;
            v.handler = nullptr;
        }
    }

    mWorkerThread.quit();
    mWorkerThread.wait();
//     mmodbusthd.quit();
//     mmodbusthd.wait();
    m_monitor_timer->stop();
    delete m_monitor_timer;
}

void MainDialog::saveCommParam()
{
    QSettings sets(QCoreApplication::applicationName()+".ini",QSettings::IniFormat);
    sets.setValue("netType",configData.netType);
    sets.setValue("ip",configData.ip);
    sets.setValue("portAddr",configData.portAddr);
    sets.setValue("replyTimeout",configData.isReplyTimeout);
    sets.setValue("serialName",configData.serialName);
    sets.setValue("baudRate",configData.baudRate);
    sets.sync();
}

void MainDialog::saveImgParam()
{
    QSettings sets(QCoreApplication::applicationName()+".ini",QSettings::IniFormat);
    sets.setValue("camera_height",QString::number(configData.camera_height, 'f', 2));
    sets.setValue("camera_angle",QString::number(configData.camera_angle, 'f', 2));
    sets.setValue("camera_abs_x",QString::number(configData.camera_abs_x, 'f', 2));
    sets.setValue("camera_abs_y",QString::number(configData.camera_abs_y, 'f', 2));
    sets.setValue("point1_x",configData.point1_x);
    sets.setValue("point1_y",configData.point1_y);
    sets.setValue("point2_x",configData.point2_x);
    sets.setValue("point2_y",configData.point2_y);
    sets.setValue("monitor_delay",configData.monitor_delay);
    sets.setValue("modbus_delay",configData.modbus_delay);
    sets.setValue("motor_rho",QString::number(configData.motor_rho, 'f', 2));
    sets.setValue("x1_start",QString::number(configData.x1_start, 'f', 2));
    sets.setValue("x2_start",QString::number(configData.x2_start, 'f', 2));
    sets.setValue("x2_rho",QString::number(configData.x2_rho, 'f', 2));
    sets.setValue("y1_start",QString::number(configData.y1_start, 'f', 2));
    sets.setValue("target_delta",QString::number(configData.target_delta, 'f', 2));
    sets.setValue("fetch_delta",QString::number(configData.fetch_delta, 'f', 2));
    sets.setValue("y_fetch_delta",QString::number(configData.y_fetch_delta, 'f', 2));
    sets.setValue("y_target_delta",QString::number(configData.y_target_delta, 'f', 2));
    sets.sync();
}

void MainDialog::on_saveImg_clicked()
{
    // ui->saveImg->setText("saving...");
    m_ui->saveImg->setDisabled(true);
    saveImgParam();
    m_ui->saveImg->setDisabled(false);
    // ui->saveImg->setText("save param");
}

std::vector<std::string> split(const std::string & str, char dim) {
   std::vector<std::string> tokens;
   std::stringstream ss(str);
   std::string token;
   while(std::getline(ss, token, dim)) {
       tokens.push_back(token);
   }
   return tokens;
}

void MainDialog::loadConfigFile()
{
    QSettings sets(QCoreApplication::applicationName()+".ini",QSettings::IniFormat);
    configData.netType = sets.value("netType",defaultSetting.netType).toInt();
    configData.ip=sets.value("ip",defaultSetting.ip).toString();
    configData.portAddr = sets.value("portAddr",defaultSetting.portAddr).toInt();
    configData.isReplyTimeout = sets.value("replyTimeout",defaultSetting.isReplyTimeout).toBool();

    configData.serialName = sets.value("serialName",defaultSetting.serialName).toString();
    configData.baudRate = sets.value("baudRate",defaultSetting.baudRate).toInt();
    configData.modbusName = sets.value("modbusName",defaultSetting.modbusName).toString();
    configData.modbusRate = sets.value("modbusRate",defaultSetting.modbusRate).toInt();

    configData.modbusTcpIp = sets.value("modbusTcpIp",defaultSetting.modbusTcpIp).toString();
    configData.modbusTcpPort = sets.value("modbusTcpPort",defaultSetting.modbusTcpPort).toInt();
    configData.modbus_delay = sets.value("modbus_delay",defaultSetting.modbus_delay).toInt();
    configData.monitor_delay = sets.value("monitor_delay",defaultSetting.monitor_delay).toInt();

    configData.camera_height = sets.value("camera_height",defaultSetting.camera_height).toFloat();
    configData.camera_angle = sets.value("camera_angle",defaultSetting.camera_angle).toFloat();
    configData.camera_abs_x = sets.value("camera_abs_x",defaultSetting.camera_abs_x).toFloat();
    configData.camera_abs_y = sets.value("camera_abs_y",defaultSetting.camera_abs_y).toFloat();
    configData.point1_x = sets.value("point1_x",defaultSetting.point1_x).toInt();
    configData.point1_y = sets.value("point1_y",defaultSetting.point1_y).toInt();
    configData.point2_x = sets.value("point2_x",defaultSetting.point2_x).toInt();
    configData.point2_y = sets.value("point2_y",defaultSetting.point2_y).toInt();

    configData.inv_thd = sets.value("inv_thd",defaultSetting.inv_thd).toInt();
    configData.canny_1 = sets.value("canny_1",defaultSetting.canny_1).toInt();
    configData.canny_2 = sets.value("canny_2",defaultSetting.canny_2).toInt();
    configData.canny_3 = sets.value("canny_3",defaultSetting.canny_3).toInt();
    configData.hgline_1 = sets.value("hgline_1",defaultSetting.hgline_1).toInt();
    configData.hgline_2 = sets.value("hgline_2",defaultSetting.hgline_2).toInt();
    configData.hgline_3 = sets.value("hgline_3",defaultSetting.hgline_3).toInt();
    configData.hgline_4 = sets.value("hgline_4",defaultSetting.hgline_4).toInt();
    configData.blur_kernel = sets.value("blur_kernel",defaultSetting.blur_kernel).toInt();

    configData.line1_ang = sets.value("line1_ang",defaultSetting.line1_ang).toInt();
    configData.line1_roh = sets.value("line1_roh",defaultSetting.line1_roh).toInt();
    configData.line1_sel_low = sets.value("line1_sel_low",defaultSetting.line1_sel_low).toInt();
    configData.line2_ang = sets.value("line2_ang",defaultSetting.line2_ang).toInt();
    configData.line2_roh = sets.value("line2_roh",defaultSetting.line2_roh).toInt();
    configData.line2_sel_low = sets.value("line2_sel_low",defaultSetting.line2_sel_low).toInt();
    configData.line_roh_abs = sets.value("line_roh_abs",defaultSetting.line_roh_abs).toFloat();
    configData.line_ang_abs = sets.value("line_ang_abs",defaultSetting.line_ang_abs).toFloat();
    configData.lines_num = sets.value("lines_num",defaultSetting.lines_num).toInt();

    configData.line_angs = sets.value("line_angs",defaultSetting.line_angs).toString();
    configData.line_rhos = sets.value("line_rhos",defaultSetting.line_rhos).toString();
    configData.a = sets.value("a",defaultSetting.a).toFloat();
    configData.b = sets.value("b",defaultSetting.b).toFloat();
    configData.c = sets.value("c",defaultSetting.c).toFloat();
    configData.d = sets.value("d",defaultSetting.d).toFloat();

    configData.seprate_rho = sets.value("seprate_rho",defaultSetting.seprate_rho).toFloat();
    configData.seprate_theta = sets.value("seprate_theta",defaultSetting.seprate_theta).toFloat();
    auto tt = HoughToPoints(configData.seprate_rho, configData.seprate_theta);
    configData.seprate_p1x = tt.first.x;
    configData.seprate_p1y = tt.first.y;
    configData.seprate_p2x = tt.second.x;
    configData.seprate_p2y = tt.second.y;
    spdlog::info("config seprate points ({},{}), ({},{}), rho {}, theta {}", configData.seprate_p1x, configData.seprate_p1y,
        configData.seprate_p2x, configData.seprate_p2y, configData.seprate_rho, configData.seprate_theta);

    configData.monitor_delay = sets.value("monitor_delay",defaultSetting.monitor_delay).toInt();
    configData.modbus_delay = sets.value("modbus_delay",defaultSetting.modbus_delay).toInt();
    spdlog::info("modbus monitor delay {}ms, modbus write delay {}ms",configData.monitor_delay, configData.modbus_delay);
    configData.x2_rho = sets.value("x2_rho",defaultSetting.x2_rho).toFloat();
    configData.motor_rho = sets.value("motor_rho",defaultSetting.motor_rho).toFloat();
    spdlog::info("config x2_rho {}, motor_rho {}", configData.x2_rho, configData.motor_rho);
    configData.x1_start = sets.value("x1_start",defaultSetting.x1_start).toFloat();
    configData.x2_start = sets.value("x2_start",defaultSetting.x2_start).toFloat();
    configData.y1_start = sets.value("y1_start",defaultSetting.x2_start).toFloat();
    configData.target_delta = sets.value("target_delta",defaultSetting.target_delta).toFloat();
    configData.fetch_delta = sets.value("fetch_delta",defaultSetting.fetch_delta).toFloat();
    configData.y_fetch_delta = sets.value("y_fetch_delta",defaultSetting.y_fetch_delta).toFloat();
    configData.y_target_delta = sets.value("y_target_delta",defaultSetting.y_target_delta).toFloat();
    spdlog::info("config x1_start {}, x2_start {}, y1_start {}",
        configData.x1_start, configData.x2_start, configData.y1_start);
    spdlog::info("config target_delta {}, fetch_delta {}, y fetch delta {}, y target delta {}",
        configData.target_delta, configData.fetch_delta, configData.y_fetch_delta, configData.y_target_delta);

    configData.hsv_low1 = sets.value("hsv_low1",defaultSetting.hsv_low1).toInt();
    configData.hsv_low2 = sets.value("hsv_low2",defaultSetting.hsv_low2).toInt();
    configData.hsv_low3 = sets.value("hsv_low3",defaultSetting.hsv_low3).toInt();
    configData.hsv_high1 = sets.value("hsv_high1",defaultSetting.hsv_high1).toInt();
    configData.hsv_high2 = sets.value("hsv_high2",defaultSetting.hsv_high2).toInt();
    configData.hsv_high3 = sets.value("hsv_high3",defaultSetting.hsv_high3).toInt();

    std::vector<std::string> angs = split(configData.line_angs.toStdString(),',');
    std::vector<std::string> rhos = split(configData.line_rhos.toStdString(),',');
    // g_lines.clear();
    for (int pos =0; pos < angs.size(); pos++) {
        Lines l;
        l.angle = std::atof(angs[pos].c_str());
        l.rho = std::atof(rhos[pos].c_str());
        // g_lines.push_back(l);
        g_lines[pos] = l;
        spdlog::info("config ang {} rho {}", l.angle, l.rho);
    }

    configData.roi = sets.value("roi",defaultSetting.roi).toString();
    std::vector<std::string> rois = split(configData.roi.toStdString(),';');
    for (int pos =0; pos < rois.size(); pos++) {
        std::vector<std::string> point = split(rois[pos],',');
        int x = std::atoi(point[0].c_str());
        int y = std::atof(point[1].c_str());
        roi_points.push_back(cv::Point(x,y));
        spdlog::info("config roi {},{} ", x, y);
    }

    X2_MACH.first = configData.x2_rho;
    X2_MACH.second = 0;
    X1_MACH.first = 1020.0 + configData.x2_rho;
    X1_MACH.second = 0;

    m_ui->txbRecv->append("config file loadded!");
}

void MainDialog::fetchNewConfig(ConfigData &configData)
{
    configData.serialName = m_ui->serialName->text();
    configData.baudRate = m_ui->baudRate->text().toUInt();
}

void MainDialog::showUIConfigData(const ConfigData& configData)
{
    m_ui->baudRate->setText(QString::number(configData.baudRate));
    m_ui->serialName->setText(configData.serialName);

    m_ui->modbus->setText(configData.modbusTcpIp+":"+QString::number(configData.modbusTcpPort));
    m_ui->monitor_delay->setText(QString::number(configData.monitor_delay));
    m_ui->modbus_delay->setText(QString::number(configData.modbus_delay));

    m_ui->x1_start->setText(QString::number(configData.x1_start));
    m_ui->x2_start->setText(QString::number(configData.x2_start));
    m_ui->x2_rho->setText(QString::number(configData.x2_rho));
    m_ui->motor_rho->setText(QString::number(configData.motor_rho));
    m_ui->y1_start->setText(QString::number(configData.y1_start));
    m_ui->target_delta->setText(QString::number(configData.target_delta));
    m_ui->fetch_delta->setText(QString::number(configData.fetch_delta));
    m_ui->y_fetch_delta->setText(QString::number(configData.y_fetch_delta));
    m_ui->y_target_delta->setText(QString::number(configData.y_target_delta));
}

void MainDialog::main_img_refresh(cv::Mat img) {
    std::vector<cv::Point> points;
    std::vector<std::string> rois = split(configData.roi.toStdString(),';');
    for (int pos =0; pos < rois.size(); pos++) {
        std::vector<std::string> point = split(rois[pos],',');
        int x = std::atoi(point[0].c_str());
        int y = std::atof(point[1].c_str());
        points.push_back(cv::Point(x,y));
    }

    cv::polylines(img, points, true, cv::Scalar(0, 255, 0), 4);

    QImage qimg = QImage((const unsigned char*)(img.data),
        img.cols,
        img.rows,
        img.step,
        // QImage::Format_Grayscale8).copy();
        QImage::Format_RGB888).copy();

    QPixmap piximg = QPixmap::fromImage(qimg);


    if (!piximg.isNull()) {
        m_ui->painter->setPixmap(piximg.scaled(m_ui->painter->size(), Qt::KeepAspectRatio));
        m_ui->painter_2->setPixmap(piximg.scaled(m_ui->painter_2->size(), Qt::KeepAspectRatio));
    }

    // m_ui->delta->setText(QString("%1").arg(m_delta));
    // m_ui->delta_mm->setText(QString("%1").arg(m_delta * 0.1));
    // m_ui->delta_c->setText(QString("%1").arg(m_delta_ang * 180 / CV_PI));
}

extern DataPkt data_pkt;
extern volatile bool run_sync;
void MainDialog::calibration_refresh_delta() {
    m_ui->y1_fetch->setText(QString::number(data_pkt.y1_fetch));
    m_ui->y1_target->setText(QString::number(data_pkt.y1_target));
    m_ui->x1_fetch->setText(QString::number(data_pkt.x1_fetch));
    m_ui->x2_fetch->setText(QString::number(data_pkt.x2_fetch));
    m_ui->x1_target->setText(QString::number(data_pkt.x1_target));
    m_ui->x2_target->setText(QString::number(data_pkt.x2_target));
    m_ui->frames->setText(QString::number(data_pkt.frames));

    if (trigger_process) {
        trigger_process = false;
        m_port->writeModbusData(500, 2, data_pkt.x1_fetch);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(504, 2, data_pkt.x2_fetch);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(508, 2, data_pkt.y1_fetch);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(520, 2, data_pkt.x1_target);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(524, 2, data_pkt.x2_target);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(528, 2, data_pkt.y1_target);
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(700, 2, 0.0f);
        m_port->thd_msleep(configData.modbus_delay);
//          m_port->writeModbusData(600, 2, 0.0f);
// //         m_port->thd_msleep(configData.modbus_delay);
//         // spdlog::info("trigger test d600 wto 0");
        m_ui->trigger->setDisabled(false);
    } else if (m_ui->auto_run->checkState() == Qt::Checked) {
        if (run_sync) {
            m_port->writeModbusData(500, 2, data_pkt.x1_fetch);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(504, 2, data_pkt.x2_fetch);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(508, 2, data_pkt.y1_fetch);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(520, 2, data_pkt.x1_target);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(524, 2, data_pkt.x2_target);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(528, 2, data_pkt.y1_target);
            m_port->thd_msleep(configData.modbus_delay);
            m_port->writeModbusData(700, 2, 0.0f);
            m_port->thd_msleep(configData.modbus_delay);
            run_sync = false;
        }
        // m_port->writeModbusData(600, 2, 0.0f);
//        m_port->thd_msleep(configData.modbus_delay);
        // spdlog::info("checkbox test d600 wto 0");
    }
}
void MainDialog::on_SerialOpen_clicked()
{
    if (m_port->isOpened == false) {
        m_port->startPort(configData);
        m_port->isOpened = true;
        m_ui->SerialOpen->setText("关闭串口");
    } else {
        m_port->closePort();
        m_port->isOpened = false;
        m_ui->SerialOpen->setText("打开串口");
    }
}

void MainDialog::on_auto_run_stateChanged(int arg1) {
    emit signal_auto_run(m_ui->auto_run->checkState());
}

QString MainDialog::toString(unsigned char*buf, unsigned int bufLen)
{
    QString str("");
    str += QString("Len %1,").arg(bufLen);
    unsigned int i = 0;
    for ( ; i < bufLen; ++i)
        str += QString(" %1").arg((unsigned char)buf[i], 2, 16, QLatin1Char('0'));
    return str;
}

void MainDialog::CameraInit() {
    // pretend to enum cam device
    m_ui->ComboDevices->clear();
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("GBK"));
    m_ui->ComboDevices->setStyle(QStyleFactory::create("Windows"));
    EnumCamDevice();
    for (auto & v : m_cameras) {
        m_ui->ComboDevices->addItem(QString::fromStdString(v.name));
        qDebug() << QString::fromStdString(v.name);
        v.handler = new (std::nothrow) CMvCamera();
        v.timer = new QTimer(this);
        v.timer->stop();
        v.timer->setInterval(configData.monitor_delay);
        v.timer->setTimerType(Qt::PreciseTimer);
        connect(v.timer, &QTimer::timeout, this, &MainDialog::monitor_modbus_hdl);
    }
    m_ui->ComboDevices->setCurrentIndex(0);
    // m_cam_idx = m_ui->ComboDevices->currentIndex();

    for (int idx=0; idx < m_cameras.size(); ++idx) {
        int nRet = m_cameras[idx].handler->Open(m_stDevList.pDeviceInfo[idx]);
        if (MV_OK != nRet) {
            spdlog::error("Open camera {} Fail {}", idx, nRet);
            return;
        }
        nRet = m_cameras[idx].handler->SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        if (nRet) {
            spdlog::error("set AcquisitionMode mode failed!");
        }
        nRet = m_cameras[idx].handler->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        if (nRet) {
            spdlog::error("set TriggerMode mode failed!");
        }

        // nRet = m_cameras[idx].handler->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        // if (nRet) {
        //     qDebug("set trigger mode failed!");
        //     return false;
        // }
    }
}

void MainDialog::on_bnOpen_clicked()
{
    #if TEST_CAMERA
    for (auto & v : m_cameras) {
        if (v.is_opened) {
            v.is_opened = false;
            v.timer->stop();
            m_ui->bnOpen->setText("打开");
            m_ui->tbExposure->setEnabled(false);
            m_ui->tbGain->setEnabled(false);
            m_ui->tbFrameRate->setEnabled(false);
            m_imgproc->camera_enable = false;
        } else {
            v.is_opened = true;
            m_ui->bnOpen->setText("关闭");
            m_ui->tbExposure->setEnabled(true);
            m_ui->tbGain->setEnabled(true);
            m_ui->tbFrameRate->setEnabled(true);
            m_imgproc->camera_enable = true;
            if (m_ui->auto_run->checkState()) {
                v.timer->start();
            } else {
                v.timer->stop();
            }
            emit cameraStart(v.handler, m_port);
        }
    }
    #else
        if (m_imgproc->camera_enable) {
            m_ui->bnOpen->setText("打开");
            m_ui->tbExposure->setEnabled(false);
            m_ui->tbGain->setEnabled(false);
            m_ui->tbFrameRate->setEnabled(false);
            m_imgproc->camera_enable = false;
            emit cameraStart(nullptr, m_port);
        } else {
            m_ui->bnOpen->setText("关闭");
            m_ui->tbExposure->setEnabled(true);
            m_ui->tbGain->setEnabled(true);
            m_ui->tbFrameRate->setEnabled(true);
            m_imgproc->camera_enable = true;
            emit cameraStart(nullptr, m_port);
        }
    #endif
}

void MainDialog::monitor_modbus_hdl() {
    float val;
    m_port->readModbusData(700, 2, val);
}

extern std::vector<std::pair<double, double>> g_roi;
void MainDialog::on_Calibration_clicked()
{
    static bool click_stat = false;
    m_ui->Calibration->setEnabled(true);
    #if TEST_CAMERA
        for (auto & v : m_cameras) {
            if (v.is_opened) {
                v.is_opened = false;
                m_ui->Calibration->setText("校准");
                m_imgproc->camera_enable = false;
                emit cameraCalStart(v.handler);
                imgw->hide();
                qDebug() << "g_lines size " << g_lines.size();
                std::stringstream ss_rho, ss_ang;
                for (const auto & v: g_lines) {
                    ss_rho << fmt::format("{:.2f}", v.rho);
                    ss_rho << fmt::format(",");

                    ss_ang << fmt::format("{:.2f}", v.angle);
                    ss_ang << fmt::format(",");
                }
                std::string line_angs = ss_ang.str();
                std::string line_rhos = ss_rho.str();
                if (!line_angs.empty()) {
                    line_angs.pop_back();
                }
                if (!line_rhos.empty()) {
                    line_rhos.pop_back();
                }
                configData.line_rhos = QString(line_rhos.c_str());
                configData.line_angs = QString(line_angs.c_str());
                spdlog::info("line angs {}, rhos {}", line_angs, line_rhos);
            } else {
                v.is_opened = true;
                m_ui->Calibration->setText("校准中...");
                imgw->total_lines = 6;
                imgw->show();
                m_imgproc->camera_enable = true;
                emit cameraCalStart(v.handler);
            }
        }
    #else
        if (click_stat) {
            click_stat = false;
            m_imgproc->camera_enable = false;
            m_ui->Calibration->setText("校准");
            emit cameraCalStart(nullptr);
            imgw->hide();
            qDebug() << "g_lines size " << g_lines.size();
            std::stringstream ss_rho, ss_ang;
            for (const auto & v: g_lines) {
                ss_rho << fmt::format("{:.2f}", v.rho);
                ss_rho << fmt::format(",");

                ss_ang << fmt::format("{:.2f}", v.angle);
                ss_ang << fmt::format(",");
            }
            std::string line_angs = ss_ang.str();
            std::string line_rhos = ss_rho.str();
            if (!line_angs.empty()) {
                line_angs.pop_back();
            }
            if (!line_rhos.empty()) {
                line_rhos.pop_back();
            }
            configData.line_rhos = QString(line_rhos.c_str());
            configData.line_angs = QString(line_angs.c_str());
            spdlog::info("line angs {}, rhos {}", line_angs, line_rhos);

            std::stringstream roi;
            for (const auto & v: g_roi) {
                roi << fmt::format("{}", v.first);
                roi << fmt::format(",");
                roi << fmt::format("{}", v.second);
                roi << fmt::format(";");
            }
            configData.roi = QString(roi.str().c_str());
        } else {
            click_stat = true;
            m_imgproc->camera_enable = true;
             m_ui->Calibration->setText("校准中...");
             imgw->show();
             emit cameraCalStart(nullptr);
         }
    #endif
}

void MainDialog::EnumCamDevice()
{
    // ch:枚举子网内所有设备 | en:Enumerate all devices within subnet
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
    if (MV_OK != nRet)
    {
        return;
    }

    // ch:将值加入到信息列表框中并显示出来 | en:Add value to the information list box and display
    for (unsigned int i = 0; i < m_stDevList.nDeviceNum; i++)
    {
        CameraInfo cam;
        MV_CC_DEVICE_INFO* pDeviceInfo = m_stDevList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
        {
            continue;
        }
        char strUserName[256] = {0};
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            if (strcmp("", (char*)pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName) != 0)
            {
                snprintf(strUserName, 256, "[%d]GigE:   %s (%s) (%d.%d.%d.%d)", i, pDeviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName,
                         pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber, nIp1, nIp2, nIp3, nIp4);
            }
            else
            {
                snprintf(strUserName, 256, "[%d]GigE:   %s (%s) (%d.%d.%d.%d)", i, pDeviceInfo->SpecialInfo.stGigEInfo.chModelName,
                         pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber, nIp1, nIp2, nIp3, nIp4);
            }
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
        {
            if (strcmp("", (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName) != 0)
            {
                snprintf(strUserName, 256, "[%d]UsbV3:  %s (%s)", i, pDeviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName,
                         pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            }
            else
            {
                snprintf(strUserName, 256, "[%d]UsbV3:  %s (%s)", i, pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName,
                         pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            }
        }
        else
        {
            ShowErrorMsg("Unknown device enumerated", 0);
        }
        cam.name = std::string(strUserName);
        cam.index = i;
        cam.is_opened = false;
        m_cameras.push_back(cam);
    }

    if (0 == m_cameras.size())
    {
        ShowErrorMsg("No camera device", 0);
        return;
    }
}

// ch:显示错误信息 | en:Show error message
void MainDialog::ShowErrorMsg(QString csMessage, unsigned int nErrorNum)
{
    QString errorMsg = csMessage;
    if (nErrorNum != 0)
    {
        QString TempMsg;
        TempMsg.sprintf(": Error = %x: ", nErrorNum);
        errorMsg += TempMsg;
    }

    switch(nErrorNum)
    {
    case MV_E_HANDLE:           errorMsg += "Error or invalid handle ";                                         break;
    case MV_E_SUPPORT:          errorMsg += "Not supported function ";                                          break;
    case MV_E_BUFOVER:          errorMsg += "Cache is full ";                                                   break;
    case MV_E_CALLORDER:        errorMsg += "Function calling order error ";                                    break;
    case MV_E_PARAMETER:        errorMsg += "Incorrect parameter ";                                             break;
    case MV_E_RESOURCE:         errorMsg += "Applying resource failed ";                                        break;
    case MV_E_NODATA:           errorMsg += "No data ";                                                         break;
    case MV_E_PRECONDITION:     errorMsg += "Precondition error, or running environment changed ";              break;
    case MV_E_VERSION:          errorMsg += "Version mismatches ";                                              break;
    case MV_E_NOENOUGH_BUF:     errorMsg += "Insufficient memory ";                                             break;
    case MV_E_ABNORMAL_IMAGE:   errorMsg += "Abnormal image, maybe incomplete image because of lost packet ";   break;
    case MV_E_UNKNOW:           errorMsg += "Unknown error ";                                                   break;
    case MV_E_GC_GENERIC:       errorMsg += "General error ";                                                   break;
    case MV_E_GC_ACCESS:        errorMsg += "Node accessing condition error ";                                  break;
    case MV_E_ACCESS_DENIED:	errorMsg += "No permission ";                                                   break;
    case MV_E_BUSY:             errorMsg += "Device is busy, or network disconnected ";                         break;
    case MV_E_NETER:            errorMsg += "Network error ";                                                   break;
    }

    QMessageBox::information(NULL, "PROMPT", errorMsg);
}

void MainDialog::on_bnGetParam_clicked()
{
    for (auto v : m_cameras) {
        if (v.handler) {    
            memset(&v.stFloatValue1, 0, sizeof(MVCC_FLOATVALUE));
            int nRet = v.handler->GetFloatValue("ExposureTime", &v.stFloatValue1);
            if (MV_OK != nRet)
            {
                ShowErrorMsg("Get Exposure Time Fail", nRet);
            }
            else
            {
                m_ui->tbExposure->setText(QString("%1").arg(v.stFloatValue1.fCurValue));
            }

            nRet = v.handler->GetFloatValue("Gain", &v.stFloatValue2);
            if (MV_OK != nRet)
            {
                ShowErrorMsg("Get Gain Fail", nRet);
            }
            else
            {
                m_ui->tbGain->setText(QString("%1").arg(v.stFloatValue2.fCurValue));
            }

            nRet = v.handler->GetFloatValue("ResultingFrameRate", &v.stFloatValue3);
            if (MV_OK != nRet)
            {
                ShowErrorMsg("Get Frame Rate Fail", nRet);
            }
            else
            {
                m_ui->tbFrameRate->setText(QString("%1").arg(v.stFloatValue3.fCurValue));
            }
        }
    }
}

void __stdcall MainDialog::ImageCallBack(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if(pUser)
    {
        MainDialog *pMainWindow = (MainDialog*)pUser;
        pMainWindow->ImageCallBackInner(pData, pFrameInfo);
    }
}

void MainDialog::ImageCallBackInner(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo)
{
    MV_DISPLAY_FRAME_INFO stDisplayInfo;
    memset(&stDisplayInfo, 0, sizeof(MV_DISPLAY_FRAME_INFO));

    stDisplayInfo.hWnd = m_hWnd;
    stDisplayInfo.pData = pData;
    stDisplayInfo.nDataLen = pFrameInfo->nFrameLen;
    stDisplayInfo.nWidth = pFrameInfo->nWidth;
    stDisplayInfo.nHeight = pFrameInfo->nHeight;
    stDisplayInfo.enPixelType = pFrameInfo->enPixelType;
    // m_pcMyCamera->DisplayOneFrame(&stDisplayInfo);
}

void MainDialog::on_cal_editingFinished()
{
    configData.x1_start = m_ui->x1_start->text().toFloat();
    configData.x2_start = m_ui->x2_start->text().toFloat();
    configData.x2_rho = m_ui->x2_rho->text().toFloat();
    configData.motor_rho = m_ui->motor_rho->text().toFloat();
    configData.y1_start = m_ui->y1_start->text().toFloat();
    configData.target_delta = m_ui->target_delta->text().toFloat();
    configData.fetch_delta = m_ui->fetch_delta->text().toFloat();
    configData.y_fetch_delta = m_ui->y_fetch_delta->text().toFloat();
    configData.y_target_delta = m_ui->y_target_delta->text().toFloat();
    configData.monitor_delay = m_ui->monitor_delay->text().toInt();
    configData.modbus_delay = m_ui->modbus_delay->text().toInt();
}

void MainDialog::on_modbusSend_clicked()
{
    if (!m_port) {
        qDebug("port is not opened!");
    } else {
        if (m_ui->optype->text().toInt() == 0) {
            float val = 0.0f;
            m_port->readModbusData(m_ui->addr->text().toInt(), m_ui->length->text().toInt(), val);
        } if (m_ui->optype->text().toInt() == 1) {
            m_port->writeModbusData(m_ui->addr->text().toInt(), m_ui->length->text().toInt(), m_ui->val->text().toFloat());
        } else {

        }

        float w_val = 1.0;
        float r_val;
        m_port->writeModbusData(500, 2 , w_val);
        m_port->writeModbusData(502, 2 , w_val + 1.1);
        m_port->writeModbusData(504, 2 , w_val + 1.1);
        m_port->writeModbusData(506, 2 , w_val + 1.1);
        m_port->readModbusData(500, 2 , r_val);
        qDebug("read %d, %f", 500, r_val);
        m_port->readModbusData(502, 2 , r_val);
         QThread::sleep(1);
        qDebug("read %d, %f", 502, r_val);
        m_port->readModbusData(504, 2 , r_val);
         QThread::sleep(1);
        qDebug("read %d, %f", 504, r_val);
        m_port->readModbusData(506, 2 , r_val);
         QThread::sleep(1);
        qDebug("read %d, %f", 506, r_val);
    }
}

void MainDialog::on_modbusSend_2_clicked()
{
    if (!m_port) {
        qDebug("port is not opened!");
    } else {
        qDebug("get D700 %f", m_port->rdy_data);
        m_port->writeModbusData(500, 2, m_ui->d500->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(504, 2, m_ui->d504->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(508, 2, m_ui->d508->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(520, 2, m_ui->d520->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(524, 2, m_ui->d524->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
        m_port->writeModbusData(528, 2, m_ui->d528->text().toFloat());
        m_port->thd_msleep(configData.modbus_delay);
//        m_port->writeModbusData(700, 2, 0.0f);
//        m_port->thd_msleep(configData.modbus_delay);
        // m_port->writeModbusData(700, 0.0f);
        // m_port->thd_msleep(500);
        qDebug("send D500 %f, D504 %f, D508 %f", m_ui->d500->text().toFloat(), m_ui->d504->text().toFloat(), m_ui->d508->text().toFloat());
    }
}

void MainDialog::on_read_clicked()
{
    float val = 0.0f;
    m_ui->test_val->setText(QString("0"));
    m_port->readModbusData(m_ui->test_addr->text().toInt(), 2 , val);
    m_ui->test_val->setText(QString("%1").arg(val));
}

void MainDialog::on_write_clicked()
{
    float val = m_ui->test_val->text().toFloat();
    m_port->writeModbusData(m_ui->test_addr->text().toInt(), 2 , val);
}

void MainDialog::on_trigger_clicked()
{
    if (m_ui->trigger->isEnabled()) {
        m_ui->trigger->setEnabled(false);
        trigger_process = true;
        emit signal_trigger();
    }
}

void MainDialog::read_modbus_data(int startAdd, int numbers) {
    float data = 0.0;
    m_port->readModbusData(startAdd, numbers, data);
    m_port->thd_msleep(configData.modbus_delay);
}


