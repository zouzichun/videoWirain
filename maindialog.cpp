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

/**
 * @brief Constructor for MainDialog class
 * @param parent Parent widget pointer
 * 
 * Initializes the main dialog window with UI components, camera settings,
 * and communication interfaces. Sets up:
 * - Window flags and UI configuration
 * - Modbus/Serial port communication
 * - Image processing components
 * - Camera initialization
 * - Timer for monitoring
 * - Signal/slot connections for UI controls
 * - Configuration data loading
 * 
 * The dialog handles camera image display, parameter adjustments for image 
 * processing, and communication with external devices.
 */
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
    // } else if (configData.netType == 3) {
    //     m_serial = new SerialPort();
    // }

//    connect(this, SIGNAL(sendMsgWait(const QByteArray&)),
//                m_serial, SLOT(sendMsgWait(const QByteArray&)));

    m_monitor_timer =new QTimer(this);
    // connect(this->m_monitor_timer,SIGNAL(timeout()),this,SLOT(monitor_thread()));
    m_monitor_timer->stop();


    m_imgproc = new ImgProcess("image");
    m_imgproc->moveToThread(&mWorkerThread); //把数据处理类移到线程
    connect(&mWorkerThread, &QThread::finished, m_imgproc, &QObject::deleteLater);

    connect(m_imgproc, &ImgProcess::signal_refresh_delta,
        this, &MainDialog::camera_refresh_delta, Qt::QueuedConnection); // 或 QueuedConnection
    connect(m_imgproc, &ImgProcess::signal_refresh_img, this, &MainDialog::camera_img_refresh, Qt::QueuedConnection); // 或 QueuedConnection
    connect(this, &MainDialog::cameraStart, m_imgproc, &ImgProcess::CameraTest, Qt::QueuedConnection); // 或 QueuedConnection



    imgw = new imgWindow();
    connect(this, &MainDialog::cameraCalStart, m_imgproc, &ImgProcess::CameraCalTest, Qt::QueuedConnection);
    connect(m_imgproc, &ImgProcess::signal_refresh_cal_img, imgw, &imgWindow::camera_img_refresh, Qt::QueuedConnection); // 或 QueuedConnection

    m_hWnd = (void*)m_ui->painter->winId();
    connect(m_ui->inv_thd,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->canny1,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->canny2,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->canny3,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->hgline1,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->hgline2,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->hgline3,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->blur_kernel,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line1_ang,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line1_roh,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line2_ang,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line2_roh,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line_roh_abs,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line_ang_abs,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));

    CameraInit();

    mWorkerThread.start();

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

    // Canny and hglines
    sets.setValue("inv_thd",configData.inv_thd);
    sets.setValue("canny_1",configData.canny_1);
    sets.setValue("canny_2",configData.canny_2);
    sets.setValue("canny_3",configData.canny_3);
    sets.setValue("hgline_1",configData.hgline_1);
    sets.setValue("hgline_2",configData.hgline_2);
    sets.setValue("hgline_3",configData.hgline_3);
    sets.setValue("blur_kernel",configData.blur_kernel);

    // target lines info
    sets.setValue("line1_ang",configData.line1_ang);
    sets.setValue("line1_roh",configData.line1_roh);
    // sets.setValue("line1_ang_delta",QString::number(configData.line1_ang_delta, 'f', 2));
    sets.setValue("line2_ang",configData.line2_ang);
    sets.setValue("line2_roh",configData.line2_roh);
    // sets.setValue("line2_ang_delta",QString::number(configData.line2_ang_delta, 'f', 2));
    sets.setValue("line_roh_abs",QString::number(configData.line_roh_abs, 'f', 3));
    sets.setValue("line_ang_abs",QString::number(configData.line_ang_abs, 'f', 3));

    sets.setValue("line_angs",configData.line_angs);
    sets.setValue("line_rhos",configData.line_rhos);

    sets.sync();
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

    m_ui->inv_thd->setText(QString::number(configData.inv_thd));
    m_ui->canny1->setText(QString::number(configData.canny_1));
    m_ui->canny2->setText(QString::number(configData.canny_2));
    m_ui->canny3->setText(QString::number(configData.canny_3));
    m_ui->hgline1->setText(QString::number(configData.hgline_1));
    m_ui->hgline2->setText(QString::number(configData.hgline_2));
    m_ui->hgline3->setText(QString::number(configData.hgline_3));
    m_ui->blur_kernel->setText(QString::number(configData.blur_kernel));

    m_ui->line1_ang->setText(QString::number(configData.line1_ang));
    m_ui->line1_roh->setText(QString::number(configData.line1_roh));
    m_ui->line2_ang->setText(QString::number(configData.line2_ang));
    m_ui->line2_roh->setText(QString::number(configData.line2_roh));
    m_ui->line_roh_abs->setText(QString::number(configData.line_roh_abs));
    m_ui->line_ang_abs->setText(QString::number(configData.line_ang_abs));
}

void MainDialog::camera_img_refresh(cv::Mat img) {
//    qDebug("cam refreshed..");
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
    for (auto & v : m_cameras) {
        if (v.is_opened) {
            v.is_opened = false;

            v.timer->stop();

            m_ui->bnOpen->setText("打开");

            m_ui->tbExposure->setEnabled(false);
            m_ui->tbGain->setEnabled(false);
            m_ui->tbFrameRate->setEnabled(false);
            m_imgproc->camera_enable = false;
            emit cameraStart(v.handler);
        } else {
            v.is_opened = true;
            m_ui->bnOpen->setText("关闭");
            m_ui->tbExposure->setEnabled(true);
            m_ui->tbGain->setEnabled(true);
            m_ui->tbFrameRate->setEnabled(true);

            v.timer->stop();

            m_imgproc->camera_enable = true;
            emit cameraStart(v.handler);
        }
    }
}

void MainDialog::on_Calibration_clicked()
{
    // m_ui->Calibration->setEnabled(true);
    for (auto & v : m_cameras) {
        if (v.is_opened) {
            v.is_opened = false;
            m_ui->Calibration->setText("校准");
            m_imgproc->camera_enable = false;
            // emit cameraCalStart(v.handler);
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
            //        m_imgproc->CameraCal(m_ui->painter,imgw->getImgPic(), m_ui->X, m_ui->Y, m_pcMyCamera);
            //        if (m_pcMyCamera)
            //        {
            //            m_pcMyCamera->Close();
            //        }
            m_imgproc->camera_enable = true;
            emit cameraCalStart(v.handler);
            // saveImgParam();
        }
    }
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

void MainDialog::on_saveImg_clicked()
{
    // m_ui->saveImg->setText("saving...");
    m_ui->saveImg->setDisabled(true);
    saveImgParam();
    m_ui->saveImg->setDisabled(false);
    // m_ui->saveImg->setText("save param");
}


void MainDialog::on_cal_editingFinished()
{
    configData.inv_thd = m_ui->inv_thd->text().toInt();
    configData.canny_1 = m_ui->canny1->text().toInt();
    configData.canny_2 = m_ui->canny2->text().toInt();
    configData.canny_3 = m_ui->canny3->text().toInt();
    configData.hgline_1 = m_ui->hgline1->text().toInt();
    configData.hgline_2 = m_ui->hgline2->text().toInt();
    configData.hgline_3 = m_ui->hgline3->text().toInt();
    configData.blur_kernel = m_ui->blur_kernel->text().toInt();
    configData.line1_ang = m_ui->line1_ang->text().toInt();
    configData.line1_roh = m_ui->line1_roh->text().toInt();
    configData.line2_ang = m_ui->line2_ang->text().toInt();
    configData.line2_roh = m_ui->line2_roh->text().toInt();
    configData.line_roh_abs = m_ui->line_roh_abs->text().toFloat();
    configData.line_ang_abs = m_ui->line_ang_abs->text().toFloat();
}

void MainDialog::on_SerialSend_clicked()
{

}


void MainDialog::on_modbusSend_clicked()
{
    if (!m_port) {
        qDebug("port is not opened!");
    } else {
        if (m_ui->optype->text().toInt() == 0) {
            m_port->readModbusData(m_ui->ceil->text().toInt(), m_ui->addr->text().toInt(), m_ui->length->text().toInt());
        } if (m_ui->optype->text().toInt() == 1) {
            m_port->writeModbusData(m_ui->ceil->text().toInt(), m_ui->addr->text().toInt(), m_ui->val->text().toInt());
        } else {

        }
    }
}

void MainDialog::camera_refresh_delta(float d_x1, float d_x2, float d_ang, float p_x, float p_y) {
    m_ui->x1->setText(QString::number(d_x1));
    m_ui->x2->setText(QString::number(d_x2));
    m_ui->d_ang->setText(QString::number(d_ang));
    m_ui->p_x->setText(QString::number(p_x));
    m_ui->p_y->setText(QString::number(p_y));
}
