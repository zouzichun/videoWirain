#include "maindialog.h"
#include <ui_maindialog.h>
#include <QMessageBox>
#include <QDebug>
#include <QSettings>
#include <QTextCodec>
#include <QStyleFactory>
#include "port/serial_port.h"
#include "port/tcp_server.h"
#include "img_process.h"
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

MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::MainDialog),
    m_serial(nullptr),
    m_imgproc(nullptr),
    m_monitor_timer(nullptr)
{
    m_ui->setupUi(this);
    logOut = m_ui->txbRecv;

    Qt::WindowFlags flags=Qt::Dialog;
    flags |=Qt::WindowMinMaxButtonsHint;
    flags |=Qt::WindowCloseButtonHint;
    setWindowFlags(flags);
    m_ui->txbRecv->document()->setMaximumBlockCount(2000);

    loadConfigFile();
    showUIConfigData(configData);

    // if (configData.netType == 3) {
        m_port = new ModbusPort(parent);
    // } else if (configData.netType == 3) {
    //     m_serial = new SerialPort();
    // }

//    connect(this, SIGNAL(sendMsgWait(const QByteArray&)),
//                m_serial, SLOT(sendMsgWait(const QByteArray&)));

    m_monitor_timer =new QTimer(this);
    // connect(this->m_monitor_timer,SIGNAL(timeout()),this,SLOT(monitor_thread()));
    m_monitor_timer->stop();


    m_cam_timer = new QTimer(this);
    connect(this->m_cam_timer,SIGNAL(timeout()),this,SLOT(cam_refresh()));
    m_cam_timer->stop();

    m_imgproc = new ImgProcess("image");
    //mvs camera
    memset(&m_stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    m_pcMyCamera = new (std::nothrow) CMvCamera;
    if (NULL == m_pcMyCamera)
    {
        qDebug()<< "new CMvCamera Instance failed" << QThread::currentThreadId();
    }
    m_hWnd = (void*)m_ui->painter->winId();

    // pretend to enum cam device
    EnumCamDevice();

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
    connect(m_ui->line2_ang,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));
    connect(m_ui->line_abs,SIGNAL(editingFinished()),this,SLOT(on_cal_editingFinished()));

    qDebug()<< "MainDialog thd id: " << QThread::currentThreadId();
}

MainDialog::~MainDialog()
{
    delete m_ui;
    if (m_serial)
        delete m_serial;
    if (configData.netType == 3) {
        delete m_port;
    }

    if (m_pcMyCamera)
    {
        // m_pcMyCamera->Close();
        delete m_pcMyCamera;
        m_pcMyCamera = NULL;
    }

    m_monitor_timer->stop();
    delete m_monitor_timer;

    m_cam_timer->stop();
    delete m_cam_timer;
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
    sets.setValue("line2_ang",configData.line2_ang);
    sets.setValue("line2_roh",configData.line2_roh);
    sets.setValue("line_abs",QString::number(configData.line_abs, 'f', 2));
    sets.sync();
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

    configData.camera_height = sets.value("camera_height",defaultSetting.camera_height).toFloat();
    configData.camera_angle = sets.value("camera_angle",defaultSetting.camera_angle).toFloat();
    configData.camera_abs_x = sets.value("camera_abs_x",defaultSetting.camera_abs_x).toFloat();
    configData.camera_abs_y = sets.value("camera_abs_y",defaultSetting.camera_abs_y).toFloat();
    configData.point1_x = sets.value("point1_x",defaultSetting.point1_x).toInt();
    configData.point1_y = sets.value("point1_y",defaultSetting.point1_y).toInt();
    configData.point2_x = sets.value("point2_x",defaultSetting.point2_x).toInt();
    configData.point2_y = sets.value("point2_y",defaultSetting.point2_y).toInt();

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
    configData.line_abs = sets.value("line_abs",defaultSetting.line_abs).toFloat();
    configData.lines_num = sets.value("lines_num",defaultSetting.lines_num).toInt();
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
    m_ui->line_abs->setText(QString::number(configData.line_abs));
}


std::mutex disp_lock;

void MainDialog::cam_refresh() {
     m_cam_timer->stop();
    // qDebug("cam refreshed..");
    QImage   img;
    {
        std::unique_lock<std::mutex> lg(disp_lock);
        img = m_img;
        QPixmap piximg = QPixmap::fromImage(img);
        if (!piximg.isNull())
            m_ui->painter->setPixmap(piximg.scaled(m_ui->painter->size(), Qt::KeepAspectRatio));
    }
    m_ui->delta->setText(QString("%1").arg(m_delta));
    m_ui->delta_mm->setText(QString("%1").arg(m_delta * 0.1));
    m_ui->delta_c->setText(QString("%1").arg(m_delta_ang * 180 / CV_PI));
    m_cam_timer->start(100);
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

void MainDialog::on_bnOpen_clicked()
{
    if (!m_camera_opened) {
        m_ui->bnOpen->setText("关闭");
        m_ui->tbExposure->setEnabled(true);
        m_ui->tbGain->setEnabled(true);
        m_ui->tbFrameRate->setEnabled(true);
        m_camera_opened = true;

        int nRet = m_pcMyCamera->Open(m_stDevList.pDeviceInfo[m_cam_idx]);
        if (MV_OK != nRet) {
            spdlog::error("Open Fail {}", nRet);
            return;
        }
        on_bnGetParam_clicked(); // ch:获取参数 | en:Get Parameter
        nRet = m_pcMyCamera->SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        if (nRet) {
            spdlog::error("set AcquisitionMode mode failed!");
        }
        nRet = m_pcMyCamera->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        if (nRet) {
            spdlog::error("set TriggerMode mode failed!");
        }

        m_imgproc->Init();
        m_cam_timer->start(100);
        m_video_thd = std::thread(&ImgProcess::CameraDemo, std::ref(m_camera_opened), std::ref(m_img), m_ui->X, m_ui->Y, &m_delta, &m_delta_ang, m_pcMyCamera);
    } else {
        m_camera_opened = false;
        m_cam_timer->stop();
        m_video_thd.join();
        
        if (m_pcMyCamera)
        {
            m_pcMyCamera->Close();
        }
        m_ui->bnOpen->setText("打开");

        m_ui->tbExposure->setEnabled(false);
        m_ui->tbGain->setEnabled(false);
        m_ui->tbFrameRate->setEnabled(false);
    }
}

void MainDialog::on_Calibration_clicked()
{
    if (!m_camera_opened) {
        m_ui->Calibration->setEnabled(false);
        m_ui->Calibration->setText("校准中...");
        int nRet = m_pcMyCamera->Open(m_stDevList.pDeviceInfo[m_cam_idx]);
        if (MV_OK != nRet) {
            spdlog::error("Open Fail {}", nRet);
            return;
        }
        nRet = m_pcMyCamera->SetEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        if (nRet) {
            spdlog::error("set AcquisitionMode mode failed!");
        }
        nRet = m_pcMyCamera->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        if (nRet) {
            spdlog::error("set TriggerMode mode failed!");
        }

        imgWindow * imgw = new imgWindow();
        imgw->show();
        m_camera_opened = true;
        m_imgproc->CameraCal(m_ui->painter,imgw->getImgPic(), m_ui->X, m_ui->Y, m_pcMyCamera);
        if (m_pcMyCamera)
        {
            m_pcMyCamera->Close();
        }
        m_camera_opened = false;
        delete imgw;
        m_ui->Calibration->setEnabled(true);
        m_ui->Calibration->setText("校准");
        saveImgParam();
    } else {
        spdlog::error("camera not enabled!");
    }
}

void MainDialog::EnumCamDevice()
{
    m_ui->ComboDevices->clear();
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("GBK"));
    m_ui->ComboDevices->setStyle(QStyleFactory::create("Windows"));
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
        m_ui->ComboDevices->addItem(QString::fromLocal8Bit(strUserName));
    }

    if (0 == m_stDevList.nDeviceNum)
    {
        ShowErrorMsg("No device", 0);
        return;
    }
    m_ui->ComboDevices->setCurrentIndex(0);
    m_cam_idx = m_ui->ComboDevices->currentIndex();
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
    MVCC_FLOATVALUE stFloatValue;
    memset(&stFloatValue, 0, sizeof(MVCC_FLOATVALUE));

    int nRet = m_pcMyCamera->GetFloatValue("ExposureTime", &stFloatValue);
    if (MV_OK != nRet)
    {
        ShowErrorMsg("Get Exposure Time Fail", nRet);
    }
    else
    {
        m_ui->tbExposure->setText(QString("%1").arg(stFloatValue.fCurValue));
    }

    nRet = m_pcMyCamera->GetFloatValue("Gain", &stFloatValue);
    if (MV_OK != nRet)
    {
        ShowErrorMsg("Get Gain Fail", nRet);
    }
    else
    {
        m_ui->tbGain->setText(QString("%1").arg(stFloatValue.fCurValue));
    }

    nRet = m_pcMyCamera->GetFloatValue("ResultingFrameRate", &stFloatValue);
    if (MV_OK != nRet)
    {
        ShowErrorMsg("Get Frame Rate Fail", nRet);
    }
    else
    {
        m_ui->tbFrameRate->setText(QString("%1").arg(stFloatValue.fCurValue));
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
    m_pcMyCamera->DisplayOneFrame(&stDisplayInfo);
}

void MainDialog::on_saveImg_clicked()
{
    m_ui->saveImg->setText("saving...");
    saveImgParam();
    m_ui->saveImg->setText("save param");
}


void MainDialog::on_cal_editingFinished()
{
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
    configData.line_abs = m_ui->line_abs->text().toFloat();
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
            m_port->readModbusData(m_ui->ceil->text().toInt(), m_ui->addr->text().toInt(), 2);
        } else {

        }
    }
}


