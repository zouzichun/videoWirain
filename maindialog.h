#ifndef MAINDIALOG_H
#define MAINDIALOG_H
#include "comdata.h"
#include <QDialog>
#include <thread>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include "imgwindow.h"
#include "port/port.h"
#include "port/serial_port.h"
#include "port/modbus.h"
#include "port/modbus_tcp.h"
#include "img_process.h"
#include "MvCamera.h"

namespace Ui {
class MainDialog;
}

const ConfigData defaultSetting {
    .netType=3,
    // net
    .ip="localhost",
    .portAddr=9000,
    .isReplyTimeout=false,
    // serial
    .serialName="/dev/ttyS1",
    .baudRate=19200,
    // modbus can
    .modbusName="/dev/ttyS7",
    .modbusRate=19200,
    .modbusTcpIp="192.168.20.32",
    .modbusTcpPort=520,
    // camera
    .camera_height=0.0,
    .camera_angle=0.0,
    .camera_abs_x=0.0,
    .camera_abs_y=0.0,
    .point1_x=20,
    .point1_y=150,
    .point2_x=300,
    .point2_y=190,
    /*CV*/
    .inv_thd=100,
    .canny_1=200,
    .canny_2=20,
    .canny_3=3,
    /*HG line*/
    .hgline_1=300,
    .hgline_2=100,
    .hgline_3=50,
    .blur_kernel=3,
    .line1_ang=35,
    .line1_roh=400,
    .line1_sel_low=0,
    .line2_ang=135,
    .line2_roh=400,
    .line2_sel_low=1,
    .line_roh_abs=0.05,
    .line_ang_abs=0.02,
    .lines_num=3,
    .a=11.0,
    .b=12.0,
    .c=13.0,
    .d=14.0,
    .seprate_rho=1234.1,
    .seprate_theta=1.5,
    .x2_rho = 330.0,
    .motor_rho = 21.6,
    .x1_start = 256.0,
    .x2_start = 256.0,
    .line_angs="0,0,0",
    .line_rhos="0,0,0",
};

typedef struct {
    std::string name = {};
    int index = 0;
    volatile bool is_opened = false;
    volatile int status = -1;
    CMvCamera *handler = nullptr;
    QTimer * timer = nullptr;
    MVCC_FLOATVALUE stFloatValue1;
    MVCC_FLOATVALUE stFloatValue2;
    MVCC_FLOATVALUE stFloatValue3;
} CameraInfo;

class MainDialog : public QDialog
{
    Q_OBJECT
public:
    explicit MainDialog(QWidget *parent = 0);
    ~MainDialog();
    void saveCommParam();
    void saveImgParam();
    void loadConfigFile();
    void fetchNewConfig(ConfigData &configData);
    void showUIConfigData(const ConfigData& configData);
    QString toString(unsigned char*buf, unsigned int bufLen);
    void clearState();
    int sendCommand(unsigned char command, unsigned int size, void * buff);

    uint32_t GetEncoder(uint32_t motor_id);
    int32_t GetInputPulse(uint32_t motor_id);
    int32_t GetPosition(uint32_t motor_id);
    void InitZero();
    void ShowErrorMsg(QString csMessage, unsigned int nErrorNum);
    void ImageCallBackInner(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo);
    void static __stdcall ImageCallBack(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

    void EnumCamDevice();

    void on_bnGetParam_clicked();

    void CameraInit();
    void CameraRelease();

signals:
    void sendMsgWait(const QByteArray &val);
    void cameraStart(CMvCamera* handler, Port * p_port);
    void cameraCalStart(CMvCamera* handler);

private slots:
    void on_SerialOpen_clicked();
    void on_SerialSend_clicked();
    void on_cal_editingFinished();

    void on_Calibration_clicked();

    void on_bnOpen_clicked();

    void on_saveImg_clicked();
    void on_modbusSend_clicked();
    void camera_img_refresh(cv::Mat img);
    void camera_refresh_delta(float d_x1, float d_x2, float d_ang, float p_x, float p_y);

    void on_modbusSend_2_clicked();

private:
    Ui::MainDialog *m_ui;
    SerialPort * m_serial = nullptr;
    ImgProcess * m_imgproc = nullptr;
    QTimer * m_monitor_timer = nullptr;
    Port *       m_port = nullptr;
    imgWindow * imgw = nullptr;

    void *m_hWnd;                          // ch:显示窗口句柄 | en:The Handle of Display Window
    MV_CC_DEVICE_INFO_LIST  m_stDevList;   // ch:设备信息链表 | en:The list of device info
    // CMvCamera*              m_pcMyCamera;  // ch:相机类设备实例 | en:The instance of CMvCamera
    std::mutex m_disp_mtx;
    QImage   m_img;
    float m_delta = 0.0;
    float m_delta_ang = 0.0;
    std::vector<CameraInfo> m_cameras;

    QThread mWorkerThread; //定义处理线程
    QThread mmodbusthd;
};

#endif // MAINDIALOG_H
