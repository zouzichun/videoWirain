#ifndef IMGPROCESS_H
#define IMGPROCESS_H

#include <QString>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QImage>
#include <iostream>
#include <functional>
#include <thread>
#include <opencv2/opencv.hpp>
#include "MvCamera.h"
#include "comdata.h"
#include "port/port.h"

extern std::vector<Lines> g_lines;
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
std::pair<cv::Point2f, cv::Point2f> HoughToPoints(double rho, double theta);
std::pair<cv::Point2f, cv::Point2f> HoughToPointsInImg(double rho, double theta, int width, int height);
std::pair<double, double> PointsToHoughParams(cv::Point2f p1, cv::Point2f p2);
std::pair<double, double> rotateHoughLine(double rho, double theta, double rotate_rad, std::pair<double, double> center);

std::pair<cv::Point2f, cv::Point2f> PointsImg2Mach(cv::Point2f p1, cv::Point2f p2);
std::pair<cv::Point2f, cv::Point2f> PointsMach2Img(cv::Point2f p1, cv::Point2f p2);
bool PointRelativeToLineUp(cv::Point pt1, cv::Point pt2, cv::Point p);
cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB);
std::pair<double, double> getCrossPoint(std::pair<double, double> line1, std::pair<double, double> line2);
bool GetCentralLines(const std::vector<std::vector<std::pair<double, double>>> &lines_filtered,
    std::vector<cv::Point2f> &line1,
    std::vector<cv::Point2f> &line2);
std::pair<float, float> calcNormalLineParams(float rho, float theta, float x0, float y0);

enum {
    UP_LINE = 0,
    DOWN_LINE,
    FULL_IMG
};

typedef struct DataPkt_ {
    bool valid = false;
    float x1_fetch = 0.0;
    float x1_target = 0.0;
    float x2_fetch = 0.0;
    float x2_target = 0.0;
    float y1_fetch = 0.0;
    float y1_target = 0.0;
    uint64_t frames = 0;
} DataPkt;

typedef struct LineSegment_ {
    cv::Point2f start;
    cv::Point2f mid;
    cv::Point2f end;
} LineSegment;

class ImgProcess : public QObject
{
    Q_OBJECT

public:
    explicit ImgProcess(QString dev_name, int img_height, int img_width, bool color_img);
    ~ImgProcess();

    bool ImgReady();
    bool Init();
    bool Deinit();
    void imgProcTimeout();
    bool PreProcess(cv::Mat &img, cv::Mat &edge_up, cv::Mat &edge_down);
    bool Process(cv::Mat &edge_img, std::vector<cv::Vec2f> & lines_found, bool up = true);
    bool ProcessCountor(cv::Mat &edge_img, std::vector<std::vector<std::pair<double, double>>> & lines_found);
    bool FilterLines(int rows, int cols, std::vector<cv::Vec2f> &lines_found, bool up = true);
    bool AdaptLines(std::vector<cv::Vec2f> &lines_found,
                    std::vector<std::vector<std::pair<double, double>>> &lines_filtered);
    bool GetCentralLine(std::vector<std::vector<std::pair<double, double>>> &lines_filtered,
                    std::vector<cv::Point2f> &line1,
                    std::vector<cv::Point2f> &line2);
    bool GetCentralLinesCountor(const std::vector<std::vector<std::pair<double, double>>> &lines_filtered,
        std::vector<cv::Point2f> &line1,
        std::vector<cv::Point2f> &line2);

    int max_hash_win_x = 0;
    int max_hash_win_y = 0;

    volatile bool camera_enable = false;
    int IMG_HEIGHT = 2048;
    int IMG_WIDTH = 2048;
    bool color_img = false;

signals:
    void signal_refresh_img(cv::Mat img);
    void signal_refresh_delta();
    void signal_refresh_cal_img(cv::Mat img);
    void signal_change_cal_img_mode(int mode);

public slots:
    void CameraTest(CMvCamera* p_cam, Port * p_port);
    void CameraCalTest(CMvCamera* p_cam);
    void ImageTest(CMvCamera* p_cam, Port * p_port);
    void ImageCalTest(CMvCamera* p_cam);
    void AutoRunSlot(bool auto_run);

private:
    QString m_dev_name;
    int cal_img_mode = 1;
    bool auto_run_status = false;
};

#endif // IMGPROCESS_H
