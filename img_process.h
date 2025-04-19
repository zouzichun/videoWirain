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

extern std::vector<Lines> g_lines;
float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
std::pair<double, double> PointsToHoughParams(cv::Point p1, cv::Point p2);
std::pair<double, double> rotateHoughLine(double rho, double theta, double rotate_rad, std::pair<double, double> center);
cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB);
std::pair<cv::Point2f, cv::Point2f> HoughToPointsInImg(double rho, double theta, int width, int height);

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
    static bool CameraDemo(bool & enable, QImage & img, QLineEdit * x, QLineEdit * y, float * delta, float * delta_ang, CMvCamera* p_cam);
    bool CameraCal(QLabel * pt, QLabel * pt3, QLineEdit * x, QLineEdit * y, CMvCamera* p_cam);
    // bool Process(cv::Mat &img, cv::Mat &out_img, QImage & qimg);
    static bool Process(cv::Mat &img, cv::Mat &edge_img, cv::Mat &contours_img, std::vector<cv::Vec2f> & lines_found);
    static bool FilterLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas,
        std::vector<cv::Point2i> & points);
    bool FilterLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found);
    static bool AdaptLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas);

    int max_hash_win_x = 0;
    int max_hash_win_y = 0;


    volatile bool camera_enable;
    int IMG_HEIGHT = 2048;
    int IMG_WIDTH = 2048;
    bool color_img = false;

signals:
    void signal_refresh_img(cv::Mat img);
    void signal_refresh_delta(float delta_x1, float delta_x2, float delta_ang, float delta_p_x, float delta_p_y);
    void signal_refresh_cal_img(cv::Mat img);

public slots:
    void CameraTest(CMvCamera* p_cam);
    void CameraCalTest(CMvCamera* p_cam);
    void ImageTest(CMvCamera* p_cam);
    void ImageCalTest(CMvCamera* p_cam);

private:
    QString m_dev_name;
};

#endif // IMGPROCESS_H
