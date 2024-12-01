#ifndef IMGPROCESS_H
#define IMGPROCESS_H

#include <QString>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <iostream>
#include <functional>
#include <thread>
#include "MvCamera.h"
#include <opencv2/opencv.hpp>

class MainDialog;

class ImgProcess : public QObject
{
    Q_OBJECT

public:
    explicit ImgProcess(QString dev_name);
    ~ImgProcess();

    bool ImgReady();
    bool Init();
    bool Deinit();
    void imgProcTimeout();
    static bool CameraDemo(bool & enable, QImage & img, QLineEdit * x, QLineEdit * y, float * delta, float * delta_ang, CMvCamera* p_cam);
    bool CameraCal(QLabel * pt, QLabel * pt3, QLineEdit * x, QLineEdit * y, CMvCamera* p_cam);
    // bool Process(cv::Mat &img, cv::Mat &out_img, QImage & qimg);
    static bool Process(cv::Mat &img, cv::Mat &edge_img, cv::Mat &contours_img);
    static bool FilterLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas,
        std::vector<cv::Point2i> & points);
    static bool AdaptLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas);

    int max_hash_win_x = 0;
    int max_hash_win_y = 0;


private:
    QString m_dev_name;
};

#endif // IMGPROCESS_H