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
#include "rknn_process.h"

#include <vector>
#include <cmath>
#include <utility>

cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB);
void sigmoid(const cv::Mat& src, cv::Mat& dst);
std::pair<std::pair<int, int>, std::pair<int, int>> get_boundary_point(int y, int x, double angle, int H, int W);

std::vector<std::vector<int>> reverse_mapping(
    const std::vector<cv::Point2f>& point_list,
    int numAngle,
    int numRho,
    std::pair<int, int> size = {32, 32}
);

struct RegionProps {
    int area;
    cv::Point2f centroid;
    cv::Rect boundingBox;
};
std::vector<RegionProps> regionprops(const cv::Mat& binaryImage);

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
    bool CameraCalTT(QLabel * pt, QLabel * pt3, QLineEdit * x, QLineEdit * y, CMvCamera* p_cam);
    static bool Process(cv::Mat &img, cv::Mat &edge_img, cv::Mat &contours_img);
    static bool FilterLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas,
        std::vector<cv::Point2i> & points);
    bool RknnFilterLines(cv::Mat &img,
                         cv::Mat &o_img,
        std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas,
        std::vector<cv::Point2i> & points);
    static bool AdaptLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
        std::vector<float> & rhos,
        std::vector<float> & thetas);

    int max_hash_win_x = 0;
    int max_hash_win_y = 0;

private:
    QString m_model_name;
    RknnProcess *  m_rknn = nullptr;
};

#endif // IMGPROCESS_H
