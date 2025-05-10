#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <limits>
#include "crcalgorithm.h"
#include "maindialog.h"
#include "img_process.h"

using namespace std;
using namespace cv;

extern bool debug_win_enable;

ImgProcess::ImgProcess(QString dev_name, int img_height, int img_width, bool color_img) :
IMG_HEIGHT(img_height),
IMG_WIDTH(img_width),
color_img(color_img),
m_dev_name(dev_name) {
}

ImgProcess::~ImgProcess() {
}

bool ImgProcess::Init() {
    return true;
}

bool ImgProcess::Deinit() {
    return true;
}

void ImgProcess::imgProcTimeout() {
}

float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB) {
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    float tt = sqrtf(A*A + B*B);
    if (tt == 0.0f) tt = 0.00001;
    float distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / tt;
    // float distance = ((float)abs(A*pointP.x + B*pointP.y + C)) * 10 / (A*A + B*B);
    return distance;
}

int getDist(cv::Point pointP, cv::Point pointA, cv::Point pointB) {
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    int distance = (abs(A*pointP.x + B*pointP.y + C)) / (sqrtf(A*A + B*B));
    return distance;
}

cv::Point2f PointMach2Img(float x, float y, float a, float b, float c, float d) {
    cv::Point2f result;
    float denominator = a * a + b * b;
    result.x = (a * (x - c) + b * (y - d)) / denominator;
    result.y = (-b * (x - c) + a * (y - d)) / denominator;
    return result;
}

cv::Point2f PointImg2Mach(float x, float y, float a, float b, float c, float d) {
    cv::Point2f result;
    result.x = x * a - y * b + c;
    result.y = x * b + y * a + d;
    return result;
}

struct win_info {
    win_info(int a, float b) :
    pos(a),
    h(b){}

    int pos;
    float h;
};

const int CONV_WIN_WIDTH = 21;
const int CONV_WIN_STRIDE = 3;

void ThinSubiteration1(cv::Mat & pSrc, cv::Mat & pDst) {
        int rows = pSrc.rows;
        int cols = pSrc.cols;
        pSrc.copyTo(pDst);
        for(int i = 0; i < rows; i++) {
                for(int j = 0; j < cols; j++) {
                        if(pSrc.at<float>(i, j) == 1.0f) {
                                /// get 8 neighbors
                                /// calculate C(p)
                                int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                                int neighbor1 = (int) pSrc.at<float>( i-1, j);
                                int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                                int neighbor3 = (int) pSrc.at<float>( i, j+1);
                                int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                                int neighbor5 = (int) pSrc.at<float>( i+1, j);
                                int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                                int neighbor7 = (int) pSrc.at<float>( i, j-1);
                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                        int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                        int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                        int(~neighbor7 & ( neighbor0 | neighbor1));
                                if(C == 1) {
                                        /// calculate N
                                        int N1 = int(neighbor0 | neighbor1) +
                                                         int(neighbor2 | neighbor3) +
                                                         int(neighbor4 | neighbor5) +
                                                         int(neighbor6 | neighbor7);
                                        int N2 = int(neighbor1 | neighbor2) +
                                                         int(neighbor3 | neighbor4) +
                                                         int(neighbor5 | neighbor6) +
                                                         int(neighbor7 | neighbor0);
                                        int N = std::min(N1,N2);
                                        if ((N == 2) || (N == 3)) {
                                                /// calculate criteria 3
                                                int c3 = ( neighbor1 | neighbor2 | ~neighbor4) & neighbor3;
                                                if(c3 == 0) {
                                                        pDst.at<float>( i, j) = 0.0f;
                                                }
                                        }
                                }
                        }
                }
        }
}


void ThinSubiteration2(cv::Mat & pSrc, cv::Mat & pDst) {
        int rows = pSrc.rows;
        int cols = pSrc.cols;
        pSrc.copyTo( pDst);
        for(int i = 0; i < rows; i++) {
                for(int j = 0; j < cols; j++) {
                        if (pSrc.at<float>( i, j) == 1.0f) {
                                /// get 8 neighbors
                                /// calculate C(p)
                            int neighbor0 = (int) pSrc.at<float>( i-1, j-1);
                            int neighbor1 = (int) pSrc.at<float>( i-1, j);
                            int neighbor2 = (int) pSrc.at<float>( i-1, j+1);
                            int neighbor3 = (int) pSrc.at<float>( i, j+1);
                            int neighbor4 = (int) pSrc.at<float>( i+1, j+1);
                            int neighbor5 = (int) pSrc.at<float>( i+1, j);
                            int neighbor6 = (int) pSrc.at<float>( i+1, j-1);
                            int neighbor7 = (int) pSrc.at<float>( i, j-1);
                                int C = int(~neighbor1 & ( neighbor2 | neighbor3)) +
                                        int(~neighbor3 & ( neighbor4 | neighbor5)) +
                                        int(~neighbor5 & ( neighbor6 | neighbor7)) +
                                        int(~neighbor7 & ( neighbor0 | neighbor1));
                                if(C == 1) {
                                        /// calculate N
                                        int N1 = int(neighbor0 | neighbor1) +
                                                int(neighbor2 | neighbor3) +
                                                int(neighbor4 | neighbor5) +
                                                int(neighbor6 | neighbor7);
                                        int N2 = int(neighbor1 | neighbor2) +
                                                int(neighbor3 | neighbor4) +
                                                int(neighbor5 | neighbor6) +
                                                int(neighbor7 | neighbor0);
                                        int N = min(N1,N2);
                                        if((N == 2) || (N == 3)) {
                                                int E = (neighbor5 | neighbor6 | ~neighbor0) & neighbor7;
                                                if(E == 0) {
                                                        pDst.at<float>(i, j) = 0.0f;
                                                }
                                        }
                                }
                        }
                }
        }
}

void NormalizeLine(cv::Mat & inputarray, cv::Mat & outputarray) {
        bool bDone = false;
        int rows = inputarray.rows;
        int cols = inputarray.cols;

        inputarray.convertTo(inputarray,CV_32FC1);
        inputarray.copyTo(outputarray);
        outputarray.convertTo(outputarray,CV_32FC1);

        /// pad source
        cv::Mat p_enlarged_src = cv::Mat(rows + 2, cols + 2, CV_32FC1);
        for(int i = 0; i < (rows+2); i++) {
            p_enlarged_src.at<float>(i, 0) = 0.0f;
            p_enlarged_src.at<float>( i, cols+1) = 0.0f;
        }
        for(int j = 0; j < (cols+2); j++) {
                p_enlarged_src.at<float>(0, j) = 0.0f;
                p_enlarged_src.at<float>(rows+1, j) = 0.0f;
        }
        for(int i = 0; i < rows; i++) {
                for(int j = 0; j < cols; j++) {
                        if (inputarray.at<float>(i, j) >= 210) {
                                p_enlarged_src.at<float>( i+1, j+1) = 1.0f;
                        }
                        else
                                p_enlarged_src.at<float>( i+1, j+1) = 0.0f;
                }
        }

        /// start to thin
        cv::Mat p_thinMat1 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
        cv::Mat p_thinMat2 = cv::Mat::zeros(rows + 2, cols + 2, CV_32FC1);
        cv::Mat p_cmp = cv::Mat::zeros(rows + 2, cols + 2, CV_8UC1);

        while (bDone != true) {
                /// sub-iteration 1
                ThinSubiteration1(p_enlarged_src, p_thinMat1);
                /// sub-iteration 2
                ThinSubiteration2(p_thinMat1, p_thinMat2);
                /// compare
                compare(p_enlarged_src, p_thinMat2, p_cmp, CV_CMP_EQ);
                /// check
                int num_non_zero = countNonZero(p_cmp);	//返回灰度值不为0的像素数
                if(num_non_zero == (rows + 2) * (cols + 2)) {
                        bDone = true;
                }
                /// copy
                p_thinMat2.copyTo(p_enlarged_src);
        }
        // copy result
        for(int i = 0; i < rows; i++) {
                for(int j = 0; j < cols; j++) {
                        outputarray.at<float>( i, j) = p_enlarged_src.at<float>( i+1, j+1);
                }
        }
}


void LineReflect(cv::Mat & inputarray, cv::Mat & outputarray)
{
    int rows = inputarray.rows;
    int cols = inputarray.cols;
    for(int i = 0; i < rows; i++) {
                for(int j = 0; j < cols; j++) {
                        if (inputarray.at<float>(i, j) == 1.0f) {
                                outputarray.at<float>( i, j) = 0.0f;
                        }
                }
    }
}

void FilterSmallRegions(cv::Mat & pSrc, cv::Mat & pDst) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(pSrc, contours, hierarchy,RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<vector<Point>>::iterator k;
    for (k = contours.begin(); k != contours.end();)
    {
        if (contourArea(*k, false) < 100) {
            k = contours.erase(k);
        }
        else
            ++k;
    }

    //contours[i]代表第i个轮廓，contours[i].size()代表第i个轮廓上所有的像素点
    for (uint32_t i = 0; i < contours.size(); i++) {
//        for (int j = 0; j < contours[i].size(); j++) {
//            Point P = Point(contours[i][j].x, contours[i][j].y);
//        }
        drawContours(pDst, contours,i, Scalar(255), -1, 8);
    }
}

QTime g_t;
volatile int old_time = 0;
int TimeElapsed() {
    int cur_time = g_t.msec();
    int tt_time = cur_time - old_time;
    old_time = cur_time;
    return tt_time;
}

cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB) {
    double ka, kb;
    double tt;
    if (LineA[2] - LineA[0] == 0) {
        tt = 0.00001;
    } else{
        tt = (double)(LineA[2] - LineA[0]);
    }
    ka = (double)(LineA[3] - LineA[1]) / tt; //求出LineA斜率
    if (LineB[2] - LineB[0] == 0) {
        tt = 0.000001;
    } else{
        tt = (double)(LineB[2] - LineB[0]);
    }
    kb = (double)(LineB[3] - LineB[1]) / tt; //求出LineB斜率

    cv::Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return crossPoint;
}

std::pair<double, double> getCrossPoint(std::pair<double, double> line1, std::pair<double, double> line2) {
    std::pair<cv::Point2f, cv::Point2f> pl1 = HoughToPoints(line1.first, line1.second);
    std::pair<cv::Point2f, cv::Point2f> pl2 = HoughToPoints(line2.first, line2.second);
    cv::Vec4d LineA(pl1.first.x, pl1.first.y, pl1.second.x, pl1.second.y);
    cv::Vec4d LineB(pl2.first.x, pl2.first.y, pl2.second.x, pl2.second.y);
    double ka, kb;
    double tt;
    if (LineA[2] - LineA[0] == 0) {
        tt = 0.00001;
    } else{
        tt = (double)(LineA[2] - LineA[0]);
    }
    ka = (double)(LineA[3] - LineA[1]) / tt; //求出LineA斜率
    if (LineB[2] - LineB[0] == 0) {
        tt = 0.000001;
    } else{
        tt = (double)(LineB[2] - LineB[0]);
    }
    kb = (double)(LineB[3] - LineB[1]) / tt; //求出LineB斜率

    cv::Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return std::pair<double, double>(crossPoint.x, crossPoint.y);
}

std::pair<cv::Point2f, cv::Point2f> HoughToPoints(double rho, double theta) {
    double aa = cos(theta), bb = sin(theta);
    double x00 = aa * rho, y00 = bb * rho;
    double x1 = cvRound(x00 + 10000*(-bb));
    double y1 = cvRound(y00 + 10000*(aa));
    double x2 = cvRound(x00 - 10000*(-bb));
    double y2 = cvRound(y00 - 10000*(aa));
    return std::pair<cv::Point2f, cv::Point2f>(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
}

std::pair<cv::Point2f, cv::Point2f> HoughToPointsInImg(double rho, double theta, int width, int height) {
    double aa = cos(theta), bb = sin(theta);
    double x00 = aa * rho, y00 = bb * rho;
    double x1 = cvRound(x00 + 10000*(-bb));            
    double y1 = cvRound(y00 + 10000*(aa));
    double x2 = cvRound(x00 - 10000*(-bb));
    double y2 = cvRound(y00 - 10000*(aa));
    cv::Point pt1(static_cast<int>(x1), static_cast<int>(y1));
    cv::Point pt2(static_cast<int>(x2), static_cast<int>(y2));
    if (cv::clipLine(Size(width, height), pt1, pt2)) {
        if (pt1.y > pt2.y) {
            return std::pair<cv::Point2f, cv::Point2f>(pt2, pt1);
        } else {
            return std::pair<cv::Point2f, cv::Point2f>(pt1, pt2);
        }
    } else {
        return std::pair<cv::Point2f, cv::Point2f>(cv::Point2f(x1, y1), cv::Point2f(x2, y2));
    }
}

std::pair<cv::Point2f, cv::Point2f> PointsImg2Mach(cv::Point2f p1, cv::Point2f p2) {
    float x1_img,y1_img,x2_img,y2_img;
    float x1_mach,y1_mach,x2_mach,y2_mach;
    x1_img = p1.x;
    y1_img = p1.y;
    x2_img = p2.x;
    y2_img = p2.y;
    float a = configData.a;
    float b = configData.b;
    float c = configData.c;
    float d = configData.d;
    x1_mach = x1_img * a - b * y1_img + c;
    y1_mach = x1_img * b + a * y1_img + d;
    x2_mach = x2_img * a - b * y2_img + c;
    y2_mach = x2_img * b + a * y2_img + d;
    return std::pair<cv::Point2f, cv::Point2f>(cv::Point2f(x1_mach, y1_mach), cv::Point2f(x2_mach, y2_mach));
}

std::pair<cv::Point2f, cv::Point2f> PointsMach2Img(cv::Point2f p1, cv::Point2f p2) {
    float x1_img,y1_img,x2_img,y2_img;
    float x1_mach,y1_mach,x2_mach,y2_mach;
    x1_mach = p1.x;
    y1_mach = p1.y;
    x2_mach = p2.x;
    y2_mach = p2.y;
    float a = configData.a;
    float b = configData.b;
    float c = configData.c;
    float d = configData.d;
    float denominator = a * a + b * b;
    x1_img = (a * (x1_mach - c) + b * (y1_mach - d)) / denominator;
    y1_img = (-b * (x1_mach - c) + a * (y1_mach - d)) / denominator;
    x2_img = (a * (x2_mach - c) + b * (y2_mach - d)) / denominator;
    y2_img = (-b * (x2_mach - c) + a * (y2_mach - d)) / denominator;
    return std::pair<cv::Point2f, cv::Point2f>(cv::Point2f(x1_img, y1_img), cv::Point2f(x2_img, y2_img));
}

std::pair<double, double> PointsToHoughParams(cv::Point2f p1, cv::Point2f p2) {
    double A = p2.y - p1.y;
    double B = p1.x - p2.x;
    double C = p2.x * p1.y - p1.x * p2.y;

    if (std::abs(B) < 1e-6) {
        return std::pair<double, double>(99999.9, 0.0);
    }

    double theta_rad = atan2(B, A);
    double rho = 0.0;
    if ((A * A + B * B) < 1.0e-6) {
        rho = C / 1.0e-3;
    } else {
        rho = C / sqrt(A * A + B * B);
    }
    // std::cout << "PointsToHoughParams rho "<< rho << ", theta "<< theta_rad << std::endl;
    double k = 0.0;
    if (p2.x == p1.x) {
        if (p2.y == p1.y) {
            k = 0.0;
        } else if (p2.y > p1.y) {
            k = 1.0e9;
        } else {
            k = -1.0e9;
        }
    } else {
        k = static_cast<double>(p2.y - p1.y) / (p2.x - p1.x); //求出直线的斜率// -3.1415926/2-----+3.1415926/2
    }

    double b = p1.y - k * p1.x; //求出直线的斜率
    // std::cout << "PointsToHoughParams k "<< k << ", b "<< b << std::endl;
    if (b < 0) {
        rho = -abs(rho);
    } else {
        rho = abs(rho);
    }

    theta_rad = fmod(theta_rad + 2*CV_PI, CV_PI);
    return std::pair<double, double>(rho, theta_rad);
}

std::pair<double, double> rotateHoughLine(double rho, double theta, double rotate_rad, std::pair<double, double> center) {
    const double EPS = 1e-9;
    const double t = 1000.0;
    auto [x0, y0] = center;
    // 生成原直线上两点
    // double x1 = rho * cos(theta);
    // double y1 = rho * sin(theta);
    // double x2 = x1 - t * sin(theta);
    // double y2 = y1 + t * cos(theta);
    auto out_p = HoughToPoints(rho, theta);
    double x1 = out_p.first.x;
    double y1 = out_p.first.y;
    double x2 = out_p.second.x;
    double y2 = out_p.second.y;
    // 旋转函数
    auto rotate = [&](double x0, double y0) -> std::pair<double, double> {
    double x_ = x1 - x0;
    double y_ = y1 - y0;
            return std::pair<double, double>(
            x_ * cos(rotate_rad) - y_ * sin(rotate_rad) + x0,
            x_ * sin(rotate_rad) + y_ * cos(rotate_rad) + y0);
    };
    // 旋转点并计算新直线
    auto [rx1, ry1] = rotate(x1, y1);
    auto [rx2, ry2] = rotate(x2, y2);
    auto new_line = PointsToHoughParams(cv::Point2f(rx1, ry1), cv::Point2f(rx2, ry2));
    return new_line;
}

bool PointRelativeToLineUp(cv::Point pt1, cv::Point pt2, cv::Point p) {
    // 计算向量叉积 (x2 - x1)(yp - y1) - (y2 - y1)(xp - x1)
    int cross = (pt2.x - pt1.x) * (p.y - pt1.y)
              - (pt2.y - pt1.y) * (p.x - pt1.x);

//    // 根据OpenCV坐标系判断方向
//    if (cross > 0) return "上方";  // 向量左侧
//    if (cross < 0) return "下方";  // 向量右侧
//    return "在直线上";
    return (cross > 0) ? true : false;
}

extern std::vector<std::string> split(const std::string & str, char dim);
extern std::vector<cv::Point> roi_points;

bool ImgProcess::PreProcess(cv::Mat &img, cv::Mat &edge_up, cv::Mat &edge_down) {
    cv::Mat hsv;
    cv::Mat processed;
    cv::Mat edge_img;
    cv::Mat grayimg;
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC3);

    if (roi_points.size() < 3) {
     cv::cvtColor(img, hsv, COLOR_RGB2HSV);
    } else {
     cv::fillPoly(mask, {roi_points}, cv::Scalar(255,255,255));
     cv::Mat roi;
     img.copyTo(roi, mask);
     cv::cvtColor(roi, hsv, COLOR_RGB2HSV);
    }

    if (debug_win_enable) {
        cv::Mat ott;
        hsv.copyTo(ott);
        cv::resize(ott, ott, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%
        imshow("hsv_img", ott);
    }

    // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
    // cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);
    // qDebug("in img depth %d, type %d, ", img.depth(), img.type());
    // cv::cvtColor(roi, hsv, COLOR_RGB2HSV);
    // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
    // cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);
    
   // 白色阈值范围
    cv::inRange(hsv, Scalar(configData.hsv_low1, configData.hsv_low2, configData.hsv_low3),
                Scalar(configData.hsv_high1, configData.hsv_high2, configData.hsv_high3), hsv);

    // if (debug_win_enable) {
    //     cv::Mat ott2;
    //     hsv.copyTo(ott2);
    //     cv::resize(ott2, ott2, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%
    //     imshow("gray_img", ott2);
    // }
    
    // 建议2：添加kernel验证
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, Size(configData.blur_kernel,configData.blur_kernel));
    cv::morphologyEx(hsv, processed, MORPH_CLOSE, kernel, Point(-1,-1), 1);
    cv::morphologyEx(processed, grayimg, MORPH_OPEN, kernel, Point(-1,-1), 2);

    if (debug_win_enable) {
        cv::Mat ott3;
        grayimg.copyTo(ott3);
        cv::resize(ott3, ott3, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%
        imshow("gray_img", ott3);
    }


//    cv::Mat img_tt;
//    cv::cvtColor(img, img_tt, COLOR_RGB2GRAY);

    // cv::bilateralFilter(img, img_tt, 0, 200, 10);
    // cv::GaussianBlur(img, img, Size(configData.blur_kernel,configData.blur_kernel), 0);
//    cv::threshold(img_tt, img_tt, configData.inv_thd, 255, THRESH_BINARY);
    // cv::medianBlur(img_tt, img_tt, configData.blur_kernel);
    // cv::GaussianBlur(img, img, Size(3,3), 0);
    // cv::fastNlMeansDenoising(img, img, std::vector<float>({120}));
    cv::Canny(grayimg, edge_img, configData.canny_1, configData.canny_2, configData.canny_3);
   if (roi_points.size() >= 2) {
       cv::Mat mask = cv::Mat::ones(edge_img.size(), CV_8UC1);
       for (auto it = roi_points.begin()+1; it < roi_points.end(); it++)
           cv::line(edge_img, *(it - 1), *it, Scalar(0), 15);
       cv::line(edge_img, *roi_points.begin(),*roi_points.rbegin(),
                Scalar(0), 15);
   }

    edge_up = edge_img.clone();
    edge_down = edge_img.clone();

    for (int i = 0; i < edge_img.rows; i++) {
        for (int j = 0; j < edge_img.cols; j++) {
            double result = PointRelativeToLineUp(cv::Point(configData.seprate_p1x, configData.seprate_p1y),
            cv::Point(configData.seprate_p2x, configData.seprate_p2y), cv::Point(j, i));
            if (result <= 0) {
                edge_down.at<uchar>(i, j) = 0;
            } else {
                edge_up.at<uchar>(i, j) = 0;
            }
        }
    }
    if (debug_win_enable) {
        cv::Mat uptt;
        cv::Mat downtt;
        edge_up.copyTo(uptt);
        edge_down.copyTo(downtt);
        cv::resize(uptt, uptt, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%
        cv::resize(downtt, downtt, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%
        imshow("edge_up", uptt);
        imshow("edge_down", downtt);
    }
}

#define CONTOUR_MODE 0

bool ImgProcess::Process(cv::Mat &edge_img, std::vector<cv::Vec2f> & lines_found, bool up) {

        #if CONTOUR_MODE
        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edge_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 筛选最上面的边
        std::vector<cv::Point> top_edge;
        double min_y_center = numeric_limits<double>::max();

        if (debug_win_enable) {
            cv::Mat image = Mat::zeros(edge_img.size(), CV_8UC3);
        }

        for (auto& contour : contours) {
            std::vector<cv::Point> approx;        
            double epsilon = 0.02 * cv::arcLength(contour, true);
            cv::approxPolyDP(contour, approx, epsilon, true);
            // 计算凸包
            // cv::convexHull(contour, approx);
            if (debug_win_enable) {
                cv::polylines(image, approx, true, Scalar(0, 255, 255), 2); // 蓝色多边形
            }

            if (approx.size() >= 2) {
                // 计算边的中心点Y坐标
                int y_min = contour[0].y;
                int y_max = contour[0].y;
                for (auto& p : approx) {
                    if (!up) {
                        if (p.y < y_min) y_min = p.y;
                        if (p.y > y_max) y_max = p.y;
                    } else {
                        if (p.y > y_min) y_min = p.y;
                        if (p.y < y_max) y_max = p.y;
                    }
                }
                double y_center = (y_min + y_max) / 2.0;

                if (!up) {
                    if (y_center < min_y_center) {
                        min_y_center = y_center;
                        top_edge = approx;
                    }
                } else {
                    if (y_center > min_y_center) {
                        min_y_center = y_center;
                        top_edge = approx;
                    }
                }
            }
        }

        if (debug_win_enable) {
            if (!top_edge.empty()) {
                for (auto& p : top_edge) {
                    cv::circle(image, p, 5, Scalar(0, 255, 0), -1);
                }
                cv::polylines(image, top_edge, true, Scalar(0, 0, 255), 2);
            }
            cv::resize(image, image, cv::Size(), 0.25, 0.25, cv::INTER_AREA);  // 缩小50%

            if (!up) {
                imshow("Result up", image);
            } else {
                imshow("Result down", image);
            }
        }
    #else
    // cv::HoughLines(edge_img, lines_found,
    //     configData.hgline_1,
    //     configData.hgline_2 == 0 ? CV_PI/180 : CV_PI/360,
    //     configData.hgline_3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edge_img, lines,
        configData.hgline_1,
        configData.hgline_2 == 0 ? CV_PI/180 : CV_PI/360,
        configData.hgline_3,
        configData.hgline_3, configData.hgline_4);
    for(const auto& line : lines) {
        Point pt1(line[0], line[1]);
        Point pt2(line[2], line[3]);
        if (sqrtf(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2)) < configData.hgline_3)
            continue;
        auto pa = PointsToHoughParams(pt1, pt2);
        // qDebug("houghlinesP %d rho %.2f ang %.2f", pos++, pa.first, pa.second);
        lines_found.push_back(cv::Vec2f(pa.first, pa.second));
    }
    #endif
}

extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);

bool ImgProcess::FilterLines(int rows, int cols, std::vector<cv::Vec2f> &lines_found, bool up) {
    const int IMG_HEIGHT = rows;
    const int IMG_WIDTH = cols;
    const float ROH_ABS = sqrtf(IMG_HEIGHT*IMG_HEIGHT + IMG_WIDTH*IMG_WIDTH) * configData.line_roh_abs;
    const float ANG_ABS = CV_PI * configData.line_ang_abs;

    for (auto it = g_lines.begin(); it != g_lines.end(); ++it) {
        size_t index = std::distance(g_lines.begin(), it);  // 实际索引位置
        if (up) {
            if (index > 2) {
                continue;
            }
        } else {
            if (index < 3) {
                continue;
            }
        }
        it->lines_filterd.clear();
        it->points_in_img.clear();
    }

    for ( auto  ilt = lines_found.begin(); ilt != lines_found.end();) {
        float rho = (*ilt)[0];
        float thetatt = (*ilt)[1];
        float theta = thetatt;

        auto valid_line_check = [&] (float rho, float theta) -> bool {
            bool ret = false;
            for (auto it = g_lines.begin(); it != g_lines.end(); ++it) {
                size_t index = std::distance(g_lines.begin(), it);  // 实际索引位置
                if (up) {
                    if (index > 2) {
                        continue;
                    }
                } else {
                    if (index < 3) {
                        continue;
                    }
                }
                float rho_norm = abs(it->rho - rho);
                float ang_norm = abs(sin(it->angle) - sin(theta));
//                spdlog::debug("{:.2f} -> {:.2f}, {:.2f} -> {:.2f}", rho, v.rho, theta, v.angle);
                // float delta = sqrtf(rho_norm * rho_norm + ang_norm * ang_norm);
                // if (delta < configData.line_roh_abs) {
                if (rho_norm < ROH_ABS && ang_norm < ANG_ABS) {
                    ret = true;
                    it->lines_filterd.push_back(std::pair<float, float>(rho, theta));
//                     spdlog::debug("{:.2f} -> {:.2f}, {:.2f} -> {:.2f}", rho, v.rho, theta, v.angle);
                    break;
                }
            }
            return ret;
        };

        if (!valid_line_check(rho, theta)) {
            lines_found.erase(ilt);
            continue;
        } else {
            // spdlog::debug("++++ valid line rho,ang {:.2f},{:.2f}", rho, theta);
            ilt++;
        }
    }

    for (auto it = g_lines.begin(); it != g_lines.end(); ++it) {
        size_t index = std::distance(g_lines.begin(), it);  // 实际索引位置
        if (up) {
            if (index > 2) {
                continue;
            }
        } else {
            if (index < 3) {
                continue;
            }
        }
        if (it->lines_filterd.size() != 0) {
            float rho = 0.0, ang = 0.0;
            for (const auto &w : it->lines_filterd) {
                rho += w.first;
                ang += w.second;
            }
            rho = rho / it->lines_filterd.size();
            ang = ang / it->lines_filterd.size();
            it->lines_filterd.clear();
            it->lines_filterd.push_back(std::pair<float, float>(rho, ang));
//             qDebug("get ang %.2f, rho %.2f", ang, rho);
            cv::Point2i pt1(static_cast<int>(rho / cos(ang)), 0);
            cv::Point2i pt2(static_cast<int>(rho / cos(ang) - IMG_HEIGHT * sin(ang) / cos(ang)), IMG_HEIGHT);
            if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                if (pt1.y > pt2.y) {
                    it->points_in_img.push_back(std::pair<int, int>(pt2.x, pt2.y));
                    it->points_in_img.push_back(std::pair<int, int>(pt1.x, pt1.y));
                } else {
                    it->points_in_img.push_back(std::pair<int, int>(pt1.x, pt1.y));
                    it->points_in_img.push_back(std::pair<int, int>(pt2.x, pt2.y));
                }
            }
        }
    }

    return true;
}

bool ImgProcess::AdaptLines(std::vector<cv::Vec2f> &lines_found,
    std::vector<std::vector<std::pair<double, double>>> &lines_filtered) {

    if (lines_found.size() > 1000 || lines_found.size() < configData.lines_num) {
        spdlog::debug("lines found {} invalid!", lines_found.size());
        return false;
    } else {
//        for (auto &v: lines_found) {
//            spdlog::debug("rho {:.2f}, theta {:.2f}", v[0], v[1]);
//        }
    }

    cv::Mat kmeans(lines_found.size(), 2, CV_32F, Scalar(0));
    cv::Mat bestlabels;
    cv::Mat centers(configData.lines_num, 2, CV_32F, Scalar(0));
    for (int pos=0; pos < lines_found.size();pos++) {
        kmeans.at<float>(pos, 0) = lines_found[pos][0];
        kmeans.at<float>(pos, 1) = lines_found[pos][1];
    }

    for (int pos=0; pos < configData.lines_num;pos++) {
        centers.at<float>(pos, 0) = 0;
        centers.at<float>(pos, 1) = 2.5;
    }

    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.02);
    // spdlog::debug("kmeans start");
    cv::kmeans(kmeans,
         configData.lines_num,
         bestlabels,
         criteria,
         configData.lines_num,
         cv::KMEANS_PP_CENTERS,
         centers);
        //  spdlog::debug("kmeans end");

    // if (centers.rows < configData.lines_num) {
    //     spdlog::debug("no valid center found!");
    //     return false;
    // }
    std::vector<std::pair<double, double>> lines;
    for (int row = 0; row < configData.lines_num; row++) {
        std::pair<double, double> line = std::pair<double, double>(centers.at<float>(row, 0), centers.at<float>(row, 1));
        lines.push_back(line);
    }
    lines_filtered.push_back(lines);

    return true;
}

bool GetCentralLines(const std::vector<std::vector<std::pair<double, double>>> &lines_filtered,
    std::vector<cv::Point2f> &line1,
    std::vector<cv::Point2f> &line2) {
    if (lines_filtered.size() != 2) {
            spdlog::debug("lines filtered size {} invalid!", lines_filtered.size());
        return false;
    }

    auto findOutlier = [] (float a, float b, float c) -> int {
        // 方法一：差值对比法
        float aa = sin(a);
        float bb = sin(b);
        float cc = sin(c);
        float d1 = fabs(aa - bb);
        float d2 = fabs(aa - cc);
        float d3 = fabs(bb - cc);
        
        // 找出最大差值对
        if (d1 > d2 && d1 > d3) {         // a-b差异最大
            return (fabs(aa - cc) < fabs(bb - cc)) ? 1 : 0;
        } else if (d2 > d3) {             // a-c差异最大
            return (fabs(aa - bb) < fabs(cc - bb)) ? 2 : 0;
        } else {                          // b-c差异最大
            return (fabs(bb - aa) < fabs(cc - aa)) ? 2 : 1;
        }
    };
    
    int line_pos = 0;
    for (const auto &v : lines_filtered) {
        if (v.size() != 3) {
            spdlog::debug("line size {} invalid!", v.size());
            return false;
        }

        double l0_ang = v[0].second;
        double l1_ang = v[1].second;
        double l2_ang = v[2].second;
        int tgt_ang = findOutlier(l0_ang, l1_ang, l2_ang);
        std::pair<double, double> p0, p_mid, p1;
        if (tgt_ang == 0) {
            p0 = getCrossPoint(v[0], v[1]);
            p1 = getCrossPoint(v[0], v[2]);
        } else if (tgt_ang == 1) {
            p0 = getCrossPoint(v[1], v[0]);
            p1 = getCrossPoint(v[1], v[2]);
        } else {
            p0 = getCrossPoint(v[2], v[0]);
            p1 = getCrossPoint(v[2], v[1]);
        }
        p_mid.first = (p1.first + p0.first) / 2;
        p_mid.second = (p1.second + p0.second) / 2;
        if (line_pos == 0) {
            line1.push_back(cv::Point2f(p0.first, p0.second));
            line1.push_back(cv::Point2f(p_mid.first, p_mid.second));
            line1.push_back(cv::Point2f(p1.first, p1.second));
        } else {
            line2.push_back(cv::Point2f(p0.first, p0.second));
            line2.push_back(cv::Point2f(p_mid.first, p_mid.second));
            line2.push_back(cv::Point2f(p1.first, p1.second));
        }
        line_pos++;
    }

    return true;
}

