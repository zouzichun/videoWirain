#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "crcalgorithm.h"
#include "maindialog.h"
#include "img_process.h"

using namespace std;
using namespace cv;

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

cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB)
{
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

std::pair<double, double> PointsToHoughParams(cv::Point p1, cv::Point p2) {
    double A = p2.y - p1.y;
    double B = p1.x - p2.x;
    double C = p2.x * p1.y - p1.x * p2.y;
    double theta_rad = atan2(B, A); // 正确获取法线方向角度
    double rho = C / sqrt(A * A + B * B);
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

    // OpenCV规范化处理
    // if(rho < 0) {
    // rho = -rho;
    // theta_rad += CV_PI;
    // }

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
    auto new_line = PointsToHoughParams(cv::Point(rx1, ry1), cv::Point(rx2, ry2));
    return new_line;
}


// 通过两个点计算霍夫参数 (rho, theta)
// void pointsToHoughParams(const cv::Point& p1, 
//         const cv::Point& p2,
//         float& rho, 
//         float& theta) {
//     // 计算直线方程参数
//     const float A = p2.y - p1.y;
//     const float B = p1.x - p2.x;
//     const float C = p2.x * p1.y - p1.x * p2.y;

//     // 计算霍夫参数
//     const float denominator = std::sqrt(A*A + B*B);
//     if (denominator == 0) { // 处理重合点的情况
//         rho = 0;
//         theta = 0;
//         return;
//     }

//     // 计算角度（弧度制）
//     theta = std::atan2(B, A);  // 法线方向角度
//     rho = C / denominator;

//     // 规范角度到 [0, π) 范围
//     if (rho < 0) {
//         rho = -rho;
//         theta += CV_PI;
//     }

//     theta = fmod(theta, CV_PI);
// }


bool ImgProcess::Process(cv::Mat &img, cv::Mat &edge_img, cv::Mat &contour_img, std::vector<cv::Vec2f> & lines_found) {
    const int IMG_HEIGHT = img.rows;
    const int IMG_WIDTH = img.cols;
    cv::Mat img_tt;
    // cv::bilateralFilter(img, img_tt, 0, 200, 10);
    // cv::GaussianBlur(img, img, Size(configData.blur_kernel,configData.blur_kernel), 0);
    cv::threshold(img, img_tt, configData.inv_thd, 255, THRESH_BINARY);
    cv::medianBlur(img_tt, img_tt, configData.blur_kernel);
    // cv::GaussianBlur(img, img, Size(3,3), 0);
    // cv::fastNlMeansDenoising(img, img, std::vector<float>({120}));
    cv::Canny(img_tt, edge_img, configData.canny_1, configData.canny_2, configData.canny_3);

    // for (int i = 0; i < edge_img.rows / 2; i++)
    //     for (int j = 0; j < edge_img.cols; j++)
    //         if (edge_img.at<uchar>(i, j) > 0)
    //             edge_img.at<uchar>(i, j) = 0;

//    std::vector<vector<Point>> contours;
//    std::vector<Point> hull_points;
//    std::vector<cv::Vec4i> hierarchy;

//    cv::findContours(edge_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//    // spdlog::debug("findContours num {}", contours.size());
//    // cv::Mat contour_img(edge_img.size(), CV_8UC1, Scalar(0));
//    for (size_t i = 0; i < contours.size(); i++)
//    {
//        // spdlog::debug("  findContours {} {}", i, contours[i].size());
//        if(contours[i].size() > 3000) {
//            // cv::drawContours(contours_img, contours, static_cast<int>(i), Scalar(255), 2, LINE_8, hierarchy, 0);
//            // spdlog::debug("  findContours {} {}", i, contours[i].size());
//            cv::convexHull(contours[i], hull_points, false, true);
//            // spdlog::debug("convhull points {}", hull_points.size());
//            std::vector<vector<Point>> hullv;
//            hullv.push_back((hull_points));
//            cv::drawContours(contour_img, hullv, 0, Scalar(255), 2, LINE_8, hierarchy, 0);
//        }
//    }

    cv::HoughLines(edge_img, lines_found,
        configData.hgline_1,
        configData.hgline_2 == 0 ? CV_PI/180 : CV_PI/360,
        configData.hgline_3);
}

extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);

bool ImgProcess::FilterLines(cv::Mat &img, std::vector<cv::Vec2f> & lines_found,
    std::vector<float> & rhos, std::vector<float> & thetas, std::vector<cv::Point2i> & points) {
    if (rhos.size() != thetas.size() || thetas.size() == 0) {
        spdlog::debug("invalid find lines param size!");
        return false;
    }
    const int IMG_HEIGHT = img.rows;
    const int IMG_WIDTH = img.cols;

    const float ROH_ABS = sqrtf(IMG_HEIGHT*IMG_HEIGHT + IMG_WIDTH*IMG_WIDTH) * configData.line_roh_abs;
    const float ANG_ABS = configData.line_ang_abs / CV_PI;
    const float ang1 = ((180 - configData.line1_ang) % 180) * CV_PI / 180;
    const float ang2 = ((180 - configData.line2_ang) % 180) * CV_PI / 180;
    spdlog::debug("ang1 {}C:{:.2f}, ang2 {}C:{:.2f}, abs {:.2f} sel_low {} {}",
                  configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_roh_abs,
                  configData.line1_sel_low,
                  configData.line2_sel_low);
    
    if (lines_found.size() == 0) {
        // spdlog::debug("found lines num 0");
        return false;
    }
    // spdlog::debug("found lines num {}", lines_found.size());

    std::vector<int> cnts;
    for (const auto v : thetas) {
        cnts.push_back(0);
    }
    for ( auto  it = lines_found.begin(); it != lines_found.end();) {
        float rho = (*it)[0];
        float thetatt = (*it)[1];
        float theta = thetatt;
        if (thetatt >= CV_PI)
            theta = thetatt - CV_PI;
        // spdlog::debug("rho {:.2f}, theta {:.2f}, {:.2f}", rho, thetatt, theta * 180 / CV_PI);

        auto valid_line_check = [&] () -> bool {
            return (abs(theta - ang1) <= ANG_ABS && abs(abs(rho) - configData.line1_roh) <= ROH_ABS) ||
                    (abs(theta - ang2) <= ANG_ABS && abs(abs(rho) - configData.line2_roh) <= ROH_ABS);
        };

        if (!valid_line_check()) {
            lines_found.erase(it);
            continue;
        } else {
            it++;
            if (abs(theta - ang1) <= ANG_ABS && abs(abs(rho) - configData.line1_roh) <= ROH_ABS) {
                thetas[0] += theta;
                rhos[0] += rho;
                cnts[0]++;
            } else if (abs(theta - ang2) <= ANG_ABS && abs(abs(rho) - configData.line2_roh) <= ROH_ABS) {
                thetas[1] += theta;
                rhos[1] += rho;
                cnts[1]++;
            }
        }
    }

    if (cnts[0] != 0) {
        rhos[0] = rhos[0] / cnts[0];
        thetas[0] = thetas[0] / cnts[0];
    }
    if (cnts[1] != 0) {
        rhos[1] = rhos[1]  / cnts[1] ;
        thetas[1]  = thetas[1]  / cnts[1] ;
    }

    std::vector<cv::Vec4i> lines;
    for (int pos = 0; pos < 2; pos++)
    {
        float rho = rhos[pos];
        float theta = thetas[pos];
        // spdlog::debug("line {} rho {:.2f}, theta {:.2f}, cnt {}", pos, rho, theta, cnts[pos]);
            //直线与第一行的交叉点
            cv::Point2i pt1(static_cast<int>(rho / cos(theta)), 0);
            //直线与最后一行的交叉点
            cv::Point2i pt2(static_cast<int>(rho / cos(theta) - IMG_HEIGHT*sin(theta) / cos(theta)), IMG_HEIGHT);
            cv::line(img, pt1, pt2, cv::Scalar(255), 2);
            if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                if (pt1.y > pt2.y) {
                    lines.push_back(cv::Vec4i(pt2.x, pt2.y, pt1.x, pt1.y));
                } else {
                    lines.push_back(cv::Vec4i(pt1.x, pt1.y, pt2.x, pt2.y));
                }
            }
    }

    double ka, kb;
    ka = (double)(lines[0][3] - lines[0][1]) / (double)(lines[0][2] - lines[0][0]); //求出LineA斜率
    kb = (double)(lines[1][3] - lines[1][1]) / (double)(lines[1][2] - lines[1][0]); //求出LineB斜率

    cv::Point2f p;
    p.x = (ka*lines[0][0] - lines[0][1] - kb*lines[1][0] + lines[1][1]) / (ka - kb);
    p.y = (ka*kb*(lines[0][0] - lines[1][0]) + ka*lines[1][1] - kb*lines[0][1]) / (ka - kb);
    // qDebug("ka %f, kb %f, p (%f, %f)", ka, kb, p.x, p.y);

    cv::Point2i p_int(int(p.x), int(p.y));
    cv::Point2i line1p, line2p;
    if (configData.line1_sel_low == 0) {
        if (lines[0][1] < lines[0][3]) {
            line1p.x = lines[0][2];
            line1p.y = lines[0][3];
        } else {
            line1p.x = lines[0][0];
            line1p.y = lines[0][1];
        }
    } else {
        if (lines[0][1] < lines[0][3]) {
            line1p.x = lines[0][0];
            line1p.y = lines[0][1];
        } else {
            line1p.x = lines[0][2];
            line1p.y = lines[0][3];
        }
    }

    if (configData.line2_sel_low == 0) {
        if (lines[1][1] < lines[1][3]) {
            line2p.x = lines[1][2];
            line2p.y = lines[1][3];
        } else {
            line2p.x = lines[1][0];
            line2p.y = lines[1][1];
        }
    } else {
        if (lines[1][1] < lines[1][3]) {
            line2p.x = lines[1][0];
            line2p.y = lines[1][1];
        } else {
            line2p.x = lines[1][2];
            line2p.y = lines[1][3];
        }
    }   

    points.clear();
    points.push_back(line1p);
    points.push_back(p_int);
    points.push_back(line2p);
    return true;
}
        

bool ImgProcess::FilterLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found) {
    const int IMG_HEIGHT = img.rows;
    const int IMG_WIDTH = img.cols;
    const float ROH_ABS = sqrtf(IMG_HEIGHT*IMG_HEIGHT + IMG_WIDTH*IMG_WIDTH) * configData.line_roh_abs;
    const float ANG_ABS = CV_PI * configData.line_ang_abs;

    for (auto & v : g_lines) {
        v.lines_filterd.clear();
        v.points_in_img.clear();
    }

    for ( auto  it = lines_found.begin(); it != lines_found.end();) {
        float rho = (*it)[0];
        float thetatt = (*it)[1];
        float theta = thetatt;
//        if (thetatt >= CV_PI)
//            theta = thetatt - CV_PI;
//        spdlog::debug("rho {:.2f}, theta {:.2f}, {:.2f}", rho, thetatt, theta * 180 / CV_PI);

        auto valid_line_check = [&] (float rho, float theta) -> bool {
            bool ret = false;
            for (auto & v : g_lines) {
                float rho_norm = abs(v.rho - rho);
                float ang_norm = abs(sin(v.angle) - sin(theta));

                // float delta = sqrtf(rho_norm * rho_norm + ang_norm * ang_norm);
                // if (delta < configData.line_roh_abs) {
                if (rho_norm < ROH_ABS && ang_norm < ANG_ABS) {
                    ret = true;
                    v.lines_filterd.push_back(std::pair<float, float>(abs(rho), theta));
                    // spdlog::debug("{:.2f} -> {:.2f}, {:.2f} -> {:.2f}", rho, v.rho, theta, v.angle);
                    break;
                }
            }
            return ret;
        };

        if (!valid_line_check(rho, theta)) {
            lines_found.erase(it);
            continue;
        } else {
            // spdlog::debug("++++ valid line rho,ang {:.2f},{:.2f}", rho, theta);
            it++;
        }
    }

    for (auto & v : g_lines) {
        if (v.lines_filterd.size() != 0) {
            float rho = 0.0, ang = 0.0;
            for (const auto &w : v.lines_filterd) {
                rho += w.first;
                ang += w.second;
            }
            rho = rho / v.lines_filterd.size();
            ang = ang / v.lines_filterd.size();
            v.lines_filterd.clear();
            v.lines_filterd.push_back(std::pair<float, float>(rho, ang));
//             qDebug("get ang %.2f, rho %.2f", ang, rho);
            cv::Point2i pt1(static_cast<int>(rho / cos(ang)), 0);
            cv::Point2i pt2(static_cast<int>(rho / cos(ang) - IMG_HEIGHT * sin(ang) / cos(ang)), IMG_HEIGHT);
            if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                if (pt1.y > pt2.y) {
                    v.points_in_img.push_back(std::pair<int, int>(pt2.x, pt2.y));
                    v.points_in_img.push_back(std::pair<int, int>(pt1.x, pt1.y));
                } else {
                    v.points_in_img.push_back(std::pair<int, int>(pt1.x, pt1.y));
                    v.points_in_img.push_back(std::pair<int, int>(pt2.x, pt2.y));
                }
            }
        }
    }

    return true;
}


bool ImgProcess::AdaptLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
    std::vector<float> & rhos, std::vector<float> & thetas) {
    if (rhos.size() != thetas.size() || thetas.size() == 0) {
        spdlog::debug("invalid find lines param size!");
        return false;
    }

    const float ang1 = ((90 + configData.line1_ang) % 180) * CV_PI / 180;
    const float ang2 = ((90 + configData.line2_ang) % 180) * CV_PI / 180;
    const float ang_abs = configData.line_ang_abs * CV_PI;
    const float roh_abs = configData.line_roh_abs * 2800;

    if (lines_found.size() > 1000 || lines_found.size() < configData.lines_num) {
        spdlog::debug("lines found {} invalid!", lines_found.size());
        return false;
    } else {
//        for (auto &v: lines_found) {
//            spdlog::debug("rho {:.2f}, theta {:.2f}", v[0], v[1]);
//        }
    }
    
    std::vector<int> cnts;
    for (const auto v : thetas) {
        cnts.push_back(0);
    }

    cv::Mat kmeans(lines_found.size(), 2, CV_32F, Scalar(0));
    cv::Mat bestlabels;
    cv::Mat centers(configData.lines_num, 1000, CV_32F, Scalar(0));
    for (int pos=0; pos < lines_found.size();pos++) {
        kmeans.at<float>(pos, 0) = lines_found[pos][0];
        kmeans.at<float>(pos, 1) = lines_found[pos][1];
    }

    for (int pos=0; pos < configData.lines_num;pos++) {
        centers.at<float>(pos, 0) = 1000;
        centers.at<float>(pos, 1) = 0;
    }

    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.02);
    cv::kmeans(kmeans,
         configData.lines_num,
         bestlabels,
         criteria,
         configData.lines_num,
         cv::KMEANS_PP_CENTERS,
         centers);
    // spdlog::debug("lines found {}, center size {}:{}", lines_found.size(), centers.rows, centers.cols);
    if (centers.rows < configData.lines_num) {
        spdlog::debug("no valid center found!");
        return false;
    }

    for (int pos=0; pos < centers.rows; pos++) {
        cv::Mat roww = centers.row(pos);
        if (abs(roww.at<float>(1) - ang1) < ang_abs) {
            rhos[0] = roww.at<float>(0);
            thetas[0] = roww.at<float>(1);
//            spdlog::debug("center {}, {:.2f}:{:.2f}", pos, rhos[0], thetas[0]);
        } else if (abs(roww.at<float>(1) - ang2) < ang_abs) {
            rhos[1] = roww.at<float>(0);
            thetas[1] = roww.at<float>(1);
//            spdlog::debug("center {}, {:.2f}:{:.2f}", pos, rhos[1], thetas[1]);
        } else {
            continue;
        }
    }

    return true;
}

