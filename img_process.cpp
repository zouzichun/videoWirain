#include "img_process.h"
#include "crcalgorithm.h"
#include "maindialog.h"
#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace std;
using namespace cv;

ImgProcess::ImgProcess(QString dev_name) :
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

struct win_info {
    win_info(int a, float b) :
    pos(a),
    h(b){}

    int pos;
    float h;
};

const int CONV_WIN_WIDTH = 21;
const int CONV_WIN_STRIDE = 3;

void ThinSubiteration1(Mat & pSrc, Mat & pDst) {
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
                                        int N = min(N1,N2);
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


void ThinSubiteration2(Mat & pSrc, Mat & pDst) {
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

void NormalizeLine(Mat & inputarray, Mat & outputarray) {
        bool bDone = false;
        int rows = inputarray.rows;
        int cols = inputarray.cols;

        inputarray.convertTo(inputarray,CV_32FC1);

        inputarray.copyTo(outputarray);

        outputarray.convertTo(outputarray,CV_32FC1);

        /// pad source
        Mat p_enlarged_src = Mat(rows + 2, cols + 2, CV_32FC1);
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
        Mat p_thinMat1 = Mat::zeros(rows + 2, cols + 2, CV_32FC1);
        Mat p_thinMat2 = Mat::zeros(rows + 2, cols + 2, CV_32FC1);
        Mat p_cmp = Mat::zeros(rows + 2, cols + 2, CV_8UC1);

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


void LineReflect(Mat & inputarray, Mat & outputarray)
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

void FilterSmallRegions(Mat & pSrc, Mat & pDst) {
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

Point2f getCrossPoint(Vec4i LineA, Vec4i LineB)
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

    Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return crossPoint;
}


// 通过两个点计算霍夫参数 (rho, theta)
void pointsToHoughParams(const cv::Point& p1, 
        const cv::Point& p2,
        float& rho, 
        float& theta) {
    // 计算直线方程参数
    const float A = p2.y - p1.y;
    const float B = p1.x - p2.x;
    const float C = p2.x * p1.y - p1.x * p2.y;

    // 计算霍夫参数
    const float denominator = std::sqrt(A*A + B*B);
    if (denominator == 0) { // 处理重合点的情况
        rho = 0;
        theta = 0;
        return;
    }

    // 计算角度（弧度制）
    theta = std::atan2(B, A);  // 法线方向角度
    rho = C / denominator;

    // 规范角度到 [0, π) 范围
    if (rho < 0) {
        rho = -rho;
        theta += CV_PI;
    }

    theta = fmod(theta, CV_PI);
}
