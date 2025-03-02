#include "img_process.h"
#include "crcalgorithm.h"
#include "maindialog.h"
#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

#include <vector>
#include <cmath>
#include <utility>


#include <rknn_api.h>

using namespace std;
using namespace cv;

ImgProcess::ImgProcess(QString model_name) :
m_model_name(model_name) {
    Init();
}

ImgProcess::~ImgProcess() {
    Deinit();
}

bool ImgProcess::Init() {
    m_rknn = new RknnProcess(QString("/home/leon/model/cnn_line.rknn"));
    return true;
}

bool ImgProcess::Deinit() {
    delete m_rknn;
    return true;
}

void ImgProcess::imgProcTimeout() {
}

float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB) {
    int A = 0, B = 0, C = 0;
    A = pointA.y - pointB.y;
    B = pointB.x - pointA.x;
    C = pointA.x*pointB.y - pointA.y*pointB.x;
    float distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / ((float)sqrtf(A*A + B*B));
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
    ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
    kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率

    Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return crossPoint;
}

void sigmoid(const cv::Mat& src, cv::Mat& dst) {
    // 确保输入是浮点型（CV_32F 或 CV_64F）
    cv::Mat src_float;
    src.convertTo(src_float, CV_32F);
    cv::Mat neg_src = src_float.clone();
    
    // 计算 -src
    // std::negate(src_float, neg_src);
    for (int y = 0; y < src_float.rows; y++) {
        for (int x = 0; x < src_float.cols; x++) {
            neg_src.at<cv::Vec2f>(y, x) = -src_float.at<cv::Vec2f>(y, x);
        }
    }
    
    // 计算 e^(-x)
    cv::exp(neg_src, dst);
    
    // 计算 1 + e^(-x)
    cv::add(dst, 1.0, dst);
    
    // 取倒数得到 1/(1 + e^(-x))
    cv::divide(1.0, dst, dst);
}

std::vector<RegionProps> regionprops(const cv::Mat& binaryImage) {
    std::vector<RegionProps> props;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 查找轮廓
    cv::findContours(binaryImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        RegionProps prop;

        // 计算面积
        prop.area = cv::contourArea(contour);

        // 计算质心
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 != 0) {
            prop.centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
        } else {
            continue;
            // prop.centroid = cv::Point(0, 0);
        }

        // 计算边界框
        prop.boundingBox = cv::boundingRect(contour);

        props.push_back(prop);
    }

    return props;
}

std::pair<std::pair<int, int>, std::pair<int, int>> get_boundary_point(int y, int x, double angle, int H, int W) {
    std::pair<int, int> point1 = {-1, -1};
    std::pair<int, int> point2 = {-1, -1};

    if (angle == -M_PI / 2) {
        point1 = {x, 0};
        point2 = {x, H - 1};
    } else if (angle == 0.0) {
        point1 = {0, y};
        point2 = {W - 1, y};
    } else {
        double k = std::tan(angle);
        if (y - k * x >= 0 && y - k * x < H) {  // left
            if (point1.first == -1) {
                point1 = {0, static_cast<int>(y - k * x)};
            } else if (point2.first == -1) {
                point2 = {0, static_cast<int>(y - k * x)};
                if (point2 == point1) point2 = {-1, -1};
            }
        }
        if (k * (W - 1) + y - k * x >= 0 && k * (W - 1) + y - k * x < H) {  // right
            if (point1.first == -1) {
                point1 = {W - 1, static_cast<int>(k * (W - 1) + y - k * x)};
            } else if (point2.first == -1) {
                point2 = {W - 1, static_cast<int>(k * (W - 1) + y - k * x)};
                if (point2 == point1) point2 = {-1, -1};
            }
        }
        if (x - y / k >= 0 && x - y / k < W) {  // top
            if (point1.first == -1) {
                point1 = {static_cast<int>(x - y / k), 0};
            } else if (point2.first == -1) {
                point2 = {static_cast<int>(x - y / k), 0};
                if (point2 == point1) point2 = {-1, -1};
            }
        }
        if (x - y / k + (H - 1) / k >= 0 && x - y / k + (H - 1) / k < W) {  // bottom
            if (point1.first == -1) {
                point1 = {static_cast<int>(x - y / k + (H - 1) / k), H - 1};
            } else if (point2.first == -1) {
                point2 = {static_cast<int>(x - y / k + (H - 1) / k), H - 1};
                if (point2 == point1) point2 = {-1, -1};
            }
        }
        if (point2.first == -1) point2 = point1;
    }
    return {point1, point2};
}

// 新函数：使用 OpenCV 的 clipLine 实现
std::pair<std::pair<int, int>, std::pair<int, int>> 
get_boundary_point_clipLine(int y, int x, double angle, int H, int W) {
    // 注意参数顺序：原函数参数为 (y, x)，这里需转换为图像坐标系 (x, y)
    cv::Point pt1(x, y); // OpenCV 使用 (列, 行) 坐标

    // 沿角度方向生成足够长的线段终点
    const double kMaxLength = 10000.0; // 足够大的长度确保与边界相交
    cv::Point pt2(
        x + static_cast<int>(kMaxLength * std::cos(angle)),
        y + static_cast<int>(kMaxLength * std::sin(angle))
    );

    // 定义图像边界矩形 (左闭右开区间 [0, W) x [0, H))
    cv::Rect img_rect(0, 0, W, H);

    // 裁剪线段
    bool is_inside = cv::clipLine(img_rect, pt1, pt2);

    // 返回结果
    if (is_inside) {
        return { {pt1.x, pt1.y}, {pt2.x, pt2.y} };
    } else {
        return { {-1, -1}, {-1, -1} };
    }
}

std::vector<cv::Point2i> get_boundary_point(int y, int x, float angle, int H, int W) {
    cv::Point2i p1 = {-1, -1};
    cv::Point2i p2 = {-1, -1};
    if (angle == -M_PI / 2) {
        p1 = {x, 0};
        p2 = {x, H - 1};
    } else if (angle == 0.0) {
        p1 = {0, y};
        p2 = {W - 1, y};
    } else {
        float k = std::tan(angle);
        if ((y - k*x) >= 0 && (y - k*x) < H) {
            if (p1.x == -1 || p1.y == -1) {
                p1 = {0, int(y - k*x)};
            } else if (p2.x == -1 || p2.y == -1) {
                p2 = {0, int(y - k*x)};
                if (p2.x == p1.x && p2.y == p1.y) {
                    p2 = {-1, -1};
                }
            }
        }

        if ((k*(W-1)+y-k*x) >= 0 && (k*(W-1)+y-k*x) < H) {
            if (p1.x == -1 || p1.y == -1) {
                p1 = {W - 1, int(k*(W-1)+y-k*x)};
            } else if (p2.x == -1 || p2.y == -1) {
                p2 = {W - 1, int(k*(W-1)+y-k*x)};
                if (p2.x == p1.x && p2.y == p1.y) {
                    p2 = {-1, -1};
                }
            }
        }

        if ((x - y/k) >= 0 && (x - y/k) < W) {
            if (p1.x == -1 || p1.y == -1) {
                p1 = {int(x - y/k), 0};
            } else if (p2.x == -1 || p2.y == -1) {
                p2 = {int(x - y/k), 0};
                if (p2.x == p1.x && p2.y == p1.y) {
                    p2 = {-1, -1};
                }
            }
        }

        if ((x-y/k+(H-1)/k) >= 0 && (x-y/k+(H-1)/k) < W) {
            if (p1.x == -1 || p1.y == -1) {
                p1 = {int(x-y/k+(H-1)/k), H-1};
            } else if (p2.x == -1 || p2.y == -1) {
                p2 = {int(x-y/k+(H-1)/k), H-1};
                if (p2.x == p1.x && p2.y == p1.y) {
                    p2 = {-1, -1};
                }
            }
        }

        if (p2.x == -1 && p2.y == -1) {
            p2 = {p1.x, p1.y};
        }
    }
    return {p1, p2};
}

// 反向映射函数
std::vector<std::vector<int>> reverse_mapping(
    const std::vector<cv::Point2f>& point_list,
    int numAngle,
    int numRho,
    std::pair<int, int> size) {
    const int H = size.first;
    const int W = size.second;

    float irho = (std::sqrt(H*H + W*W) + 1) / (numRho - 1); // Rho 的平方根
    float itheta = M_PI / numAngle; // 角度步长

    printf("itheta = %.4f, irho = %.4f\n", itheta, irho);

    std::vector<std::vector<int>> points_out;

    // 遍历所有输入点
    for (const auto& p : point_list) {
        // 遍历所有角度
        float theta = itheta * p.x;
        float r = p.y - numRho / 2;

        float cosi = std::cos(theta) / irho;
        float sini = std::sin(theta) / irho;

        printf("x = %.3f, y = %.3f, theta=%.3f, r=%.3f, sini = %.3f, cosi = %.3f\n", p.x, p.y, theta, r, sini, cosi);
        if (theta == 0.0f) {
            float x = std::round(r / cosi + W/2);
            points_out.push_back({0, int(x), H-1, int(x)});
        } else {
            float angle = std::atan(- cosi / sini);
            float y = std::round(r / sini + W * cosi / sini /2 + H/2);

            // printf("k = %.4f, angle=%.4f, b=%d\n", (-cosi / sini), angle, int(std::round(r / sini + W * cosi / sini / 2 + H / 2)));

            std::vector<cv::Point2i> bp = get_boundary_point(int(y), 0, angle, H, W);
            if (bp[0].x != -1 && bp[0].y != -1 && bp[1].x != -1 && bp[1].y != -1) {
                points_out.push_back({bp[0].y, bp[0].x, bp[1].y, bp[1].x});
            }
        }
    }
    return points_out;
}

std::vector<RegionProps> PostProcess(const cv::Mat& binaryImage) {
    std::vector<RegionProps> props;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 查找轮廓
    cv::findContours(binaryImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        RegionProps prop;

        // 计算面积
        prop.area = cv::contourArea(contour);

        // 计算质心
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 != 0) {
            prop.centroid = cv::Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
        } else {
            continue;
            // prop.centroid = cv::Point(0, 0);
        }

        // 计算边界框
        prop.boundingBox = cv::boundingRect(contour);

        props.push_back(prop);
    }

    return props;
}

QPixmap convertCvMatToQPixmap(const cv::Mat &mat) {
    if (mat.empty() || mat.depth() != CV_8U) {
        return QPixmap();
    }

    QImage img;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
        img = QImage(mat.data, mat.cols, mat.rows, 
                    mat.step, QImage::Format_RGB888);
    } else if (mat.channels() == 1) {
        img = QImage(mat.data, mat.cols, mat.rows,
                    mat.step, QImage::Format_Grayscale8);
    }

    if (img.isNull()) {
        qWarning() << "Failed to convert CV Mat to QImage";
        return QPixmap();
    }
    
    return QPixmap::fromImage(img.copy());
}
