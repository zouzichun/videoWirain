#include "img_process.h"
#include "crcalgorithm.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/spdlog.h"
#include "maindialog.h"
#include <QByteArray>
#include <QDebug>
#include <QPainter>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace std;
using namespace cv;

std::shared_ptr<spdlog::logger> g_logger;

ImgProcess::ImgProcess(QString dev_name) :
m_dev_name(dev_name) {
    std::vector<spdlog::sink_ptr> sinks;
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::level_enum::debug);
    console_sink->set_pattern("[%^%l%$][%H:%M:%S:%e][%n] %v");
    sinks.push_back(console_sink);

    auto is_truncate_log_file = false;
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("debug_log.txt", is_truncate_log_file);
    file_sink->set_level(spdlog::level::level_enum::debug);
    file_sink->set_pattern("[%C-%m-%d %z %H:%M:%S:%e] %v");
    sinks.push_back(file_sink);

    g_logger = std::make_shared<spdlog::logger>("server", begin(sinks), end(sinks));
    g_logger->set_level(spdlog::level::level_enum::debug);
    spdlog::register_logger(g_logger);
    spdlog::set_default_logger(g_logger);
}

bool ImgProcess::Init() {
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

bool ImgProcess::Process(QImage & qimg) {
    unsigned int num = rand() % 16 + 2;
    std::ostringstream stream;
    stream << num;
    string s = stream.str();
    string path = "D:/code/weild_img/img-" + s + ".jpeg";
    qDebug("open : %s", path.c_str());

    Mat img = imread(path.c_str());
    if ( !img.data )
    {
        qDebug("No image data %s", path.c_str());
        return false;
    }

    // sub image = 480 * 320
    Mat binaryImage;
    Mat sub_img(img, cv::Range(480, 960),
                 cv::Range(200, 520));
//    imshow("video", sub_img);

//    int sum_row_num = sub_img.rows;
//    int sum_col_num = sub_img.cols;

    Mat sub_gray;
    cvtColor(sub_img, sub_gray, CV_BGR2GRAY);
    threshold(sub_gray, binaryImage, 200, 255, THRESH_BINARY);
    Mat tmp_img = Mat::zeros(binaryImage.size(), CV_8UC1);

    FilterSmallRegions(binaryImage, tmp_img);
    // imshow("FilterSmallRegions", tmp_img);

    NormalizeLine(tmp_img,binaryImage);
//    imshow("NormalizeLine", binaryImage);
    // qDebug("binaryImage, %d x %d, type %d", binaryImage.rows, binaryImage.cols, binaryImage.type());

    Mat line_img = Mat::zeros(binaryImage.size(), CV_8UC1);
    normalize(binaryImage, line_img, 0, 255, NORM_MINMAX, CV_8U);
    // qDebug("line_img, %d x %d, type %d", line_img.rows, line_img.cols, line_img.type());

    std::vector<cv::Vec3f> lines;
    cv::HoughLines(line_img, lines, 1, CV_PI/180, 50);

    float rho = 0.0f, theta = 0.0f;
    int cnt = 0;
    for (auto idx = lines.begin(); idx != lines.end(); idx++) {
        if((*idx)[1] < 1.4 || (*idx)[1] > 1.7) {
            continue;
        }

        if ((*idx)[2] > 70) {
            cnt++;
            rho += (*idx)[0];
            theta += (*idx)[1];
        }
        qDebug("rho/theta/vote %f , %f, %f", (*idx)[0], (*idx)[1], (*idx)[2]);
    }
    qDebug("horiz lines found size %d", lines.size());

    if(cnt > 0) {
        rho = rho / cnt;
        theta = theta / cnt;
    }

    if (theta < (0.3f * CV_PI) || theta > (0.6f * CV_PI) || rho < 50 || rho > 300) {
        return false;
    }
    qDebug("final rho %f, theta %f, ang %f", rho, theta, theta * 180 / CV_PI);

    int x0 = rho * cos(theta);
    int y0 = rho * sin(theta);
    int x1 = x0 - 500 * sin(theta);
    int y1 = y0 + 500 * cos(theta);
    int x2 = x0 + 500 * sin(theta);
    int y2 = y0 - 500 * cos(theta);

    std::vector<win_info> win_info_vec;
    for (int col_pos = 40;  col_pos < line_img.cols - CONV_WIN_WIDTH - 100; col_pos +=CONV_WIN_STRIDE) {
            float dist = 0.0f;
            int row_min = rho / sin(theta);
            int row_max = -(line_img.cols - 1) / tan(theta) + rho / sin(theta);
            // qDebug("row_min %3d, row_max %d", row_min, row_max);
            for (int row_pos = row_max; row_pos <= row_min; row_pos++) {
                    if (*line_img.ptr<uchar>(row_pos, col_pos) > 210) {
                            dist += getDist_P2L(cv::Point(col_pos, row_pos),
                            cv::Point(x1, y1),
                            cv::Point(x2, y2));
                    }
            }
            win_info_vec.push_back(win_info(col_pos,dist));
            // qDebug("hash %3d, %f", col_pos, dist);
            spdlog::debug("hash {}, {}", col_pos, dist);
    }

    float FLITER_WIN[3] ={1.0f, 5.0f, 20.f};
    std::vector<win_info> filter_win_info_vec;
    for (uint32_t filter_pos = 0; filter_pos < win_info_vec.size() - 2; filter_pos++) {
        float tt = win_info_vec[filter_pos].h * FLITER_WIN[0] +
                win_info_vec[filter_pos + 1].h * FLITER_WIN[1] +
                win_info_vec[filter_pos + 2].h * FLITER_WIN[2];
        filter_win_info_vec.push_back(win_info(win_info_vec[filter_pos].pos, tt));
        // qDebug("filter_win_info_vec %d, %f", win_info_vec[filter_pos].pos, tt);
        if (filter_pos >= 2) {
            float tt = filter_win_info_vec[filter_pos - 2].h + filter_win_info_vec[filter_pos - 1].h + filter_win_info_vec[filter_pos].h;
            if (tt > 145) {
                max_hash_win_x = filter_win_info_vec[filter_pos - 2].pos;
                max_hash_win_y = -max_hash_win_x / tan(theta) + rho / sin(theta);
                // qDebug("start point found! (%d, %d)", max_hash_win_x, max_hash_win_y);
                break;
            } else {
                max_hash_win_x = 0;
                max_hash_win_y = 0;
            }
        }
    }

    cv::line(sub_img, cv::Point(x1, y1),
                         cv::Point(x2,y2), cv::Scalar(0,0,0), 1, CV_AA);
    cv::rectangle(sub_img, cv::Point(max_hash_win_x, max_hash_win_y - 11),
                 cv::Point(max_hash_win_x + CONV_WIN_WIDTH * 2, max_hash_win_y + CONV_WIN_WIDTH * 2 - 11),
                 Scalar(0, 255, 255), 1, CV_AA);
    cv::rectangle(line_img, cv::Point(max_hash_win_x, max_hash_win_y - 11),
                 cv::Point(max_hash_win_x + CONV_WIN_WIDTH * 2, max_hash_win_y + CONV_WIN_WIDTH * 2 - 11),
                 Scalar(255), 1, CV_AA);
//    imshow("sub_img", sub_img);

//     const int WIN_WIDTH = win_size.width();
//     const int WIN_HEIGHT = win_size.height();
//     resize(line_bin, line_bin, Size(WIN_WIDTH,WIN_HEIGHT));
    
    qimg = QImage((const unsigned char*)(line_img.data),
                  line_img.cols,
                  line_img.rows,
                  line_img.step,
                  QImage::Format_Grayscale8).copy();

    return true;
}

QTime g_t;
int old_time = 0;
int TimeElapsed() {
    int cur_time = g_t.msec();
    int tt_time = cur_time - old_time;
    old_time = cur_time;
    return tt_time;
}

bool ImgProcess::VideoProcess(QLabel * painter) {
    string path = "d:/code/weild_img/1.mp4";
    namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
    VideoCapture capture(path);
    Mat img, binaryImage;
    Mat disp_img;
    if (!capture.isOpened()) {
         qDebug("open video failed!");
    } else {
        int frame_cnt = 0;
        g_t.start();
        old_time = g_t.msec();
        for(;;) {
            spdlog::debug("frame {}", frame_cnt++);
            TimeElapsed();
            capture >> img;
            spdlog::debug("time elapsed {}ms, capture one frame", TimeElapsed());
            if(!img.data) {
                break;
            }
            Mat sub_img(img, cv::Range(480, 960),
                         cv::Range(200, 520));  // 480 * 320
            Mat sub_gray;
            cvtColor(sub_img, sub_gray, CV_BGR2GRAY);
            spdlog::debug("time elapsed {}ms, bgr2gray", TimeElapsed());
            threshold(sub_gray, binaryImage, 200, 255, THRESH_BINARY);
            spdlog::debug("time elapsed {}ms, threshold bin", TimeElapsed());
            Mat tmp_img = Mat::zeros(binaryImage.size(), CV_8UC1);

            FilterSmallRegions(binaryImage, tmp_img);
            spdlog::debug("time elapsed {}ms, FilterSmallRegions", TimeElapsed());
            // imshow("FilterSmallRegions", tmp_img);

            NormalizeLine(tmp_img,binaryImage);
            spdlog::debug("time elapsed {}ms, NormalizeLine", TimeElapsed());
            // imshow("NormalizeLine", binaryImage);
            // qDebug("binaryImage, %d x %d, type %d", binaryImage.rows, binaryImage.cols, binaryImage.type());

            Mat line_img = Mat::zeros(binaryImage.size(), CV_8UC1);
            normalize(binaryImage, line_img, 0, 255, NORM_MINMAX, CV_8U);
            spdlog::debug("time elapsed {}ms, normalize", TimeElapsed());
            // qDebug("line_img, %d x %d, type %d", line_img.rows, line_img.cols, line_img.type());

            std::vector<cv::Vec3f> lines;
            cv::HoughLines(line_img, lines, 1, CV_PI/180, 50);
            spdlog::debug("time elapsed {}ms, HoughLines", TimeElapsed());

            float rho = 0.0f, theta = 0.0f;
            int cnt = 0;
            for (auto idx = lines.begin(); idx != lines.end(); idx++) {
                if((*idx)[1] < 1.4 || (*idx)[1] > 1.7) {
                    continue;
                }

                if ((*idx)[2] > 70) {
                    cnt++;
                    rho += (*idx)[0];
                    theta += (*idx)[1];
                }
                qDebug("rho/theta/vote %f , %f, %f", (*idx)[0], (*idx)[1], (*idx)[2]);
            }

            if(cnt > 0) {
                rho = rho / cnt;
                theta = theta / cnt;
            }

            if (theta < (0.3f * CV_PI) || theta > (0.6f * CV_PI) || rho < 50 || rho > 300) {
                continue;
            }
            spdlog::debug("final rho {}, theta {}, ang {}", rho, theta, theta * 180 / CV_PI);
            int x0 = rho * cos(theta);
            int y0 = rho * sin(theta);
            int x1 = x0 - 500 * sin(theta);
            int y1 = y0 + 500 * cos(theta);
            int x2 = x0 + 500 * sin(theta);
            int y2 = y0 - 500 * cos(theta);

            std::vector<win_info> win_info_vec;
            int row_min = rho / sin(theta);
            int row_max = -(line_img.cols - 1) / tan(theta) + rho / sin(theta);

            if (row_min < 0 || row_min >= line_img.rows) {
                break;
            }

            if (row_max < 0 || row_max >= line_img.rows) {
                break;
            }

            int tmp = min(row_min, row_max);
            row_max = max(row_min, row_max);
            row_min = tmp;
            spdlog::debug("row_min {}, row_max {}", row_min, row_max);
            spdlog::debug("time elapsed {}ms, line calculate", TimeElapsed());

            for (int col_pos = 40;  col_pos < line_img.cols - CONV_WIN_WIDTH - 100; col_pos +=CONV_WIN_STRIDE) {
                    float dist = 0.0f;
                    for (int row_pos = row_min; row_pos <= row_max; row_pos++) {
                            if (*line_img.ptr<uchar>(row_pos, col_pos) > 210) {
                                    dist += getDist_P2L(cv::Point(col_pos, row_pos),
                                    cv::Point(x1, y1),
                                    cv::Point(x2, y2));
                            }
                    }
                    win_info_vec.push_back(win_info(col_pos,dist));
                    // spdlog::debug("hash {}, {}", col_pos, dist);
            }
            spdlog::debug("time elapsed {}ms, getDist_P2L", TimeElapsed());

            float FLITER_WIN[3] ={1.0f, 5.0f, 20.f};
            std::vector<win_info> filter_win_info_vec;
            for (uint32_t filter_pos = 0; filter_pos < win_info_vec.size() - 2; filter_pos++) {
                float tt = win_info_vec[filter_pos].h * FLITER_WIN[0] +
                        win_info_vec[filter_pos + 1].h * FLITER_WIN[1] +
                        win_info_vec[filter_pos + 2].h * FLITER_WIN[2];
                filter_win_info_vec.push_back(win_info(win_info_vec[filter_pos].pos, tt));
                // qDebug("filter_win_info_vec %d, %f", win_info_vec[filter_pos].pos, tt);
                if (filter_pos >= 2) {
                    float tt = filter_win_info_vec[filter_pos - 2].h + filter_win_info_vec[filter_pos - 1].h + filter_win_info_vec[filter_pos].h;
                    if (tt > 145) {
                        max_hash_win_x = filter_win_info_vec[filter_pos - 2].pos;
                        max_hash_win_y = -max_hash_win_x / tan(theta) + rho / sin(theta);
                        // spdlog::debug("start point found! ({}, {})", max_hash_win_x, max_hash_win_y);
                        break;
                    } else {
                        max_hash_win_x = 0;
                        max_hash_win_y = 0;
                    }
                }
            }
            spdlog::debug("time elapsed {}ms, window detect", TimeElapsed());
            if (max_hash_win_x != 0) {
                cv::line(sub_img, cv::Point(x1, y1),
                                 cv::Point(x2,y2), cv::Scalar(0,0,0), 1, CV_AA);
                cv::rectangle(sub_img, cv::Point(max_hash_win_x, max_hash_win_y - 11),
                            cv::Point(max_hash_win_x + CONV_WIN_WIDTH * 2, max_hash_win_y + CONV_WIN_WIDTH * 2 - 11),
                            Scalar(0, 255, 255), 1, CV_AA);
                cv::rectangle(line_img, cv::Point(max_hash_win_x, max_hash_win_y - 11),
                            cv::Point(max_hash_win_x + CONV_WIN_WIDTH * 2, max_hash_win_y + CONV_WIN_WIDTH * 2 - 11),
                            Scalar(255), 1, CV_AA);
            }

//            sub_img.copyTo(disp_img);
//            imshow("video", disp_img);
            spdlog::debug("time elapsed {}ms, draw and show img", TimeElapsed());
            QImage img = QImage((const unsigned char*)(line_img.data),
                             line_img.cols,
                             line_img.rows,
                             line_img.step,
                             QImage::Format_Grayscale8).copy();

            painter->setPixmap(QPixmap::fromImage(img));
//            ui->X->setText(QString::number(max_hash_win_x));
//            ui->Y->setText(QString::number(max_hash_win_y));
//            ui->Z->setText(QString::number(0));

            waitKey(1);
        }
    }

//     const int WIN_WIDTH = win_size.width();
//     const int WIN_HEIGHT = win_size.height();
//     resize(line_bin, line_bin, Size(WIN_WIDTH,WIN_HEIGHT));

//    qimg = QImage((const unsigned char*)(line_img.data),
//                  line_img.cols,
//                  line_img.rows,
//                  line_img.step,
//                  QImage::Format_Grayscale8).copy();

    return true;
}
