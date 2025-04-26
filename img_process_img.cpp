#include "img_process.h"
#include "crcalgorithm.h"
#include "maindialog.h"
#include "imgwindow.h"
#include "ui_imgwindow.h"
#include <QDebug>
#include <QPainter>
#include <QLineEdit>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;

extern std::vector<Lines> g_lines;
cv::Point2f getCrossPoint(cv::Vec4i LineA, cv::Vec4i LineB);

cv::Point2f X21(-1959.74, -1716.01);
cv::Point2f X22(-1970.84, 12.132);

cv::Point2f X11(-1959.74, -1716.01);
cv::Point2f X12(-1970.84, 12.132);

std::pair<double, double> X2_MACH(-330.0, 0.0);
std::pair<double, double> X1_MACH(690.0, 0.0);
DataPkt data_pkt;


void ImgProcess::ImageTest(CMvCamera* p_cam, Port * p_port) {
    qDebug() << "image test";
    cv::VideoCapture cap("/home/leon/1.mp4");
//    qDebug() << "正在尝试打开:" << QString::fromStdString("/home/leon/1.mp4");
//    qDebug() << "OpenCV编译信息:" << cv::getBuildInformation().c_str();
    try {
//         cap.open(videoPath); // 明确指定使用FFmpeg后端
         if (!cap.isOpened()) {
             throw std::runtime_error("open failed!");
         }
        
        // 验证关键视频参数
        const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (width <= 0 || height <= 0) {
            throw std::runtime_error("无效的视频尺寸");
        }
        
        qDebug() << "open vide ok, size " << width << "x" << height;
    } 
    catch (const std::exception& e) {
        qCritical() << "video invalid, error:" << e.what();
        return;
    }
    int frame_cnt = 0;
    if (camera_enable) {
        frame_cnt = 0;
        // namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
        for(;;) {
            if (!camera_enable) {
                qDebug() << "video cap stoped, frames " <<  frame_cnt;
                return;
            }
            // 1. 创建视频捕获对象

            // 2. 逐帧读取并显示
            cv::Mat color_img;
            cap >> color_img;  // 读取新帧
            
            // 检查视频是否结束
            if (color_img.empty()) break;

            const int IMG_HEIGHT = color_img.rows;
            const int IMG_WIDTH = color_img.cols;
            cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));

            // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            // cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

            cv::cvtColor(color_img, color_img, COLOR_BGR2RGB);
            PreProcess(color_img, edge, contours_img);

            std::vector<cv::Vec2f> lines_found;
            Process(edge, lines_found);

            if (!FilterLines(IMG_HEIGHT, IMG_WIDTH, lines_found)) {
                frame_cnt++;
            }
            // qDebug("lines_found_up size %d,", lines_found_up.size())


        //            qDebug("g_lines.points_in_img %d, %d, %d, %d, %d, %d",
        //                    g_lines[0].points_in_img.size(),
        //                    g_lines[1].points_in_img.size(),
        //                    g_lines[2].points_in_img.size(),
        //                    g_lines[3].points_in_img.size(),
        //                    g_lines[4].points_in_img.size(),
        //                    g_lines[5].points_in_img.size());

            cv::Point2f p1, p2;
            float dist = 0.0, delta_x = 0.0, delta_x2 = 0.0;
            cv::Point2f p_mid;
            if (g_lines[0].points_in_img.size() >= 2 &&
                g_lines[1].points_in_img.size() >= 2 &&
                g_lines[2].points_in_img.size() >= 2) {
                p1 = getCrossPoint(
                    cv::Vec4i(g_lines[0].points_in_img[0].first, g_lines[0].points_in_img[0].second,
                        g_lines[0].points_in_img[1].first, g_lines[0].points_in_img[1].second),
                    cv::Vec4i(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second,
                        g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second));
                p2 = getCrossPoint(
                    cv::Vec4i(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second,
                        g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second),
                    cv::Vec4i(g_lines[2].points_in_img[0].first, g_lines[2].points_in_img[0].second,
                        g_lines[2].points_in_img[1].first, g_lines[2].points_in_img[1].second));
            }

            cv::Point2f p21, p22;
            cv::Point2f p_mid2;
            if (g_lines[3].points_in_img.size() >= 2 &&
                g_lines[4].points_in_img.size() >= 2 &&
                g_lines[5].points_in_img.size() >= 2) {
                p21 = getCrossPoint(
                    cv::Vec4i(g_lines[3].points_in_img[0].first, g_lines[3].points_in_img[0].second,
                        g_lines[3].points_in_img[1].first, g_lines[3].points_in_img[1].second),
                    cv::Vec4i(g_lines[4].points_in_img[0].first, g_lines[4].points_in_img[0].second,
                        g_lines[4].points_in_img[1].first, g_lines[4].points_in_img[1].second));
                p22 = getCrossPoint(
                    cv::Vec4i(g_lines[4].points_in_img[0].first, g_lines[4].points_in_img[0].second,
                        g_lines[4].points_in_img[1].first, g_lines[4].points_in_img[1].second),
                    cv::Vec4i(g_lines[5].points_in_img[0].first, g_lines[5].points_in_img[0].second,
                        g_lines[5].points_in_img[1].first, g_lines[5].points_in_img[1].second));
            }

            p_mid = cv::Point2f((p1.x + p2.x)/2, (p1.y + p2.y) /2);
            p_mid2 = cv::Point2f((p21.x + p22.x)/2, (p21.y + p22.y) /2);
            // spdlog::debug("  find point {}:{}", p_int.x, p_int.y);
            cv::drawMarker(color_img, p1, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p2, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p_mid, cv::Scalar(0,255,255), 3, 20, 8);

            cv::drawMarker(color_img, p21, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p22, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p_mid2, cv::Scalar(255,255,0), 3, 20, 8);

            cv::line(color_img, p1, p2, cv::Scalar(0,255,255), 4);
            cv::line(color_img, p21, p22, cv::Scalar(255,255,0), 4);
            cv::line(color_img, p_mid, p_mid2, cv::Scalar(0,255,255), 4);


            auto mach_p_up = PointsImg2Mach(p1, p2);
            auto mach_p_down = PointsImg2Mach(p21, p22);

            auto mach_hline_up = PointsToHoughParams(mach_p_up.first, mach_p_up.second);
            auto mach_hline_down = PointsToHoughParams(mach_p_down.first, mach_p_down.second);
            auto mach_start = PointsToHoughParams(cv::Point2f(0, configData.x2_start),
                cv::Point2f(315, configData.x1_start));

            auto img_points = PointsMach2Img(cv::Point2f(0, configData.x2_start), cv::Point2f(315, configData.x1_start));
            auto mach_pp = PointsImg2Mach(img_points.first, img_points.second);
            auto mach_start_pp = PointsToHoughParams(mach_pp.first, mach_pp.second);

            auto img_start = PointsMach2Img(cv::Point2f(0, configData.x2_start),
                                            cv::Point2f(315, configData.x1_start));
            cv::line(color_img, img_start.first, img_start.second,
                    cv::Scalar(255,255,255), 4);

            mach_hline_up.first = mach_hline_up.first + configData.motor_rho;
            mach_hline_down.first = mach_hline_down.first + configData.motor_rho;

            std::pair<double, double> x2_corss_up = getCrossPoint(mach_hline_up, X2_MACH);
            std::pair<double, double> x2_corss_down = getCrossPoint(mach_hline_down, X2_MACH);
            std::pair<double, double> x1_corss_up = getCrossPoint(mach_hline_up, X1_MACH);
            std::pair<double, double> x1_corss_down = getCrossPoint(mach_hline_down, X1_MACH);
            std::pair<double, double> x1_start = getCrossPoint(mach_start, X1_MACH);
            std::pair<double, double> x2_start = getCrossPoint(mach_start, X2_MACH);

            double start_delta2 = sqrtf(pow(x2_start.first - x2_corss_down.first, 2) + pow(x2_start.second - x2_corss_down.second, 2));
            double start_delta1 = sqrtf(pow(x1_start.first - x1_corss_down.first, 2) + pow(x1_start.second - x1_corss_down.second, 2));
            // qDebug("start_delta2 %.2f, start_delta1 %.2f", start_delta2, start_delta1);

            double dist_x2 = sqrtf(pow(x2_corss_up.first - x2_corss_down.first, 2) + pow(x2_corss_up.second - x2_corss_down.second, 2));
            double dist_x1 = sqrtf(pow(x1_corss_up.first - x1_corss_down.first, 2) + pow(x1_corss_up.second - x1_corss_down.second, 2));
            if (x2_corss_up.second < x2_corss_down.second) {
                dist_x2 = -dist_x2;
            }

            if (x1_corss_up.second < x1_corss_up.second) {
                dist_x1 = -dist_x1;
            }
            
            data_pkt.x2_start = x2_start.second;
            data_pkt.x2_fetch = x2_corss_down.second;
            data_pkt.x2_target = x2_corss_up.second;
            data_pkt.x1_start = x1_start.second;
            data_pkt.x1_fetch = x1_corss_down.second;
            data_pkt.x1_target = x1_corss_up.second;
            data_pkt.x1_delta = dist_x1;
            data_pkt.x2_delta = dist_x2;
            data_pkt.start_delta = start_delta2;
            data_pkt.frames = frame_cnt;
            data_pkt.valid = true;

            emit signal_refresh_img(color_img);

            emit signal_refresh_delta();

            // qDebug() << "video, frames " <<  frame_cnt;
            frame_cnt++;
            // waitKey(1);
        }
    } else {
        qDebug() << "test img exit, frames " <<  frame_cnt;
        return;
    }

    // 3. 释放资源
    cap.release();
    cv::destroyAllWindows();

    return;
}

void ImgProcess::ImageCalTest(CMvCamera* p_cam) {
    cv::VideoCapture cap("/home/leon/1.mp4");
//    qDebug() << "正在尝试打开:" << QString::fromStdString("/home/leon/1.mp4");
//    qDebug() << "OpenCV编译信息:" << cv::getBuildInformation().c_str();
    try {
//         cap.open(videoPath); // 明确指定使用FFmpeg后端
         if (!cap.isOpened()) {
             throw std::runtime_error("open failed!");
         }
        
        // 验证关键视频参数
        const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (width <= 0 || height <= 0) {
            throw std::runtime_error("无效的视频尺寸");
        }
        
        qDebug() << "成功打开视频：" << width << "x" << height;
    } 
    catch (const std::exception& e) {
        qCritical() << "初始化错误：" << e.what();
        return;
    }
    int frame_cnt = 0;
    if (camera_enable) {
        for(;;) {
            if (!camera_enable) {
                spdlog::debug("img cal exit, frames {}", frame_cnt);
                return;
            }
            
            // 2. 逐帧读取并显示
            cv::Mat color_img;
            cap >> color_img;  // 读取新帧
            
            // 检查视频是否结束
            if (color_img.empty()) break;

            const int IMG_HEIGHT = color_img.rows;
            const int IMG_WIDTH = color_img.cols;
            cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));

            PreProcess(color_img, edge, contours_img);

            std::vector<cv::Vec2f> lines_found;
            Process(edge, lines_found);
            if (!FilterLines(edge.rows, edge.cols, lines_found)) {
                frame_cnt++;
            }

            for (const auto & v : lines_found) {
                float rho = v[0];
                float theta = v[1];
                // spdlog::debug("line {} rho {:.2f}, theta {:.2f}, cnt {}", pos, rho, theta, cnts[pos]);
                    //直线与第一行的交叉点
                    cv::Point2i pt1(static_cast<int>(rho / cos(theta)), 0);
                    //直线与最后一行的交叉点
                    cv::Point2i pt2(static_cast<int>(rho / cos(theta) - IMG_HEIGHT*sin(theta) / cos(theta)), IMG_HEIGHT);
                    cv::line(color_img, pt1, pt2, cv::Scalar(255, 255, 255), 2);
                    // if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                    //     if (pt1.y > pt2.y) {
                    //         lines.push_back(cv::Vec4i(pt2.x, pt2.y, pt1.x, pt1.y));
                    //     } else {
                    //         lines.push_back(cv::Vec4i(pt1.x, pt1.y, pt2.x, pt2.y));
                    //     }
                    // }
            }

            emit signal_refresh_img(color_img);

            emit signal_refresh_cal_img(edge);

            frame_cnt++;
            // waitKey(10);
        }
    } else {
        qDebug() << "img cal exit, frames " <<  frame_cnt;
    }

    // 3. 释放资源
    cap.release();
    cv::destroyAllWindows();

    return;
}

