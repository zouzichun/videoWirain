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
extern bool img_sw_status;

cv::Point2f X21(-1959.74, -1716.01);
cv::Point2f X22(-1970.84, 12.132);

cv::Point2f X11(-1959.74, -1716.01);
cv::Point2f X12(-1970.84, 12.132);

extern std::pair<double, double> X2_MACH;
extern std::pair<double, double> X1_MACH;
DataPkt data_pkt;

std::string vdname("/home/leon/share/3.avi");

void ImgProcess::ImageTest(CMvCamera* p_cam, Port * p_port) {
    qDebug() << "image test";
    cv::VideoCapture cap(vdname);
    try {
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
            cv::Mat edge_up(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat edge_down(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            std::vector<cv::Vec2f> lines_found_up;
            std::vector<cv::Vec2f> lines_found_down;
            std::vector<std::vector<std::pair<double, double>>> lines_filtered;

            cv::cvtColor(color_img, color_img, COLOR_BGR2RGB);
            PreProcess(color_img, edge_up, edge_down);
            bool valid_flag = true;
            // Process(edge_up, lines_found_up, true);
            // if (!AdaptLines(lines_found_up, lines_filtered)) {
            //     valid_flag = false;
            // }
            // Process(edge_down, lines_found_down, false);
            // if (!AdaptLines(lines_found_down, lines_filtered)) {
            //     valid_flag = false;
            // }
            ProcessCountor(edge_up, lines_filtered);

            std::vector<cv::Point2f> line1;
            std::vector<cv::Point2f> line2;
            if (!GetCentralLinesCountor(lines_filtered, line1, line2)) {
                frame_cnt++;
                continue;
            }

            cv::Point2f p1 = line1[0];
            cv::Point2f p_mid = line1[1];
            cv::Point2f p2 = line1[2];

            cv::Point2f p21 = line2[0];
            cv::Point2f p_mid2 = line2[1];
            cv::Point2f p22 = line2[2];

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

            mach_hline_up.first = mach_hline_up.first + configData.motor_rho;
            mach_hline_down.first = mach_hline_down.first + configData.motor_rho;
    //        qDebug("up roh %f, down rho %f", mach_hline_up.first, mach_hline_down.first);

            std::pair<double, double> x2_corss_up = getCrossPoint(mach_hline_up, X2_MACH);
            std::pair<double, double> x2_corss_down = getCrossPoint(mach_hline_down, X2_MACH);
            std::pair<double, double> x1_corss_up = getCrossPoint(mach_hline_up, X1_MACH);
            std::pair<double, double> x1_corss_down = getCrossPoint(mach_hline_down, X1_MACH);

            double zero_y = configData.y1_start - (185 - configData.x2_rho);
            double y_down = sqrtf(pow((x2_corss_down.first - (mach_p_down.first.x + mach_p_down.second.x) / 2),2) +
                                pow((x2_corss_down.second - (mach_p_down.first.y + mach_p_down.second.y) / 2),2)) + zero_y;
            double y_up = sqrtf(pow((x2_corss_up.first - (mach_p_up.first.x + mach_p_up.second.x) / 2),2) +
                                pow((x2_corss_up.second - (mach_p_up.first.y + mach_p_up.second.y) / 2),2)) + zero_y;

            data_pkt.x1_fetch = configData.x1_start + configData.motor_rho - x1_corss_down.second + configData.fetch_delta;
            data_pkt.x1_target = configData.x1_start +  configData.motor_rho - x1_corss_up.second + configData.target_delta;
            data_pkt.x2_fetch = configData.x2_start + configData.motor_rho - x2_corss_down.second + configData.fetch_delta;
            data_pkt.x2_target = configData.x2_start + configData.motor_rho - x2_corss_up.second + configData.target_delta;
            data_pkt.y1_fetch = y_down + configData.y_fetch_delta;
            data_pkt.y1_target = y_up + configData.y_target_delta;
            data_pkt.frames = frame_cnt;
            data_pkt.valid = true;

            emit signal_refresh_img(color_img);

            // emit signal_refresh_delta();

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
    cv::VideoCapture cap(vdname);
    try {
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
            cv::Mat edge_up(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat edge_down(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            std::vector<cv::Vec2f> lines_found_up;
            std::vector<cv::Vec2f> lines_found_down;
            std::vector<std::vector<std::pair<double, double>>> lines_filtered;

            PreProcess(color_img, edge_up, edge_down);
            // Process(edge_up, lines_found_up, true);
            // if (!FilterLines(edge_up.rows, edge_up.cols, lines_found_up, true)) {
            //     frame_cnt++;
            // }
            // Process(edge_down, lines_found_down, false);
            // if (!FilterLines(edge_down.rows, edge_down.cols, lines_found_down, false)) {
            //     frame_cnt++;
            // }

            ProcessCountor(edge_up, lines_filtered);
            
//            cv::Mat edge;
//            cv::addWeighted(edge_up, 0.5, edge_down, 0.5, 0, edge);

            emit signal_refresh_img(color_img);

            if (img_sw_status) {
                emit signal_refresh_cal_img(edge_up);
            } else {
                emit signal_refresh_cal_img(color_img);
            }

            data_pkt.frames = frame_cnt;
            data_pkt.valid = true;

            emit signal_refresh_delta();

            frame_cnt++;
//            waitKey(30);
        }
    } else {
        qDebug() << "img cal exit, frames " <<  frame_cnt;
    }

    // 3. 释放资源
    cap.release();
//    cv::destroyAllWindows();

    return;
}

