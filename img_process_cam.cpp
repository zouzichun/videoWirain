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

extern std::pair<double, double> X2_MACH;
extern std::pair<double, double> X1_MACH;
extern DataPkt data_pkt;
extern bool trigger_process;
extern bool img_sw_status;

volatile bool run_sync = false;

void syncDelay(int milliseconds) {
    QEventLoop loop;
    QTimer timer;

    // 绑定定时器超时信号与事件循环退出
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);

    timer.start(milliseconds);  // 启动定时器
    loop.exec();                 // 进入事件循环等待
}

void ImgProcess::CameraTest(CMvCamera* p_cam, Port * p_port) {
    int frame_cnt = 0;
    int ret = p_cam->StartGrabbing();
    if (ret) {
        spdlog::error("Start Grabing failed!");
        return;
    }
    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE_EX stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    ret = p_cam->GetIntValue("PayloadSize", &stParam);
    if (MV_OK != ret) {
        spdlog::error("Get PayloadSize fail! {}", ret);
        p_cam->StopGrabbing();
        return;
    } else {
        spdlog::debug("Get PayloadSize {}", stParam.nCurValue);
    }
    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(MV_FRAME_OUT));

    while (camera_enable) {
        if (trigger_status) {
            trigger_status = false;
            spdlog::info("manual trigger");
        } else if (auto_run_status) {
            if (run_sync) {
                syncDelay(configData.modbusDelay);
                continue;
            }
            if (p_port->rdy_flag) {
                p_port->rdy_flag = false;
                if (p_port->rdy_data != 0.0f) {
                    p_port->rdy_data = 0.0f;
                    spdlog::info("get data ready, val {:.2f}", p_port->rdy_data);
                    run_sync = true;
                }
            }
        }

        ret = p_cam->GetImageBuffer(&frame, 1000);
        if (ret != MV_OK) {
            spdlog::debug("No data {:#x}", ret);
            p_cam->StopGrabbing();
            return;
        }

        const int IMG_HEIGHT = frame.stFrameInfo.nHeight;
        const int IMG_WIDTH = frame.stFrameInfo.nWidth;

        cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);
        cv::Mat color_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);

        cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);

         ret = p_cam->FreeImageBuffer(&frame);
         if (ret != MV_OK) {
             spdlog::debug("free img buffer failed!");
             p_cam->StopGrabbing();
             return;
         }

        cv::Mat edge_up(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat edge_down(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        std::vector<cv::Vec2f> lines_found_up;
        std::vector<cv::Vec2f> lines_found_down;
        std::vector<std::vector<std::pair<double, double>>> lines_filtered;

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
        // ProcessCountor(edge_down, lines_filtered);

        if (!valid_flag) {
            frame_cnt++;
            continue;
        }

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

        auto vetical_line_up_left = calcNormalLineParams(mach_hline_up.first, mach_hline_up.second, mach_p_up.first.x, mach_p_up.first.y);
        auto vetical_line_up_right = calcNormalLineParams(mach_hline_up.first, mach_hline_up.second, mach_p_up.second.x, mach_p_up.second.y);
        auto vetical_line_down_left = calcNormalLineParams(mach_hline_down.first, mach_hline_down.second, mach_p_down.first.x, mach_p_down.first.y);
        auto vetical_line_down_right = calcNormalLineParams(mach_hline_down.first, mach_hline_down.second, mach_p_down.second.x, mach_p_down.second.y);

        mach_hline_up.first = mach_hline_up.first + configData.motor_rho;
        mach_hline_down.first = mach_hline_down.first + configData.motor_rho;
//        qDebug("up roh %f, down rho %f", mach_hline_up.first, mach_hline_down.first);

        auto mach_motor_up_left = getCrossPoint(vetical_line_up_left, mach_hline_up);
        auto mach_motor_up_right = getCrossPoint(vetical_line_up_right, mach_hline_up);
        auto mach_motor_down_left = getCrossPoint(vetical_line_down_left, mach_hline_down);
        auto mach_motor_down_right = getCrossPoint(vetical_line_down_right, mach_hline_down);

        std::pair<double, double> x2_corss_up = getCrossPoint(mach_hline_up, X2_MACH);
        std::pair<double, double> x2_corss_down = getCrossPoint(mach_hline_down, X2_MACH);
        std::pair<double, double> x1_corss_up = getCrossPoint(mach_hline_up, X1_MACH);
        std::pair<double, double> x1_corss_down = getCrossPoint(mach_hline_down, X1_MACH);

        double zero_y = configData.y1_start - (185 - configData.x2_rho);
        double y_down = sqrtf(pow((x2_corss_down.first - (mach_motor_down_left.first + mach_motor_down_right.first) / 2),2) +
                              pow((x2_corss_down.second - (mach_motor_down_left.second + mach_motor_down_right.second) / 2),2)) + zero_y;
        double y_up = sqrtf(pow((x2_corss_up.first - (mach_motor_up_left.first + mach_motor_up_right.first) / 2),2) +
                              pow((x2_corss_up.second - (mach_motor_up_left.second + mach_motor_up_right.second) / 2),2)) + zero_y;

        data_pkt.x1_fetch = configData.x1_start + configData.motor_rho - x1_corss_down.second + configData.fetch_delta;
        data_pkt.x1_target = configData.x1_start +  configData.motor_rho - x1_corss_up.second + configData.target_delta;
        data_pkt.x2_fetch = configData.x2_start + configData.motor_rho - x2_corss_down.second + configData.fetch_delta;
        data_pkt.x2_target = configData.x2_start + configData.motor_rho - x2_corss_up.second + configData.target_delta;
        data_pkt.y1_fetch = y_down + configData.y_fetch_delta;
        data_pkt.y1_target = y_up + configData.y_target_delta;
        data_pkt.frames = frame_cnt;
        data_pkt.valid = true;

        emit signal_refresh_img(color_img);

        if (trigger_status) {
            trigger_status = false;
            emit signal_refresh_delta();
        }

        if (auto_run_status) {
            if (run_sync) {
                emit signal_refresh_delta();
            }
        }
        
        frame_cnt++;
    }

    p_cam->StopGrabbing();
    return;
}

void ImgProcess::CameraCalTest(CMvCamera* p_cam) {
    int frame_cnt = 0;
    int ret = p_cam->StartGrabbing();
    if (ret) {
        qDebug("set StartGrabbing mode failed!");
        return;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE_EX stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    ret = p_cam->GetIntValue("PayloadSize", &stParam);
    if (MV_OK != ret)
    {
        qDebug("Get PayloadSize fail! [%d]\n", ret);
        p_cam->StopGrabbing();
        return;
    } else {
        qDebug("Get PayloadSize %d", stParam.nCurValue);
    }
    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(MV_FRAME_OUT));

    std::vector<std::vector<cv::Point2i> > pgp;
    qDebug("inv_thd %d", configData.inv_thd);
    qDebug("canny %d %d %d", configData.canny_1, configData.canny_2, configData.canny_3);
    qDebug("houghline %d %d %d %d", configData.hgline_1, configData.hgline_2, configData.hgline_3, configData.hgline_4);

    for(;;) {
        if (!camera_enable) {
            p_cam->StopGrabbing();
            spdlog::debug("video exit, frames {}", frame_cnt);
            return;
        }

        ret = p_cam->GetImageBuffer(&frame, 1000);
        if (ret != MV_OK)
        {
            qDebug("No data[%x]\n", ret);
            p_cam->StopGrabbing();
            return;
        }

        const int IMG_HEIGHT = frame.stFrameInfo.nHeight;
        const int IMG_WIDTH = frame.stFrameInfo.nWidth;
        cv::Mat grayimg, color_img;
        cv::Mat edge_up(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat edge_down(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);
        std::vector<cv::Vec2f> lines_found_up;
        std::vector<cv::Vec2f> lines_found_down;
        std::vector<std::vector<std::pair<double, double>>> lines_filtered;

        cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);

        ret = p_cam->FreeImageBuffer(&frame);
        if (ret != MV_OK) {
            qDebug("free img buffer failed!");
            p_cam->StopGrabbing();
            return;
        }

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
//        ProcessCountor(edge_down, lines_filtered);


        // qDebug("found lines num up %d, down %d", lines_found_up.size(), lines_found_down.size());

        cv::Mat edge;
        cv::addWeighted(edge_up, 0.5, edge_down, 0.5, 0, edge);

        emit signal_refresh_img(color_img);

        if (img_sw_status)
            emit signal_refresh_cal_img(edge);
        else
            emit signal_refresh_cal_img(color_img);

        if (!valid_flag) {
            frame_cnt++;
            continue;
        }

        frame_cnt++;
    }

    p_cam->StopGrabbing();
    qDebug() << "video exit, frames " <<  frame_cnt;
    return;
}

