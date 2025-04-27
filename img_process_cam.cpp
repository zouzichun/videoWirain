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

//cv::Point2f X21(-1959.74, -1716.01);
//cv::Point2f X22(-1970.84, 12.132);

//cv::Point2f X11(-1959.74, -1716.01);
//cv::Point2f X12(-1970.84, 12.132);

extern std::pair<double, double> X2_MACH;
extern std::pair<double, double> X1_MACH;
extern DataPkt data_pkt;


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

    ret = p_cam->GetImageBuffer(&frame, 1000);
    if (ret != MV_OK) {
        spdlog::debug("No data {:#x}", ret);
        p_cam->StopGrabbing();
        return;
    }

    ret = p_cam->FreeImageBuffer(&frame);
    if (ret != MV_OK) {
        spdlog::debug("free img buffer failed!");
        p_cam->StopGrabbing();
        return;
    }

    const int IMG_HEIGHT = frame.stFrameInfo.nHeight;
    const int IMG_WIDTH = frame.stFrameInfo.nWidth;
    X2_MACH.first = configData.x2_rho;
    X1_MACH.first = 1020.0 + configData.x2_rho;

    while (camera_enable) {
        {
            std::lock_guard<std::mutex> lg(p_port->mtx);
            p_port->rdy_flag = false;
            p_port->rdy_data = 0.0f;
        }
        
        // do {
        //     p_port->readModbusData(3, 700, 2);
        //     p_port->thd_msleep(100);
        // } while(!p_port->rdy_flag && p_port->rdy_data != 1.0f);
    
        ret = p_cam->GetImageBuffer(&frame, 1000);
        if (ret != MV_OK) {
            spdlog::debug("No data {:#x}", ret);
            p_cam->StopGrabbing();
            return;
        }

        cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);
        cv::Mat color_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
        cv::Mat edge_up(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat edge_down(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        std::vector<cv::Vec2f> lines_found_up;
        std::vector<cv::Vec2f> lines_found_down;

        cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);

        PreProcess(color_img, edge_up, edge_down);
        ret = p_cam->FreeImageBuffer(&frame);
        if (ret != MV_OK) {
            spdlog::debug("free img buffer failed!");
            p_cam->StopGrabbing();
            return;
        }

        Process(edge_up, lines_found_up);
        if (!FilterLines(edge_up.rows, edge_up.cols, lines_found_up, true)) {
            frame_cnt++;
        }
        Process(edge_down, lines_found_down);
        if (!FilterLines(edge_down.rows, edge_down.cols, lines_found_down, false)) {
            frame_cnt++;
        }

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
        
        frame_cnt++;
    }

    p_cam->StopGrabbing();
    return;
}

void ImgProcess::CameraCalTest(CMvCamera* p_cam) {
    int frame_cnt = 0;
    if (camera_enable) {
        std::vector<std::vector<cv::Point2i> > pgp;
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

        qDebug("inv_thd %d", configData.inv_thd);
        qDebug("canny %d %d %d", configData.canny_1, configData.canny_2, configData.canny_3);
        qDebug("houghline %d %d %d", configData.hgline_1, configData.hgline_2, configData.hgline_3);
        const float ang1 = ((90 + configData.line1_ang) % 180) * CV_PI / 180;
        const float ang2 = ((90 + configData.line2_ang) % 180) * CV_PI / 180;
        const float ang_abs = configData.line_ang_abs * 180;
        const float roh_abs = configData.line_roh_abs * 2800;
        // spdlog::debug("ang1 {}.C:{:.2f}, ang2 {}.C:{:.2f}, abs {:.2f} sel_low {} {}",
        //             configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_roh_abs,
        //             configData.line1_sel_low,
        //             configData.line2_sel_low);
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

            cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            PreProcess(color_img, edge_up, edge_down);

            ret = p_cam->FreeImageBuffer(&frame);
            if (ret != MV_OK) {
                qDebug("free img buffer failed!");
                p_cam->StopGrabbing();
                return;
            }

            bool valid_flag = true;
            Process(edge_up, lines_found_up);
            if (!FilterLines(edge_up.rows, edge_up.cols, lines_found_up, true)) {
                qDebug() << "up lines not enough";
                valid_flag = false;
            }
            Process(edge_down, lines_found_down);
            if (!FilterLines(edge_down.rows, edge_down.cols, lines_found_down, false)) {
                qDebug() << "down lines not enough";
                valid_flag = false;
            }

            if (!valid_flag) {
                frame_cnt++;
                continue;
            }

            for (const auto & v : lines_found_up) {
                float rho = v[0];
                float theta = v[1];
                auto pts = HoughToPoints(rho, theta);
                    cv::line(color_img, pts.first, pts.second, cv::Scalar(255, 255, 255), 2);
            }

            for (const auto & v : lines_found_down) {
                float rho = v[0];
                float theta = v[1];
                auto pts = HoughToPoints(rho, theta);
                    cv::line(color_img, pts.first, pts.second, cv::Scalar(255, 255, 255), 2);
            }

            cv::Mat edge;
            cv::addWeighted(edge_up, 0.5, edge_down, 0.5, 0, edge);

            emit signal_refresh_img(color_img);
            emit signal_refresh_cal_img(edge);
            frame_cnt++;
        }
    } else {
        p_cam->StopGrabbing();
        qDebug() << "video exit, frames " <<  frame_cnt;
    }
    return;
}

