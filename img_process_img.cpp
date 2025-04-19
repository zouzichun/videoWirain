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

void ImgProcess::ImageTest(CMvCamera* p_cam) {
    int frame_cnt = 0;
    if (camera_enable) {
        frame_cnt = 0;
        // namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
        for(;;) {
            if (!camera_enable) {
                qDebug() << "video cap stoped, frames " <<  frame_cnt;
                return;
            }
            cv::Mat color_img = cv::imread("../test_img/1.jpg");

            const int IMG_HEIGHT = color_img.rows;
            const int IMG_WIDTH = color_img.cols;
            cv::Mat grayimg;
            cv::Mat tt_img;
            cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));

            // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            // cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

            Mat hsv;
            cv::cvtColor(color_img, hsv, COLOR_BGR2HSV);
            
            // 白色阈值范围
            cv::inRange(hsv, Scalar(0, 0, 40), Scalar(200, 60, 255), hsv);
            
            // 形态学操作
            cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
            cv::morphologyEx(hsv, hsv, MORPH_CLOSE, kernel, Point(-1,-1), 1);
            cv::morphologyEx(hsv, hsv, MORPH_OPEN, kernel, Point(-1,-1), 2);
            cv::cvtColor(hsv, tt_img, COLOR_HSV2RGB);
            cv::cvtColor(tt_img, grayimg, COLOR_RGB2GRAY);
            std::vector<cv::Vec2f> lines_found;
            Process(grayimg, edge, contours_img, lines_found);

            if (!FilterLines(edge, lines_found)) {
                frame_cnt++;
                continue;
            }

            cv::Point2f p1 = getCrossPoint(
                    cv::Vec4i(g_lines[0].points_in_img[0].first, g_lines[0].points_in_img[0].second,
                        g_lines[0].points_in_img[1].first, g_lines[0].points_in_img[1].second),
                    cv::Vec4i(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second,
                        g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second));
            cv::Point2f p2 = getCrossPoint(
                    cv::Vec4i(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second,
                        g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second),
                    cv::Vec4i(g_lines[2].points_in_img[0].first, g_lines[2].points_in_img[0].second,
                        g_lines[2].points_in_img[1].first, g_lines[2].points_in_img[1].second));

            cv::Point2f p_mid = cv::Point2f((p1.x + p2.x)/2, (p1.y + p2.y) /2);
            // spdlog::debug("  find point {}:{}", p_int.x, p_int.y);
            cv::drawMarker(color_img, p1, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p2, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p_mid, cv::Scalar(255,255,0), 3, 20, 8);

            cv::line(color_img, p1, p_mid, cv::Scalar(0,255,255), 4);
            cv::line(color_img, p_mid, p2, cv::Scalar(0,255,255), 4);

            emit signal_refresh_img(color_img);

            // qDebug() << "video, frames " <<  frame_cnt;
            frame_cnt++;
            // waitKey(1);
        }
    } else {
        qDebug() << "test img exit, frames " <<  frame_cnt;
        return;
    }

    return;
}

void ImgProcess::ImageCalTest(CMvCamera* p_cam) {
    int frame_cnt = 0;
    if (camera_enable) {
        std::vector<std::vector<cv::Point2i> > pgp;

        qDebug("inv_thd %d", configData.inv_thd);
        qDebug("canny %d %d %d", configData.canny_1, configData.canny_2, configData.canny_3);
        qDebug("houghline %d %d %d", configData.hgline_1, configData.hgline_2, configData.hgline_3);
        const float ang1 = ((90 + configData.line1_ang) % 180) * CV_PI / 180;
        const float ang2 = ((90 + configData.line2_ang) % 180) * CV_PI / 180;

        for(;;) {
            if (!camera_enable) {
                spdlog::debug("img cal exit, frames {}", frame_cnt);
                return;
            }
            
            cv::Mat color_img = cv::imread("../test_img/2.jpg");

            const int IMG_HEIGHT = color_img.rows;
            const int IMG_WIDTH = color_img.cols;
            cv::Mat grayimg;
            cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));

            // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

            // cv::Mat grayimg;
            // cv::Mat tt_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
            // cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            // cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));

            // // cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            // // cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

            // Mat hsv;
            // cv::cvtColor(color_img, hsv, COLOR_BGR2HSV);
            
            // // 白色阈值范围
            // cv::inRange(hsv, Scalar(0, 0, 40), Scalar(200, 60, 255), hsv);
            
            // // 形态学操作
            // cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
            // cv::morphologyEx(hsv, hsv, MORPH_CLOSE, kernel, Point(-1,-1), 1);
            // cv::morphologyEx(hsv, hsv, MORPH_OPEN, kernel, Point(-1,-1), 2);
            // cv::cvtColor(hsv, tt_img, COLOR_HSV2RGB);
            // cv::cvtColor(tt_img, grayimg, COLOR_RGB2GRAY);

            std::vector<cv::Vec2f> lines_found;
            Process(grayimg, edge, contours_img, lines_found);
            if (!FilterLines(edge, lines_found)) {
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

    return;
}

