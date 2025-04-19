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

cv::Point2f X21(-1959.74, -1716.01);
cv::Point2f X22(-1970.84, 12.132);

cv::Point2f X11(-1959.74, -1716.01);
cv::Point2f X12(-1970.84, 12.132);

void ImgProcess::CameraTest(CMvCamera* p_cam) {
    int frame_cnt = 0;
    if (camera_enable) {
        frame_cnt = 0;
        // namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
        int ret = p_cam->StartGrabbing();
        if (ret) {
            spdlog::error("sstart grabbing failed!");
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
        for (auto & v : g_lines) {
            float rho = 0.0, ang = 0.0;
            rho = v.rho;
            ang = v.angle;
            // cv::Point2i pt1(static_cast<int>(rho / cos(ang)), 0);
            // cv::Point2i pt2(static_cast<int>(rho / cos(ang) - IMG_HEIGHT * sin(ang) / cos(ang)), IMG_HEIGHT);
            // if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
            //     if (pt1.y > pt2.y) {
            //         v.golde_points.push_back(std::pair<int, int>(pt2.x, pt2.y));
            //         v.golde_points.push_back(std::pair<int, int>(pt1.x, pt1.y));
            //     } else {
            //         v.golde_points.push_back(std::pair<int, int>(pt1.x, pt1.y));
            //         v.golde_points.push_back(std::pair<int, int>(pt2.x, pt2.y));
            //     }
            // }

            auto points = HoughToPointsInImg(rho, ang, IMG_WIDTH, IMG_HEIGHT);
            v.golde_points.push_back(std::pair<double, double>(points.first.x, points.first.y));
            v.golde_points.push_back(std::pair<double, double>(points.second.x, points.second.y));
        }

        cv::Point2f golden_p11, golden_p12;
        if (g_lines[0].golde_points.size() >= 2 &&
            g_lines[1].golde_points.size() >= 2 &&
            g_lines[2].golde_points.size() >= 2) {
            golden_p11 = getCrossPoint(
                cv::Vec4i(g_lines[0].golde_points[0].first, g_lines[0].golde_points[0].second,
                    g_lines[0].golde_points[1].first, g_lines[0].golde_points[1].second),
                cv::Vec4i(g_lines[1].golde_points[0].first, g_lines[1].golde_points[0].second,
                    g_lines[1].golde_points[1].first, g_lines[1].golde_points[1].second));
            golden_p12 = getCrossPoint(
                cv::Vec4i(g_lines[1].golde_points[0].first, g_lines[1].golde_points[0].second,
                    g_lines[1].golde_points[1].first, g_lines[1].golde_points[1].second),
                cv::Vec4i(g_lines[2].golde_points[0].first, g_lines[2].golde_points[0].second,
                    g_lines[2].golde_points[1].first, g_lines[2].golde_points[1].second));
        }

        cv::Point2f golden_p21, golden_p22;
        if (g_lines[3].golde_points.size() >= 2 &&
            g_lines[4].golde_points.size() >= 2 &&
            g_lines[5].golde_points.size() >= 2) {
            golden_p21 = getCrossPoint(
                cv::Vec4i(g_lines[3].golde_points[0].first, g_lines[3].golde_points[0].second,
                    g_lines[3].golde_points[1].first, g_lines[3].golde_points[1].second),
                cv::Vec4i(g_lines[4].golde_points[0].first, g_lines[4].golde_points[0].second,
                    g_lines[4].golde_points[1].first, g_lines[4].golde_points[1].second));
            golden_p22 = getCrossPoint(
                cv::Vec4i(g_lines[4].golde_points[0].first, g_lines[4].golde_points[0].second,
                    g_lines[4].golde_points[1].first, g_lines[4].golde_points[1].second),
                cv::Vec4i(g_lines[5].golde_points[0].first, g_lines[5].golde_points[0].second,
                    g_lines[5].golde_points[1].first, g_lines[5].golde_points[1].second));
        }

//        auto get_cross = [&](std::vector<std::pair<cv::Point2f, cv::Point2f>> ) -> {

//        };

        for(;;) {
            if (!camera_enable) {
                p_cam->StopGrabbing();
                qDebug() << "video cap stoped, frames " <<  frame_cnt;
                return;
            }

            ret = p_cam->GetImageBuffer(&frame, 1000);
            if (ret != MV_OK) {
                spdlog::debug("No data {:#x}", ret);
                p_cam->StopGrabbing();
                return;
            }
             cv::Mat grayimg, color_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
             cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
             cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
             cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

             cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
             cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);
             std::vector<cv::Vec2f> lines_found;

            ret = p_cam->FreeImageBuffer(&frame);
            if (ret != MV_OK) {
                spdlog::debug("free img buffer failed!");
                p_cam->StopGrabbing();
                return;
            }

           Process(grayimg, edge, contours_img, lines_found);

           if (!FilterLines(edge, lines_found)) {
               frame_cnt++;
               continue;
           }

            qDebug("g_lines.points_in_img %d, %d, %d, %d, %d, %d",
                    g_lines[0].points_in_img.size(),
                    g_lines[1].points_in_img.size(),
                    g_lines[2].points_in_img.size(),
                    g_lines[3].points_in_img.size(),
                    g_lines[4].points_in_img.size(),
                    g_lines[5].points_in_img.size());

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
                
                // dist  = getDist_P2L(p_mid, cv::Point2f(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second),
                //     cv::Point2f(g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second));
            
                // if (cos(g_lines[1].lines_filterd[0].second) != 0.0 && sin(g_lines[1].lines_filterd[0].second) != 0.0) {
                //     delta_x = dist / cos(g_lines[1].lines_filterd[0].second);
                //     delta_x2 = dist / sin(g_lines[1].lines_filterd[0].second);
                // }
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
                
                // dist  = getDist_P2L(p_mid, cv::Point2f(g_lines[1].points_in_img[0].first, g_lines[1].points_in_img[0].second),
                //     cv::Point2f(g_lines[1].points_in_img[1].first, g_lines[1].points_in_img[1].second));
            
                // if (cos(g_lines[1].lines_filterd[0].second) != 0.0 && sin(g_lines[1].lines_filterd[0].second) != 0.0) {
                //     delta_x = dist / cos(g_lines[1].lines_filterd[0].second);
                //     delta_x2 = dist / sin(g_lines[1].lines_filterd[0].second);
                // }
            }

            p_mid = cv::Point2f((p1.x + p2.x)/2, (p1.y + p2.y) /2);
            p_mid2 = cv::Point2f((p21.x + p22.x)/2, (p21.y + p22.y) /2);
            // spdlog::debug("  find point {}:{}", p_int.x, p_int.y);
            cv::drawMarker(color_img, p1, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p2, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p_mid, cv::Scalar(255,255,0), 3, 20, 8);

            cv::drawMarker(color_img, p21, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p22, cv::Scalar(0,255,0), 3, 20, 8);
            cv::drawMarker(color_img, p_mid2, cv::Scalar(255,255,0), 3, 20, 8);

            cv::line(color_img, p1, p2, cv::Scalar(0,255,255), 4);
            cv::line(color_img, p21, p22, cv::Scalar(0,255,255), 4);

            // cv::Point2f p_mid_golden = cv::Point2f((golden_p1.x + golden_p2.x)/2, (golden_p1.y + golden_p2.y) /2);
            // cv::drawMarker(color_img, golden_p1, cv::Scalar(255,0,0), 3, 20, 8);
            // cv::drawMarker(color_img, golden_p2, cv::Scalar(255,0,0), 3, 20, 8);
            // cv::drawMarker(color_img, p_mid_golden, cv::Scalar(255,0,0), 3, 20, 8);
//            qDebug("abs edge length %f pix, %f pix per ms",
//               sqrtf(pow(golden_p1.x - golden_p2.x, 2) + pow(golden_p1.y - golden_p2.y, 2)),
//                   srtf(pow(golden_p1.x - golden_p2.x, 2) + pow(golden_p1.y - golden_p2.y, 2))/ 198);

            // qDebug("abs edge length %f pix, %f pix per ms",
            //     sqrtf(pow(golden_p1.x - golden_p2.x, 2) + pow(golden_p1.y - golden_p2.y, 2)),
            //         sqrtf(pow(golden_p1.x - golden_p2.x, 2) + pow(golden_p1.y - golden_p2.y, 2))/ 198);
            
            // qDebug("delta x %.2f, delta x2 %.2f", delta_x, delta_x2 / 5.98);
            cv::line(color_img, p_mid, p_mid2, cv::Scalar(0,255,255), 4);

            // float golden_ang = std::atan((golden_p2.y - golden_p1.y) /  (golden_p2.x - golden_p1.x));
            // spdlog::debug("golden p {},{},  {},{}, ang {}", golden_p1.x, golden_p1.y, golden_p2.x, golden_p2.y, golden_ang);
            // cv::line(color_img, cv::Point2i(configData.point1_x, configData.point1_y),
            //            cv::Point2i(int(configData.camera_abs_x), int(configData.camera_abs_y)), Scalar(0, 255, 0), 5, CV_AA);
            // cv::line(color_img, cv::Point2i(int(configData.camera_abs_x), int(configData.camera_abs_y)),
            //            cv::Point2i(configData.point2_x, configData.point2_y), Scalar(0, 255, 0), 5, CV_AA);

            emit signal_refresh_img(color_img);

//            emit signal_refresh_delta(0.0f, 1.1f, golden_ang - g_lines[1].lines_filterd[0].second, p_mid_golden.x - p_mid.x, p_mid_golden.y - p_mid.y);

            // qDebug() << "video, frames " <<  frame_cnt;
            frame_cnt++;
//            waitKey(10);
        }
    } else {
        p_cam->StopGrabbing();
        qDebug() << "video exit, frames " <<  frame_cnt;
        return;
    }

    return;
}

bool ImgProcess::CameraDemo(bool & enable, QImage & qimg, QLineEdit * x, QLineEdit * y, float * delta, float * delta_ang, CMvCamera* p_cam) {
    // namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
    int ret = p_cam->StartGrabbing();
    if (ret) {
        spdlog::error("Start Grabing failed!");
        return false;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE_EX stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    ret = p_cam->GetIntValue("PayloadSize", &stParam);
    if (MV_OK != ret)
    {
        spdlog::error("Get PayloadSize fail! {}", ret);
        p_cam->StopGrabbing();
        return false;
    } else {
        spdlog::debug("Get PayloadSize {}", stParam.nCurValue);
    }
    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(MV_FRAME_OUT));
    int frame_cnt = 0;

    for(;;) {
        if (!enable) {
            p_cam->StopGrabbing();
            spdlog::debug("video exit");
            return false;
        }

        ret = p_cam->GetImageBuffer(&frame, 1000);
        if (ret != MV_OK)
        {
            spdlog::debug("No data {}", ret);
            p_cam->StopGrabbing();
            return false;
        }

         const int IMG_HEIGHT = frame.stFrameInfo.nHeight;
         const int IMG_WIDTH = frame.stFrameInfo.nWidth;
         cv::Mat grayimg, color_img;
         cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
         cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
         cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

         cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
         cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);
         std::vector<cv::Vec2f> lines_found;
         Process(grayimg, edge, contours_img, lines_found);
         
         std::vector<float> rhos(2);
         std::vector<float> thetas(2);
         std::vector<cv::Point2i> points;

         if (!AdaptLines(edge, lines_found, rhos, thetas)) {
             spdlog::debug("adapt lines failed!");
             // p_cam->StopGrabbing();
             continue;
         }

         ret = p_cam->FreeImageBuffer(&frame);
         if (ret != MV_OK) {
             spdlog::debug("free img buffer failed!");
             p_cam->StopGrabbing();
             return false;
         }

         std::vector<cv::Vec4i> lines;
         for (int pos = 0; pos < 2; pos++)
         {
             float rho = rhos[pos];
             float theta = thetas[pos];
             // qDebug("line %d rho %f, theta %f, cnt %d", pos, rho, theta, cnts[pos]);
                 //直线与第一行的交叉点
                 cv::Point2i pt1(static_cast<int>(rho / cos(theta)), 0);
                 //直线与最后一行的交叉点
                 cv::Point2i pt2(static_cast<int>(rho / cos(theta) - IMG_HEIGHT*sin(theta) / cos(theta)), IMG_HEIGHT);
                 cv::line(color_img, pt1, pt2, cv::Scalar(0,0,255), 2);
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

         // spdlog::debug("  find point {}:{}", p_int.x, p_int.y);
         cv::drawMarker(color_img, p_int, cv::Scalar(0,255,0), 3, 20, 8);
         cv::Vec4i base_line1(configData.camera_abs_x, configData.camera_abs_y, configData.point1_x, configData.point1_y);
         cv::Vec4i base_line2(configData.camera_abs_x, configData.camera_abs_y, configData.point2_x, configData.point2_y);
         
         // int tt = (configData.point1_x - configData.point2_x) / (configData.point1_y - configData.point2_y) * (p.y - configData.point2_y) + configData.point2_x;
         cv::Point2f tgt_p = getCrossPoint(base_line1, base_line2);
        //  if (tt <= p.x) {
        //      tgt_p = getCrossPoint(lines[0], base_line1);
        //  } else {
        //      tgt_p = getCrossPoint(lines[1], base_line2);
        //  }
         // spdlog::debug("  target point {:.2f}:{:.2f}", tgt_p.x, tgt_p.y);

         cv::drawMarker(color_img, tgt_p, cv::Scalar(0,255,255), 3, 20, 8);
         cv::line(color_img, tgt_p, p_int, cv::Scalar(0,0,255), 5);

         *delta = sqrt(pow(tgt_p.x - p.x, 2) + pow(tgt_p.y - p.y, 2));
         *delta_ang = ((thetas[0] - ((180 - configData.line1_ang) % 180) * CV_PI / 180) + (thetas[1] - ((180 - configData.line2_ang) % 180) * CV_PI / 180))/2;
         x->setText(QString(QString::number(p.x, 'f', 1)));
         y->setText(QString(QString::number(p.y, 'f', 1)));

         cv::line(color_img, cv::Point2i(configData.point1_x, configData.point1_y),
                     cv::Point2i(int(configData.camera_abs_x), int(configData.camera_abs_y)), Scalar(0, 255, 0), 5, CV_AA);
         cv::line(color_img, cv::Point2i(int(configData.camera_abs_x), int(configData.camera_abs_y)),
                     cv::Point2i(configData.point2_x, configData.point2_y), Scalar(0, 255, 0), 5, CV_AA);

        //  qimg = QImage((const unsigned char*)(color_img.data),
        //                      color_img.cols,
        //                      color_img.rows,
        //                      color_img.step,
        //                      // QImage::Format_Grayscale8).copy();
        //                      QImage::Format_RGB888).copy();
        frame_cnt++;
    }

    p_cam->StopGrabbing();

    return true;
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
            cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
            cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

            cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
            cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

            ret = p_cam->FreeImageBuffer(&frame);
            if (ret != MV_OK) {
                qDebug("free img buffer failed!");
                p_cam->StopGrabbing();
                return;
            }

            std::vector<cv::Vec2f> lines_found;
            Process(grayimg, edge, contours_img, lines_found);
//            std::vector<Lines> lines_out;
            if (!FilterLines(edge, lines_found)) {
                // p_cam->StopGrabbing();
                // return false;
                // qDebug("FilterLines failed!");
                frame_cnt++;
                // waitKey(1);
                // continue;
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

            // QImage qimg = QImage((const unsigned char*)(edge.data),
            //                                 edge.cols,
            //                                 edge.rows,
            //                                 edge.step,
            //                                 // QImage::Format_BGR888).copy();
            //                                 QImage::Format_Grayscale8).copy();
            // pt3->setPixmap(QPixmap::fromImage(qimg).scaled(pt->size(), Qt::KeepAspectRatio));

            emit signal_refresh_img(color_img);

            emit signal_refresh_cal_img(edge);

            // pgp.push_back(points);
            // // spdlog::debug("top {},{}, p {}:{}, bottom {},{}", points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
            // cv::line(color_img, points[0], points[1], cv::Scalar(0,255,255), 2, CV_AA);
            // cv::line(color_img, points[1], points[2], cv::Scalar(0,255,255), 2, CV_AA);
            // x->setText(QString(QString::number(points[1].x)));
            // y->setText(QString(QString::number(points[1].y)));
            // cv::drawMarker(color_img, points[1], cv::Scalar(255,255,255), 3, 20, 4);

            frame_cnt++;
            // waitKey(10);
        }
    } else {
        p_cam->StopGrabbing();
        qDebug() << "video exit, frames " <<  frame_cnt;
    }

    // std::vector<cv::Point2i> linesp = {cv::Point2i(0,0),cv::Point2i(0,0),cv::Point2i(0,0)};
    // const int SZ = pgp.size() == 0? 1 : pgp.size();
    // auto f = [&] () ->int {
    //     for (const auto & v: pgp) {
    //         linesp[0].x = linesp[0].x + v[0].x;
    //         linesp[0].y = linesp[0].y + v[0].y;
    //         linesp[1].x = linesp[1].x + v[1].x;
    //         linesp[1].y = linesp[1].y + v[1].y;
    //         linesp[2].x = linesp[2].x + v[2].x;
    //         linesp[2].y = linesp[2].y + v[2].y;
    //     }

    //     for (auto & v: linesp) {
    //         v.x = v.x / SZ;
    //         v.y = v.y / SZ;
    //     }
    // };

    // f();

    // qDebug("cal x:y to %d:%d", linesp[1].x, linesp[1].y);
    // configData.camera_abs_x = linesp[1].x;
    // configData.camera_abs_y = linesp[1].y;

    // configData.point1_x = linesp[0].x;
    // configData.point1_y = linesp[0].y;
    // configData.point2_x = linesp[2].x;
    // configData.point2_y = linesp[2].y;

    return;
}


bool ImgProcess::CameraCal(QLabel * pt, QLabel * pt3, QLineEdit * x, QLineEdit * y, CMvCamera* p_cam) {
    int frame_cnt = 0;
    int ret = -1;
    cv::Mat img;
//    cv::namedWindow("video", WINDOW_NORMAL);
    std::vector<int> x_sta;
    std::vector<int> y_sta;
    std::vector<std::vector<cv::Point2i> > pgp;

    ret = p_cam->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
    if (ret) {
        qDebug("set trigger mode failed!");
        return false;
    }
    ret = p_cam->StartGrabbing();
    if (ret) {
        qDebug("set trigger mode failed!");
        return false;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE_EX stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE_EX));
    ret = p_cam->GetIntValue("PayloadSize", &stParam);
    if (MV_OK != ret)
    {
        qDebug("Get PayloadSize fail! [%d]\n", ret);
        p_cam->StopGrabbing();
        return false;
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
    spdlog::debug("ang1 {}.C:{:.2f}, ang2 {}.C:{:.2f}, abs {:.2f} sel_low {} {}",
                  configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_roh_abs,
                  configData.line1_sel_low,
                  configData.line2_sel_low);
    for(;;) {
        if(frame_cnt > 100) {
            break;
        } else {
            // qDebug("frame pos %d", frame_cnt);
        }
        ret = p_cam->GetImageBuffer(&frame, 1000);
        if (ret != MV_OK)
        {
            qDebug("No data[%x]\n", ret);
            p_cam->StopGrabbing();
            return false;
        }

        const int IMG_HEIGHT = frame.stFrameInfo.nHeight;
        const int IMG_WIDTH = frame.stFrameInfo.nWidth;
        cv::Mat grayimg, color_img;
        cv::Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        cv::Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

        cv::cvtColor(img, color_img, COLOR_BayerBG2RGB);
        cv::cvtColor(color_img, grayimg, COLOR_RGB2GRAY);

        std::vector<cv::Vec2f> lines_found;
        Process(grayimg, edge, contours_img, lines_found);
        std::vector<float> rhos = {0, 0};
        std::vector<float> thetas = {0, 0};
        std::vector<cv::Point2i> points;

        if (!FilterLines(edge, lines_found, rhos, thetas, points)) {
            p_cam->FreeImageBuffer(&frame);
            // p_cam->StopGrabbing();
            // return false;
            frame_cnt++;
            waitKey(1);
            continue;
        }

        QImage qimg = QImage((const unsigned char*)(edge.data),
                                        edge.cols,
                                        edge.rows,
                                        edge.step,
                                        // QImage::Format_BGR888).copy();
                                        QImage::Format_Grayscale8).copy();
        pt3->setPixmap(QPixmap::fromImage(qimg).scaled(pt->size(), Qt::KeepAspectRatio));

        ret = p_cam->FreeImageBuffer(&frame);
        if (ret != MV_OK) {
            qDebug("free img buffer failed!");
            p_cam->StopGrabbing();
            return false;
        }

        pgp.push_back(points);
        // spdlog::debug("top {},{}, p {}:{}, bottom {},{}", points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y);
        cv::line(color_img, points[0], points[1], cv::Scalar(0,255,255), 2, CV_AA);
        cv::line(color_img, points[1], points[2], cv::Scalar(0,255,255), 2, CV_AA);
        x->setText(QString(QString::number(points[1].x)));
        y->setText(QString(QString::number(points[1].y)));
        cv::drawMarker(color_img, points[1], cv::Scalar(255,255,255), 3, 20, 4);

        QImage oimg = QImage((const unsigned char*)(color_img.data),
                            color_img.cols,
                            color_img.rows,
                            color_img.step,
                            // QImage::Format_Grayscale8).copy();
                            QImage::Format_RGB888).copy();

        pt->setPixmap(QPixmap::fromImage(oimg).scaled(pt->size(), Qt::KeepAspectRatio));
        frame_cnt++;
        waitKey(1);
    }

    //delete imgw;
    p_cam->StopGrabbing();

    std::vector<cv::Point2i> linesp = {cv::Point2i(0,0),cv::Point2i(0,0),cv::Point2i(0,0)};
    const int SZ = pgp.size() == 0? 1 : pgp.size();
    auto f = [&] () ->int {
        for (const auto & v: pgp) {
            linesp[0].x = linesp[0].x + v[0].x;
            linesp[0].y = linesp[0].y + v[0].y;
            linesp[1].x = linesp[1].x + v[1].x;
            linesp[1].y = linesp[1].y + v[1].y;
            linesp[2].x = linesp[2].x + v[2].x;
            linesp[2].y = linesp[2].y + v[2].y;
        }

        for (auto & v: linesp) {
            v.x = v.x / SZ;
            v.y = v.y / SZ;
        }
    };

    f();

    qDebug("cal x:y to %d:%d", linesp[1].x, linesp[1].y);
    configData.camera_abs_x = linesp[1].x;
    configData.camera_abs_y = linesp[1].y;

    configData.point1_x = linesp[0].x;
    configData.point1_y = linesp[0].y;
    configData.point2_x = linesp[2].x;
    configData.point2_y = linesp[2].y;

    return true;
}


