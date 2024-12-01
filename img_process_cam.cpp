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

using namespace std;
using namespace cv;

Point2f getCrossPoint(Vec4i LineA, Vec4i LineB);
extern std::mutex disp_lock;

bool ImgProcess::CameraDemo(bool & enable, QImage & qimg, QLineEdit * x, QLineEdit * y, float * delta, float * delta_ang, CMvCamera* p_cam) {
    // namedWindow("video", WINDOW_KEEPRATIO | WINDOW_NORMAL);
    int ret = p_cam->StartGrabbing();
    if (ret) {
        spdlog::error("set trigger mode failed!");
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
         Mat grayimg, color_img;
         Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
         Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
         Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

         img.copyTo(grayimg);
         cv::cvtColor(img, color_img, COLOR_GRAY2BGR);

         Process(grayimg, edge, contours_img);

         std::vector<cv::Vec2f> lines_found;
         std::vector<float> rhos(2);
         std::vector<float> thetas(2);
         std::vector<Point2i> points;

         if (!AdaptLines(contours_img, lines_found, rhos, thetas)) {
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
                 Point2i pt1(static_cast<int>(rho / cos(theta)), 0);
                 //直线与最后一行的交叉点
                 Point2i pt2(static_cast<int>(rho / cos(theta) - IMG_HEIGHT*sin(theta) / cos(theta)), IMG_HEIGHT);
                 cv::line(color_img, pt1, pt2, cv::Scalar(0,0,255), 2);
                 if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                     if (pt1.y > pt2.y) {
                         lines.push_back(Vec4i(pt2.x, pt2.y, pt1.x, pt1.y));
                     } else {
                         lines.push_back(Vec4i(pt1.x, pt1.y, pt2.x, pt2.y));
                     }
                 }
         }

         double ka, kb;
         ka = (double)(lines[0][3] - lines[0][1]) / (double)(lines[0][2] - lines[0][0]); //求出LineA斜率
         kb = (double)(lines[1][3] - lines[1][1]) / (double)(lines[1][2] - lines[1][0]); //求出LineB斜率

         Point2f p;
         p.x = (ka*lines[0][0] - lines[0][1] - kb*lines[1][0] + lines[1][1]) / (ka - kb);
         p.y = (ka*kb*(lines[0][0] - lines[1][0]) + ka*lines[1][1] - kb*lines[0][1]) / (ka - kb);
         // qDebug("ka %f, kb %f, p (%f, %f)", ka, kb, p.x, p.y);

         Point2i p_int(int(p.x), int(p.y));
         Point2i line1p, line2p;
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
         Point2f tgt_p = getCrossPoint(base_line1, base_line2);
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

         std::unique_lock<std::mutex> lg(disp_lock);
         qimg = QImage((const unsigned char*)(color_img.data),
                             color_img.cols,
                             color_img.rows,
                             color_img.step,
                             // QImage::Format_Grayscale8).copy();
                             QImage::Format_BGR888).copy();
        frame_cnt++;
    }

    p_cam->StopGrabbing();

    return true;
}

bool ImgProcess::CameraCal(QLabel * pt, QLabel * pt3, QLineEdit * x, QLineEdit * y, CMvCamera* p_cam) {
    int frame_cnt = 0;
    int ret = -1;
    Mat img;
//    cv::namedWindow("video", WINDOW_NORMAL);
    std::vector<int> x_sta;
    std::vector<int> y_sta;
    std::vector<std::vector<Point2i> > pgp;

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

    qDebug("canny %d %d %d", configData.canny_1, configData.canny_2, configData.canny_3);
    qDebug("houghline %d %d %d", configData.hgline_1, configData.hgline_2, configData.hgline_3);
    const float ang1 = ((180 - configData.line1_ang) % 180) * CV_PI / 180;
    const float ang2 = ((180 - configData.line2_ang) % 180) * CV_PI / 180;
    spdlog::debug("ang1 {}.C:{:.2f}, ang2 {}.C:{:.2f}, abs {:.2f} sel_low {} {}",
                  configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_abs,
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
        Mat grayimg, color_img;
        Mat edge(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        Mat contours_img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
        Mat img(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, frame.pBufAddr);

        img.copyTo(grayimg);
        cv::cvtColor(img, color_img, COLOR_GRAY2BGR);

        Process(grayimg, edge, contours_img);

        std::vector<cv::Vec2f> lines_found;
        std::vector<float> rhos = {0, 0};
        std::vector<float> thetas = {0, 0};
        std::vector<Point2i> points;

        if (!FilterLines(contours_img, lines_found, rhos, thetas, points)) {
            p_cam->FreeImageBuffer(&frame);
            //p_cam->StopGrabbing();
            //return false;
            continue;
        }

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

        QImage qimg = QImage((const unsigned char*)(contours_img.data),
                                        contours_img.cols,
                                        contours_img.rows,
                                        contours_img.step,
                                        // QImage::Format_BGR888).copy();
                                        QImage::Format_Grayscale8).copy();
        pt3->setPixmap(QPixmap::fromImage(qimg).scaled(pt->size(), Qt::KeepAspectRatio));

        QImage oimg = QImage((const unsigned char*)(color_img.data),
                            color_img.cols,
                            color_img.rows,
                            color_img.step,
                            // QImage::Format_Grayscale8).copy();
                            QImage::Format_BGR888).copy();

        pt->setPixmap(QPixmap::fromImage(oimg).scaled(pt->size(), Qt::KeepAspectRatio));
        frame_cnt++;
        waitKey(1);
    }

    //delete imgw;
    p_cam->StopGrabbing();

    std::vector<Point2i> linesp = {Point2i(0,0),Point2i(0,0),Point2i(0,0)};
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



bool ImgProcess::Process(cv::Mat &img, cv::Mat &edge_img, cv::Mat &contour_img) {
    const int IMG_HEIGHT = img.rows;
    const int IMG_WIDTH = img.cols;
    cv::GaussianBlur(img, img, Size(configData.blur_kernel,configData.blur_kernel), 0);
    cv::threshold(img, img, 50, 255, THRESH_BINARY);
    // cv::medianBlur(grayimg, grayimg, configData.blur_kernel);
    // cv::GaussianBlur(img, img, Size(3,3), 0);
    // cv::fastNlMeansDenoising(img, img, std::vector<float>({120}));
    cv::Canny(img, edge_img, configData.canny_1, configData.canny_2, configData.canny_3);

    std::vector<vector<Point>> contours;
    std::vector<Point> hull_points;
    std::vector<Vec4i> hierarchy;

    cv::findContours(edge_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // spdlog::debug("findContours num {}", contours.size());
    // Mat contour_img(edge_img.size(), CV_8UC1, Scalar(0));
    for (size_t i = 0; i < contours.size(); i++)
    {
        // spdlog::debug("  findContours {} {}", i, contours[i].size());
        if(contours[i].size() > 3000) {
            // cv::drawContours(contours_img, contours, static_cast<int>(i), Scalar(255), 2, LINE_8, hierarchy, 0);
            // spdlog::debug("  findContours {} {}", i, contours[i].size());
            cv::convexHull(contours[i], hull_points, false, true);
            // spdlog::debug("convhull points {}", hull_points.size());
            std::vector<vector<Point>> hullv;
            hullv.push_back((hull_points));
            cv::drawContours(contour_img, hullv, 0, Scalar(255), 2, LINE_8, hierarchy, 0);
        }
    }
}

bool ImgProcess::FilterLines(cv::Mat &img, std::vector<cv::Vec2f> & lines_found,
    std::vector<float> & rhos, std::vector<float> & thetas, std::vector<Point2i> & points) {
    if (rhos.size() != thetas.size() || thetas.size() == 0) {
        spdlog::debug("invalid find lines param size!");
        return false;
    }
    const int IMG_HEIGHT = img.rows;
    const int IMG_WIDTH = img.cols;
    const float ang1 = ((180 - configData.line1_ang) % 180) * CV_PI / 180;
    const float ang2 = ((180 - configData.line2_ang) % 180) * CV_PI / 180;
    // spdlog::debug("ang1 {}C:{:.2f}, ang2 {}C:{:.2f}, abs {:.2f} sel_low {} {}",
    //               configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_abs,
    //               configData.line1_sel_low,
    //               configData.line2_sel_low);
    // std::vector<cv::Vec2f> lines_found;
    cv::HoughLines(img, lines_found,
        configData.hgline_1,
        configData.hgline_2 == 0 ? CV_PI/180 : CV_PI/360,
        configData.hgline_3);
    // spdlog::debug("lines num {}", lines_found.size());

    if (lines_found.size() == 0) {
        spdlog::debug("found lines num 0");
        return false;
    }
    
    std::vector<int> cnts;
    for (const auto v : thetas) {
        cnts.push_back(0);
    }
    for ( auto  it = lines_found.begin(); it != lines_found.end();) {
        float rho = (*it)[0];
        float thetatt = (*it)[1];
        float theta = thetatt;
        if (thetatt > CV_PI)
            theta = thetatt - CV_PI;
        // spdlog::debug("rho {:.2f}, theta {:.2f}, {:.2f}", rho, thetatt, theta * 180 / CV_PI);
    
        if (abs(theta - ang1) > configData.line_abs && abs(theta - ang2) > configData.line_abs) {
            lines_found.erase(it);
            continue;
        } else {
            it++;
            if (abs(theta - ang1) <= configData.line_abs) {
                thetas[0] += theta;
                rhos[0] += rho;
                cnts[0]++;
            } else if (abs(theta - ang2) <= configData.line_abs) {
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
        // qDebug("line %d rho %f, theta %f, cnt %d", pos, rho, theta, cnts[pos]);
            //直线与第一行的交叉点
            Point2i pt1(static_cast<int>(rho / cos(theta)), 0);
            //直线与最后一行的交叉点
            Point2i pt2(static_cast<int>(rho / cos(theta) - IMG_HEIGHT*sin(theta) / cos(theta)), IMG_HEIGHT);
            //cv::line(color_img, pt1, pt2, cv::Scalar(0,0,255), 2);
            if (cv::clipLine(Size(IMG_WIDTH, IMG_HEIGHT), pt1, pt2)) {
                if (pt1.y > pt2.y) {
                    lines.push_back(Vec4i(pt2.x, pt2.y, pt1.x, pt1.y));
                } else {
                    lines.push_back(Vec4i(pt1.x, pt1.y, pt2.x, pt2.y));
                }
            }
    }

    double ka, kb;
    ka = (double)(lines[0][3] - lines[0][1]) / (double)(lines[0][2] - lines[0][0]); //求出LineA斜率
    kb = (double)(lines[1][3] - lines[1][1]) / (double)(lines[1][2] - lines[1][0]); //求出LineB斜率

    Point2f p;
    p.x = (ka*lines[0][0] - lines[0][1] - kb*lines[1][0] + lines[1][1]) / (ka - kb);
    p.y = (ka*kb*(lines[0][0] - lines[1][0]) + ka*lines[1][1] - kb*lines[0][1]) / (ka - kb);
    // qDebug("ka %f, kb %f, p (%f, %f)", ka, kb, p.x, p.y);

    Point2i p_int(int(p.x), int(p.y));
    Point2i line1p, line2p;
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

bool ImgProcess::AdaptLines(cv::Mat &img, std::vector<cv::Vec2f> &lines_found,
    std::vector<float> & rhos, std::vector<float> & thetas) {
    if (rhos.size() != thetas.size() || thetas.size() == 0) {
        spdlog::debug("invalid find lines param size!");
        return false;
    }
    const float ang1 = ((180 - configData.line1_ang) % 180) * CV_PI / 180;
    const float ang2 = ((180 - configData.line2_ang) % 180) * CV_PI / 180;

    // spdlog::debug("ang1 {}C:{:.2f}, ang2 {}C:{:.2f}, abs {:.2f} sel_low {} {}",
    //               configData.line1_ang, ang1, configData.line2_ang, ang2, configData.line_abs,
    //               configData.line1_sel_low,
    //               configData.line2_sel_low);
    // std::vector<cv::Vec2f> lines_found;
    cv::HoughLines(img, lines_found,
        configData.hgline_1,
        configData.hgline_2 == 0 ? CV_PI/180 : CV_PI/360,
        configData.hgline_3);
    // spdlog::debug("lines num {}", lines_found.size());
    
    std::vector<int> cnts;
    for (const auto v : thetas) {
        cnts.push_back(0);
    }

    if (lines_found.size() > 1000) {
        spdlog::debug("too many lines found!");
        return false;
    }

    Mat kmeans(lines_found.size(), 2, CV_32F, Scalar(0));
    Mat bestlabels;
    Mat centers(configData.lines_num, 1000, CV_32F, Scalar(0));
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
        Mat roww = centers.row(pos);
        if (abs(roww.at<float>(1) - ang1) < 0.3) {
            rhos[0] = roww.at<float>(0);
            thetas[0] = roww.at<float>(1);
            // spdlog::debug("center {}, {:.2f}:{:.2f}", pos, rhos[0], thetas[0]);
        } else if (abs(roww.at<float>(1) - ang2) < 0.3) {
            rhos[1] = roww.at<float>(0);
            thetas[1] = roww.at<float>(1);
            // spdlog::debug("center {}, {:.2f}:{:.2f}", pos, rhos[1], thetas[1]);
        } else {
            continue;
        }
    }

    return true;
}

