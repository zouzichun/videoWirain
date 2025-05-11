#include "comdata.h"
#include "imgwindow.h"
#include "ui_imgwindow.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <QMouseEvent>
#include <QSettings>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "img_process.h"

extern ConfigData configData;
extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
extern std::vector<Lines> g_lines;
extern std::vector<std::pair<double, double>> g_roi;
bool img_sw_status = false;

imgWindow::imgWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::imgWindow)
{
    ui->setupUi(this);
    cal_enabled = false;
    ui->a->setText(QString::number(configData.a));
    ui->b->setText(QString::number(configData.b));
    ui->c->setText(QString::number(configData.c));
    ui->d->setText(QString::number(configData.d));

    ui->inv_thd->setText(QString::number(configData.inv_thd));
    ui->canny1->setText(QString::number(configData.canny_1));
    ui->canny2->setText(QString::number(configData.canny_2));
    ui->canny3->setText(QString::number(configData.canny_3));
    ui->hgline1->setText(QString::number(configData.hgline_1));
    ui->hgline2->setText(QString::number(configData.hgline_2));
    ui->hgline3->setText(QString::number(configData.hgline_3));
    ui->hgline4->setText(QString::number(configData.hgline_4));
    ui->blur_kernel->setText(QString::number(configData.blur_kernel));
    ui->line_roh_abs->setText(QString::number(configData.line_roh_abs));
    ui->line_ang_abs->setText(QString::number(configData.line_ang_abs));

    ui->hsv_low1->setText(QString::number(configData.hsv_low1));
    ui->hsv_low2->setText(QString::number(configData.hsv_low2));
    ui->hsv_low3->setText(QString::number(configData.hsv_low3));
    ui->hsv_high1->setText(QString::number(configData.hsv_high1));
    ui->hsv_high2->setText(QString::number(configData.hsv_high2));
    ui->hsv_high3->setText(QString::number(configData.hsv_high3));

    connect(ui->inv_thd,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->canny1,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->canny2,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->canny3,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hgline1,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hgline2,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hgline3,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->blur_kernel,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->line_roh_abs,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->line_ang_abs,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);

    connect(ui->hsv_low1,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hsv_low2,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hsv_low3,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hsv_high1,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hsv_high2,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
    connect(ui->hsv_high3,&QLineEdit::editingFinished,this,&imgWindow::on_cal_editingFinished);
}

imgWindow::~imgWindow()
{
    delete ui;
}


void imgWindow::on_cal_editingFinished()
{
    configData.inv_thd = ui->inv_thd->text().toInt();
    configData.canny_1 = ui->canny1->text().toInt();
    configData.canny_2 = ui->canny2->text().toInt();
    configData.canny_3 = ui->canny3->text().toInt();
    configData.hgline_1 = ui->hgline1->text().toInt();
    configData.hgline_2 = ui->hgline2->text().toInt();
    configData.hgline_3 = ui->hgline3->text().toInt();
    configData.hgline_4 = ui->hgline4->text().toInt();
    configData.blur_kernel = ui->blur_kernel->text().toInt();
    configData.line_roh_abs = ui->line_roh_abs->text().toFloat();
    configData.line_ang_abs = ui->line_ang_abs->text().toFloat();
    configData.hsv_low1 = ui->hsv_low1->text().toInt();
    configData.hsv_low2 = ui->hsv_low2->text().toInt();
    configData.hsv_low3 = ui->hsv_low3->text().toInt();
    configData.hsv_high1 = ui->hsv_high1->text().toInt();
    configData.hsv_high2 = ui->hsv_high2->text().toInt();
    configData.hsv_high3 = ui->hsv_high3->text().toInt();
}


void imgWindow::saveImgParam()
{
    QSettings sets(QCoreApplication::applicationName()+".ini",QSettings::IniFormat);
    // Canny and hglines
    sets.setValue("inv_thd",configData.inv_thd);
    sets.setValue("canny_1",configData.canny_1);
    sets.setValue("canny_2",configData.canny_2);
    sets.setValue("canny_3",configData.canny_3);
    sets.setValue("hgline_1",configData.hgline_1);
    sets.setValue("hgline_2",configData.hgline_2);
    sets.setValue("hgline_3",configData.hgline_3);
    sets.setValue("hgline_4",configData.hgline_4);
    sets.setValue("blur_kernel",configData.blur_kernel);

    // target lines info
    sets.setValue("line_roh_abs",QString::number(configData.line_roh_abs, 'f', 3));
    sets.setValue("line_ang_abs",QString::number(configData.line_ang_abs, 'f', 3));
    sets.setValue("line_angs",configData.line_angs);
    sets.setValue("line_rhos",configData.line_rhos);
    sets.setValue("roi",configData.roi);
    sets.setValue("a",QString::number(configData.a, 'f', 6));
    sets.setValue("b",QString::number(configData.b, 'f', 6));
    sets.setValue("c",QString::number(configData.c, 'f', 6));
    sets.setValue("d",QString::number(configData.d, 'f', 6));
    sets.setValue("seprate_rho",QString::number(configData.seprate_rho, 'f', 3));
    sets.setValue("seprate_theta",QString::number(configData.seprate_theta, 'f', 3));

    sets.setValue("hsv_low1",configData.hsv_low1);
    sets.setValue("hsv_low2",configData.hsv_low2);
    sets.setValue("hsv_low3",configData.hsv_low3);
    sets.setValue("hsv_high1",configData.hsv_high1);
    sets.setValue("hsv_high2",configData.hsv_high2);
    sets.setValue("hsv_high3",configData.hsv_high3);

    sets.sync();
}

void imgWindow::on_saveImg_clicked()
{
    // ui->saveImg->setText("saving...");
    ui->saveImg->setDisabled(true);
    saveImgParam();
    ui->saveImg->setDisabled(false);
    // ui->saveImg->setText("save param");
}

int position;
int curr_line_pos;

void imgWindow::on_sel1_clicked()
{
    if (!cal_enabled)
        ui->sel1->setText(QString("结束"));
    else
        ui->sel1->setText(QString("开始"));
    cal_enabled = ~cal_enabled;
    curr_line_pos = 0;
    position = 0;
}

void imgWindow::on_sep_but_clicked()
{
    if (!sep_enable) {
        ui->sep_but->setText(QString("sep end"));
    } else {
        ui->sep_but->setText(QString("sep start"));
    }
    sep_enable = ~sep_enable;
}

void imgWindow::mouseMoveEvent(QMouseEvent* event) {
    QPoint pppt = event->globalPos();
    QPoint ppp = ui->img_label->mapFromGlobal(pppt);
    float ppx = ppp.x();
    float ppy = ppp.y();
    ppx = ppx * SCALE;
    ppy = ppy * SCALE;
    ui->mouse_pos->setText(QString::number(ppx) + QString(", ") + QString::number(ppy));
}

void imgWindow::mousePressEvent(QMouseEvent *event) {
    float ppx = 0.0f;
    float ppy = 0.0f;
    if (event->QEvent::MouseButtonPress) {
        QPoint pppt = event->globalPos();
        QPoint ppp = ui->img_label->mapFromGlobal(pppt);
        ppx = ppp.x() * SCALE;
        ppy = ppp.y() * SCALE;
    }
    if (cal_enabled) {
        if (event->QEvent::MouseButtonPress) {
            switch (position % 2) {
                case 0:
                    ui->l1_p1x->setText(QString::number(ppx));
                    ui->l1_p1y->setText(QString::number(ppy));
                    ui->l1_p2x->setText(QString::number(0));
                    ui->l1_p2y->setText(QString::number(0));
                    ui->l1_out->setText(QString(""));
                    ui->total_lines->setText(QString::number(curr_line_pos));
                    position++;
                    break;
                case 1:
                    ui->l1_p2x->setText(QString::number(ppx));
                    ui->l1_p2y->setText(QString::number(ppy));
                    position++;
                break;
                default:
                    break;
            }

            if (position % 2 == 0) {
                Lines line = {};
                std::vector<float>  p11{ui->l1_p1x->text().toFloat(), ui->l1_p1y->text().toFloat()};
                std::vector<float>  p12{ui->l1_p2x->text().toFloat(), ui->l1_p2y->text().toFloat()};
                cv::Point2f p1(p11[0], p11[1]);
                cv::Point2f p2(p12[0], p12[1]);
                float rho, theta;
                auto tmp = PointsToHoughParams(p1, p2);
                rho = tmp.first;
                theta = tmp.second;
                spdlog::debug("line {}, p1 ({}, {}), p2 ({}, {}), theta {}, rho {}", curr_line_pos, p1.x, p1.y, p2.x, p2.y, theta, rho);
                ui->l1_out->setText(QString::number(theta) +QString(", ")+QString::number(rho));

                g_lines[curr_line_pos].rho = rho;
                g_lines[curr_line_pos].angle = theta;
            
                curr_line_pos++;
                if (curr_line_pos >= total_lines) {
                    curr_line_pos = 0;
                    position = 0;
                }
            }
        }
    }

    if (sep_enable) {
        if (event->QEvent::MouseButtonPress) {
            switch (position % 2) {
                case 0:
                    ui->sep1x->setText(QString::number(ppx));
                    ui->sep1y->setText(QString::number(ppy));
                    ui->sep2x->setText(QString::number(0));
                    ui->sep2y->setText(QString::number(0));
                    position++;
                    break;
                case 1:
                    ui->sep2x->setText(QString::number(ppx));
                    ui->sep2y->setText(QString::number(ppy));
                    position++;
                break;
                default:
                    break;
            }

            if (position % 2 == 0) {
                position = 0;
                Lines line = {};
                std::vector<float>  p11{ui->sep1x->text().toFloat(), ui->sep1y->text().toFloat()};
                std::vector<float>  p12{ui->sep2x->text().toFloat(), ui->sep2y->text().toFloat()};
                cv::Point2f p1(p11[0], p11[1]);
                cv::Point2f p2(p12[0], p12[1]);
                auto tmp = PointsToHoughParams(p1, p2);
                configData.seprate_rho = tmp.first;
                configData.seprate_theta = tmp.second;
                auto tt = HoughToPoints(configData.seprate_rho, configData.seprate_theta);
                configData.seprate_p1x = tt.first.x;
                configData.seprate_p1y = tt.first.y;
                configData.seprate_p2x = tt.second.x;
                configData.seprate_p2y = tt.second.y;
                spdlog::info("seprate p1 ({},{}), p2 ({},{}), rho {}, theta {}", configData.seprate_p1x, configData.seprate_p1y,
                    configData.seprate_p2x, configData.seprate_p2y, configData.seprate_rho, configData.seprate_theta);
            }
        }

    }

    if (roi_enable) {
        g_roi.push_back(std::pair<double, double>(ppx, ppy));
        spdlog::info("roi point ({:.2f},{:.2f})", ppx, ppy);
    }
}

void imgWindow::calibration_refresh(cv::Mat img) {
    QImage qimg;
    if (img_sw_status) {
        qimg = QImage((const unsigned char*)(img.data),
                           img.cols,
                           img.rows,
                           img.step,
                           QImage::Format_Grayscale8).copy();

    } else {
        qimg = QImage((const unsigned char*)(img.data),
                           img.cols,
                           img.rows,
                           img.step,
                           QImage::Format_RGB888).copy();
    }

    QPixmap piximg = QPixmap::fromImage(qimg);

    if (!piximg.isNull()) {
        ui->img_label->setPixmap(piximg.scaled(ui->img_label->size(), Qt::KeepAspectRatio));
    }
}

void imgWindow::on_img2mach_clicked()
{
    float a,b,c,d;
    float x1_img,y1_img,x2_img,y2_img;
    float x1_mach,y1_mach,x2_mach,y2_mach;
    x1_img = ui->x1_img->text().toFloat();
    y1_img = ui->y1_img->text().toFloat();
    x2_img = ui->x2_img->text().toFloat();
    y2_img = ui->y2_img->text().toFloat();
    a = ui->a->text().toFloat();
    b = ui->b->text().toFloat();
    c = ui->c->text().toFloat();
    d = ui->d->text().toFloat();

    x1_mach = x1_img * a - b * y1_img + c;
    y1_mach = x1_img * b + a * y1_img + d;
    x2_mach = x2_img * a - b * y2_img + c;
    y2_mach = x2_img * b + a * y2_img + d;

    ui->x1_mach->setText(QString::number(x1_mach));
    ui->y1_mach->setText(QString::number(y1_mach));
    ui->x2_mach->setText(QString::number(x2_mach));
    ui->y2_mach->setText(QString::number(y2_mach));
}

void imgWindow::on_abcd_clicked()
{
    float a,b,c,d;
    float x1_img,y1_img,x2_img,y2_img;
    float x1_mach,y1_mach,x2_mach,y2_mach;
    x1_img = ui->x1_img->text().toFloat();
    y1_img = ui->y1_img->text().toFloat();
    x2_img = ui->x2_img->text().toFloat();
    y2_img = ui->y2_img->text().toFloat();
    x1_mach = ui->x1_mach->text().toFloat();
    y1_mach = ui->y1_mach->text().toFloat();
    x2_mach = ui->x2_mach->text().toFloat();
    y2_mach = ui->y2_mach->text().toFloat();

    a = ((x1_mach - x2_mach) * (x1_img - x2_img) + (y1_mach - y2_mach) * (y1_img - y2_img))
        / ((x1_img - x2_img) * (x1_img - x2_img) + (y1_img - y2_img) * (y1_img - y2_img));
    b = ((y1_mach - y2_mach) * (x1_img - x2_img) - (x1_mach - x2_mach) * (y1_img - y2_img))
        / ((x1_img - x2_img) * (x1_img - x2_img) + (y1_img - y2_img) * (y1_img - y2_img));
    c = x1_mach - a * x1_img + b * y1_img;
    d = y1_mach - b * x1_img - a * y1_img;

    ui->a->setText(QString::number(a));
    ui->b->setText(QString::number(b));
    ui->c->setText(QString::number(c));
    ui->d->setText(QString::number(d));

    configData.a = a;
    configData.b = b;
    configData.c = c;
    configData.d = d;
}

void imgWindow::on_mach2img_clicked()
{
    float a,b,c,d;
    float x1_img,y1_img,x2_img,y2_img;
    float x1_mach,y1_mach,x2_mach,y2_mach;
    x1_mach = ui->x1_mach->text().toFloat();
    y1_mach = ui->y1_mach->text().toFloat();
    x2_mach = ui->x2_mach->text().toFloat();
    y2_mach = ui->y2_mach->text().toFloat();

    a = ui->a->text().toFloat();
    b = ui->b->text().toFloat();
    c = ui->c->text().toFloat();
    d = ui->d->text().toFloat();

    float denominator = a * a + b * b;
    x1_img = (a * (x1_mach - c) + b * (y1_mach - d)) / denominator;
    y1_img = (-b * (x1_mach - c) + a * (y1_mach - d)) / denominator;
    x2_img = (a * (x2_mach - c) + b * (y2_mach - d)) / denominator;
    y2_img = (-b * (x2_mach - c) + a * (y2_mach - d)) / denominator;

    ui->x1_img->setText(QString::number(x1_img));
    ui->y1_img->setText(QString::number(y1_img));
    ui->x2_img->setText(QString::number(x2_img));
    ui->y2_img->setText(QString::number(y2_img));
}

void imgWindow::on_roi_sel_clicked()
{
    if (!roi_enable) {
        ui->roi_sel->setText(QString("roi end"));
        configData.roi.clear();
    } else {
        ui->roi_sel->setText(QString("roi start"));
        std::stringstream ss;
        for (const auto v: g_roi) {
            ss << v.first << "," << v.second << ";";
        }
        spdlog::info("roi points: {}", ss.str());
        configData.roi = QString(ss.str().c_str());;
    }
    roi_enable = !roi_enable;
}

void imgWindow::on_img_sw_clicked()
{
    img_sw_status = !img_sw_status;
    spdlog::info("img sw sts {}", img_sw_status);
}

bool debug_win_enable = false;
void imgWindow::on_dbg_win_clicked()
{
    debug_win_enable = !debug_win_enable;
    spdlog::info("debug win {}", debug_win_enable);
    if (debug_win_enable) {
        ui->dbg_win->setText(QString("debug win off"));
        cv::namedWindow("hsv_img", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("gray_img", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("edge_up", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("edge_down", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("cnt_img", cv::WINDOW_AUTOSIZE);
    } else {
        ui->dbg_win->setText(QString("debug win on"));
        cv::destroyWindow("hsv_img");
        cv::destroyWindow("gray_img");
        cv::destroyWindow("edge_up");
        cv::destroyWindow("edge_down");
        cv::destroyWindow("cnt_img");
    }
}

