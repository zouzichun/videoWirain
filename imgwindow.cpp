#include "comdata.h"
#include "imgwindow.h"
#include "ui_imgwindow.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <QMouseEvent>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "img_process.h"

extern ConfigData configData;

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
}

imgWindow::~imgWindow()
{
    delete ui;
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
    if (!sep_enable)
        ui->sep_but->setText(QString("sep end"));
    else
        ui->sep_but->setText(QString("sep start"));
    sep_enable = ~sep_enable;
}

extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
extern std::vector<Lines> g_lines;

void imgWindow::mousePressEvent(QMouseEvent *event) {
    if (cal_enabled) {
        if (event->QEvent::MouseButtonPress) {
            QPoint pppt = event->globalPos();
            QPoint ppp = ui->img_label->mapFromGlobal(pppt);

            float ppx = ppp.x();
            float ppy = ppp.y();
            ppx = ppx * SCALE;
            ppy = ppy * SCALE;

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
            QPoint pppt = event->globalPos();
            QPoint ppp = ui->img_label->mapFromGlobal(pppt);

            float ppx = ppp.x();
            float ppy = ppp.y();
            ppx = ppx * SCALE;
            ppy = ppy * SCALE;

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
}

void imgWindow::camera_img_refresh(cv::Mat img) {
    // qDebug("cam refreshed..");
    QImage qimg = QImage((const unsigned char*)(img.data),
        img.cols,
        img.rows,
        img.step,
        QImage::Format_Grayscale8).copy();
        // QImage::Format_RGB888).copy();

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

