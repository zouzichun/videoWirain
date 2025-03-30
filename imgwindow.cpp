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

using namespace cv;

imgWindow::imgWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::imgWindow)
{
    ui->setupUi(this);
    cal_enabled = false;
}

imgWindow::~imgWindow()
{
    delete ui;
}

QLabel * imgWindow::getImgPic() {
    return ui->img_label;
};

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

extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);
extern std::vector<Lines> g_lines;
extern void pointsToHoughParams(const cv::Point& p1, 
    const cv::Point& p2,
    float& rho, 
    float& theta);

void imgWindow::mousePressEvent(QMouseEvent *event)
{
    if (cal_enabled) {
        // if (curr_line_pos == 0) {
        //     g_lines.clear();
        // }

        if (event->QEvent::MouseButtonPress) {
            QPoint pppt = event->globalPos();
            QPoint ppp = ui->img_label->mapFromGlobal(pppt);

            switch (position) {
                case 0:
                    ui->l1_p1x->setText(QString::number(ppp.x()));
                    ui->l1_p1y->setText(QString::number(ppp.y()));
                    ui->l1_p2x->setText(QString::number(0));
                    ui->l1_p2y->setText(QString::number(0));
                    ui->l1_out->setText(QString(""));
                    ui->total_lines->setText(QString::number(curr_line_pos));
                    position++;
                    break;
                case 1:
                    ui->l1_p2x->setText(QString::number(ppp.x()));
                    ui->l1_p2y->setText(QString::number(ppp.y()));
                    position++;
                break;
                default:
                    break;
            }

            if (position>=2) {
                position = 0;
                Lines line = {};

                std::vector<float>  p11{ui->l1_p1x->text().toFloat(), ui->l1_p1y->text().toFloat()};
                std::vector<float>  p12{ui->l1_p2x->text().toFloat(), ui->l1_p2y->text().toFloat()};
                cv::Point2f p1(p11[0] * SCALE, p11[1] * SCALE);
                cv::Point2f p2(p12[0] * SCALE, p12[1] * SCALE);
                
                // float tt1;
                // if (p12[0] - p11[0] < 0.0001) {
                //     tt1 = (p12[1] - p11[1]) / 0.0001;
                // } else {
                //     tt1 = (p12[1] - p11[1]) / (p12[0] - p11[0]);
                // }

                // float dis1 = getDist_P2L(cv::Point(0,0), cv::Point(p11[0], p11[1]), cv::Point(p12[0], p12[1]));
                // float ang1 = atan(tt1) * 180 / CV_PI;
                float rho, theta;
                pointsToHoughParams(p1, p2, rho, theta);
                spdlog::debug("line {}, p1 ({}, {}), p2 ({}, {}), theta {}, rho {}", curr_line_pos, p1.x, p1.y, p2.x, p2.y, theta, rho);
                // spdlog::debug("line 1 out theta {}, dist {}", atan(tt1), atan(tt1) * 180 / 3.1415 + 90, dis1);
                // spdlog::debug("line 2 out theta {}, dist {}", atan(tt2), atan(tt2) * 180 / 3.1415 + 90, dis2);
                ui->l1_out->setText(QString::number(theta) +QString(", ")+QString::number(rho));

                g_lines[curr_line_pos].rho = rho;
                g_lines[curr_line_pos].angle = theta;
            
                curr_line_pos++;
                if (curr_line_pos >= total_lines) {
                    curr_line_pos = 0;
                }
            }
        } else {

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
