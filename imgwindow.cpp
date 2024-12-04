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

void imgWindow::on_sel1_clicked()
{
    cal_enabled = ~cal_enabled;
}

extern float getDist_P2L(cv::Point pointP, cv::Point pointA, cv::Point pointB);

void imgWindow::mousePressEvent(QMouseEvent *event)
{
    static int pos = 0;
    if (cal_enabled) {
        if (event->QEvent::MouseButtonPress) {
            QPoint pppt = event->globalPos();
            QPoint ppp = ui->img_label->mapFromGlobal(pppt);
            switch (pos) {
                case 0:
                    ui->l1_p1x->setText(QString::number(ppp.x()));
                    ui->l1_p1y->setText(QString::number(ppp.y()));
                    pos++;
                    break;
                case 1:
                    ui->l1_p2x->setText(QString::number(ppp.x()));
                    ui->l1_p2y->setText(QString::number(ppp.y()));
                    pos++;
                break;
                case 2:
                    ui->l2_p1x->setText(QString::number(ppp.x()));
                    ui->l2_p1y->setText(QString::number(ppp.y()));
                    pos++;
                    break;
                case 3:
                    ui->l2_p2x->setText(QString::number(ppp.x()));
                    ui->l2_p2y->setText(QString::number(ppp.y()));
                    pos++;
                    break;
                default:
                    break;
            }

            if (pos>=4) {
                pos = 0;
                cal_enabled = false;
                std::vector<float>  p11{ui->l1_p1x->text().toFloat(), ui->l1_p1y->text().toFloat()};
                std::vector<float>  p12{ui->l1_p2x->text().toFloat(), ui->l1_p2y->text().toFloat()};
                std::vector<float>  p21{ui->l2_p1x->text().toFloat(), ui->l2_p1y->text().toFloat()};
                std::vector<float>  p22{ui->l2_p2x->text().toFloat(), ui->l2_p2y->text().toFloat()};
                float tt1, tt2;
                if (p12[0] - p11[0] < 0.0001) {
                    tt1 = (p12[1] - p11[1]) / 0.0001;
                } else {
                    tt1 = (p12[1] - p11[1]) / (p12[0] - p11[0]);
                }

                if (p22[0] - p21[0] < 0.0001) {
                    tt2 = (p22[1] - p21[1]) / 0.0001;
                } else {
                    tt2 = (p22[1] - p21[1]) / (p22[0] - p21[0]);
                }

                float dis1 = getDist_P2L(cv::Point(0,0), cv::Point(p11[0], p11[1]), cv::Point(p12[0], p12[1]));
                float dis2 = getDist_P2L(cv::Point(0,0), cv::Point(p21[0], p21[1]), cv::Point(p22[0], p22[1]));
                float ang1 = atan(tt1) * 180 / 3.1415;
                float ang2 = atan(tt2) * 180 / 3.1415;
                spdlog::debug("line 1 tt {}, theta {}, {}, dist {}", tt1, atan(tt1), ang1, dis1 * 4.26);
                spdlog::debug("line 2 tt {}, theta {}, {}, dist {}", tt2, atan(tt2), ang2, dis2 * 4.26);
                // spdlog::debug("line 1 out theta {}, dist {}", atan(tt1), atan(tt1) * 180 / 3.1415 + 90, dis1);
                // spdlog::debug("line 2 out theta {}, dist {}", atan(tt2), atan(tt2) * 180 / 3.1415 + 90, dis2);
                ui->l1_out->setText(QString::number(ang1) +QString(", ")+QString::number(dis1 * 4.26));
                ui->l2_out->setText(QString::number(ang2) +QString(", ")+QString::number(dis2 * 4.26));
            }
        } else {

        }
    }
}
