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
    static int curr_line_pos = 0;
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
                default:
                    break;
            }

            if (pos>=2) {
                pos = 0;

                if (curr_line_pos >= total_lines)
                    cal_enabled = false;
                std::vector<float>  p11{ui->l1_p1x->text().toFloat(), ui->l1_p1y->text().toFloat()};
                std::vector<float>  p12{ui->l1_p2x->text().toFloat(), ui->l1_p2y->text().toFloat()};
                float tt1;
                if (p12[0] - p11[0] < 0.0001) {
                    tt1 = (p12[1] - p11[1]) / 0.0001;
                } else {
                    tt1 = (p12[1] - p11[1]) / (p12[0] - p11[0]);
                }

                float dis1 = getDist_P2L(cv::Point(0,0), cv::Point(p11[0], p11[1]), cv::Point(p12[0], p12[1]));
                float ang1 = atan(tt1) * 180 / 3.1415;
                spdlog::debug("line {} tt {}, theta {}, {}, dist {}", curr_line_pos, tt1, atan(tt1), ang1, dis1 * 4.26);
                // spdlog::debug("line 1 out theta {}, dist {}", atan(tt1), atan(tt1) * 180 / 3.1415 + 90, dis1);
                // spdlog::debug("line 2 out theta {}, dist {}", atan(tt2), atan(tt2) * 180 / 3.1415 + 90, dis2);
                ui->l1_out->setText(QString::number(ang1) +QString(", ")+QString::number(dis1 * 4.26));
                curr_line_pos++;
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
