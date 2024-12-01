#include "imgwindow.h"
#include "ui_imgwindow.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <QMouseEvent>

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
                float tt1 = (p12[1] - p11[1]) / (p12[0] - p11[0]);
                float tt2 = (p22[1] - p21[1]) / (p22[0] - p21[0]);
                spdlog::debug("line 1 tt {}, theta {}, {}", tt1, atan(tt1), atan(tt1) * 180 / 3.1415);
                spdlog::debug("line 2 tt {}, theta {}, {}", tt2, atan(tt2), atan(tt2) * 180 / 3.1415);
            }
        } else {

        }
    }
}
