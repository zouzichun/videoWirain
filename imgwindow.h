#ifndef IMGWINDOW_H
#define IMGWINDOW_H

#include <QWidget>
#include <QLabel>
#include <opencv2/opencv.hpp>

namespace Ui {
class imgWindow;
}

class imgWindow : public QWidget
{
    Q_OBJECT

public:
    explicit imgWindow(QWidget *parent = nullptr);
    ~imgWindow();
    QLabel * getImgPic();
    bool cal_enabled = false;

public slots:
void camera_img_refresh(cv::Mat img);

private slots:
    void on_sel1_clicked();
    void mousePressEvent(QMouseEvent *event);


private:
    Ui::imgWindow *ui;
    int total_lines = 2;
};

#endif // IMGWINDOW_H
