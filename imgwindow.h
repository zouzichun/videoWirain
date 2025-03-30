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
    int total_lines = 3;

public slots:
void camera_img_refresh(cv::Mat img);

private slots:
    void on_sel1_clicked();
    void mousePressEvent(QMouseEvent *event);


private:
    Ui::imgWindow *ui;
    const float SCALE = (2048.0f / 600.0f);
};

#endif // IMGWINDOW_H
