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
    bool cal_enabled = false;
    int total_lines = 6;
    bool sep_enable = false;
    bool roi_enable = false;

signals:
    void img_sw_status_changed(int mode);

public slots:
void calibration_refresh(cv::Mat img);

private slots:
    void on_sel1_clicked();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);

    void on_img2mach_clicked();

    void on_abcd_clicked();

    void on_mach2img_clicked();

    void on_sep_but_clicked();

    void on_roi_sel_clicked();

    void on_img_sw_clicked();

private:
    Ui::imgWindow *ui;
    const float SCALE = (2048.0f / 600.0f);
    int img_sw_status = 0;
};

#endif // IMGWINDOW_H
