#ifndef IMGWINDOW_H
#define IMGWINDOW_H

#include <QWidget>
#include <QLabel>

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
    bool cal_enabled;

private slots:
    void on_sel1_clicked();
    void mousePressEvent(QMouseEvent *event);

private:
    Ui::imgWindow *ui;
};

#endif // IMGWINDOW_H
