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

private:
    Ui::imgWindow *ui;
};

#endif // IMGWINDOW_H
