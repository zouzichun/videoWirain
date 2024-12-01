#include "imgwindow.h"
#include "ui_imgwindow.h"

imgWindow::imgWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::imgWindow)
{
    ui->setupUi(this);
}

imgWindow::~imgWindow()
{
    delete ui;
}


QLabel * imgWindow::getImgPic() {
    return ui->img_label;
};
