#include "modbusWin.h"
#include "ui_modbusWin.h"

modbusWin::modbusWin(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::modbusWin)
{

}

void modbusWin::on_modbusSend_2_clicked()
{
    emit modbusSend(
                ui->d500->text().toFloat(),
                ui->d504->text().toFloat(),
                ui->d508->text().toFloat(),
                ui->d520->text().toFloat(),
                ui->d524->text().toFloat(),
                ui->d528->text().toFloat());
}


void modbusWin::on_read_clicked()
{
    emit modbusRead(ui->test_addr->text().toInt());
}


void modbusWin::on_write_clicked()
{
    emit modbusWrite(ui->test_addr->text().toInt(), ui->test_val->text().toFloat());
}

