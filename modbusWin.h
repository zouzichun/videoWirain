#ifndef MODBUSWIN_H
#define MODBUSWIN_H

#include <QWidget>

namespace Ui {
class modbusWin;
}

class modbusWin : public QWidget
{
    Q_OBJECT
public:
    explicit modbusWin(QWidget *parent = nullptr);
    ~modbusWin();

signals:
    void modbusRead(int addr);
    void modbusWrite(int addr, float val);
    void modbusSend(float d500, float d504, float d508, float d520, float d524, float d528);

private slots:
    void on_modbusSend_2_clicked();

    void on_read_clicked();

    void on_write_clicked();

private:
    Ui::modbusWin *ui;
};

#endif // MODBUSWIN_H
