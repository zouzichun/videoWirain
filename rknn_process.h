#ifndef RKNNPROCESS_H
#define RKNNPROCESS_H

#include <QString>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <iostream>
#include <functional>
#include <thread>
// #include <rknn_api.h>

class RknnProcess
{
public:
    explicit RknnProcess(QString dev_name);
    ~RknnProcess();

    bool Init();
    bool Deinit();
    int Run();

    unsigned char* input_buffer = nullptr;
    int8_t* output_buffer = nullptr;

    rknn_tensor_mem* input_mem;
    rknn_tensor_mem* output_mem;
    rknn_tensor_attr input_attrs;
    rknn_tensor_attr output_attrs;

private:
    QString m_model_name;
    unsigned char* model = nullptr;
    int model_size = 0;
    rknn_context ctx = 0;

};

#endif  // RKNNPROCESS_H
