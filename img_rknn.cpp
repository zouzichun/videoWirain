#include "img_process.h"
#include "crcalgorithm.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/spdlog.h"
#include "maindialog.h"
#include <QByteArray>
#include <QDebug>
#include <QPainter>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace std;
using namespace cv;

ImgProcessRknn::ImgProcessRknn(QString model_name) : ImgProcess(model_name, 2048, 2048, true) {
}

ImgProcessRknn::~ImgProcessRknn() {
}
