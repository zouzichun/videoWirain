#-------------------------------------------------
#
# Project created by QtCreator 2020-01-01T11:19:19
#
#-------------------------------------------------

QT       += core gui network serialport serialbus
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += console

TARGET = videoWirain
TEMPLATE = app

INCLUDEPATH += /home/leon/opencv/opencv-3.4.20/install/include
INCLUDEPATH += /opt/MVS/include
LIBS += /home/leon/opencv/opencv-3.4.20/install/lib/libopencv*.so
LIBS += /opt/MVS/lib/64/libMvCameraControl.so

SOURCES += main.cpp\
    MvCamera.cpp \
    imgwindow.cpp \
    maindialog.cpp \
    crcalgorithm.cpp \
    port/serial_port.cpp \
    port/modbus.cpp \
    port/port.cpp \
    img_process.cpp \
    img_process_cam.cpp

HEADERS  += maindialog.h \
    MvCamera.h \
    comdata.h \
    crcalgorithm.h \
    imgwindow.h \
    port/serial_port.h \
    port/modbus.h \
    port/port.h \
    spdlog/async.h \
    spdlog/async_logger.h \
    spdlog/common.h \
    spdlog/details/async_logger_impl.h \
    spdlog/details/circular_q.h \
    spdlog/details/console_globals.h \
    spdlog/details/file_helper.h \
    spdlog/details/fmt_helper.h \
    spdlog/details/log_msg.h \
    spdlog/details/logger_impl.h \
    spdlog/details/mpmc_blocking_q.h \
    spdlog/details/null_mutex.h \
    spdlog/details/os.h \
    spdlog/details/pattern_formatter.h \
    spdlog/details/periodic_worker.h \
    spdlog/details/registry.h \
    spdlog/details/thread_pool.h \
    spdlog/fmt/bin_to_hex.h \
    spdlog/fmt/bundled/chrono.h \
    spdlog/fmt/bundled/color.h \
    spdlog/fmt/bundled/core.h \
    spdlog/fmt/bundled/format-inl.h \
    spdlog/fmt/bundled/format.h \
    spdlog/fmt/bundled/locale.h \
    spdlog/fmt/bundled/ostream.h \
    spdlog/fmt/bundled/posix.h \
    spdlog/fmt/bundled/printf.h \
    spdlog/fmt/bundled/ranges.h \
    spdlog/fmt/bundled/time.h \
    spdlog/fmt/fmt.h \
    spdlog/fmt/ostr.h \
    spdlog/formatter.h \
    spdlog/logger.h \
    spdlog/sinks/android_sink.h \
    spdlog/sinks/ansicolor_sink.h \
    spdlog/sinks/base_sink.h \
    spdlog/sinks/basic_file_sink.h \
    spdlog/sinks/daily_file_sink.h \
    spdlog/sinks/dist_sink.h \
    spdlog/sinks/msvc_sink.h \
    spdlog/sinks/null_sink.h \
    spdlog/sinks/ostream_sink.h \
    spdlog/sinks/rotating_file_sink.h \
    spdlog/sinks/sink.h \
    spdlog/sinks/stdout_color_sinks.h \
    spdlog/sinks/stdout_sinks.h \
    spdlog/sinks/syslog_sink.h \
    spdlog/sinks/wincolor_sink.h \
    spdlog/spdlog.h \
    spdlog/tweakme.h \
    spdlog/version.h \
    tcp_server.h \
    img_process.h

FORMS    += maindialog.ui \
    imgwindow.ui

DISTFILES += \
    spdlog/LICENSE \
    spdlog/fmt/bundled/LICENSE.rst
