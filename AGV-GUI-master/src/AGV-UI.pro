#-------------------------------------------------
#
# Project created by QtCreator 2019-06-03T15:03:10
#
#-------------------------------------------------

QT       += core gui qml quick multimedia
INCLUDEPATH += /opt/ros/kinetic/include \
#               /usr/local/include/opencv \
#              /usr/local/include/opencv2
INCLUDEPATH += /includes

LIBS += /usr/lib/x86_64-linux-gnu/libboost_system.so \
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime -lrviz -limage_transport

#LIBS += /usr/local/lib/libopencv_highgui.so \
#        /usr/local/lib/libopencv_core.so    \
#        /usr/local/lib/libopencv_imgproc.so

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = AGV-UI #test2
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11
CONFIG += console

RESOURCES += resource.qrc


SOURCES += \
        ListenerNode.cpp \
        MarkerNode.cpp \
        TalkerNode.cpp \
        VizlibWidget.cpp \
        main.cpp \
        mainwindow.cpp

HEADERS += \
        includes/MarkerNode.h \
        includes/Settings.h \
        includes/TalkerNode.h \
        includes/VizlibWidget.h \
        includes/buttons.h \
        includes/labels.h \
        includes/mainwindow.h \
        includes/ListenerNode.h \
        includes/msgs.h

FORMS += \
        mainwindow.ui

# UI_DIR=./UI

QMAKE_CXXFLAGS += -DBUILD_SHARED_LIBS=ON

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
