#-------------------------------------------------
#
# Project created by QtCreator 2016-11-08T22:02:53
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_opengl_test
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    myopenglwidget.cpp

HEADERS  += mainwindow.h \
    myopenglwidget.h

FORMS    += mainwindow.ui

LIBS += -lGLU
