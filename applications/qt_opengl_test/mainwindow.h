#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <myopenglwidget.h>
#include <QWidget>
#include <QTabWidget>
#include <QComboBox>
#include <QSlider>
#include <QPushButton>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    i3d::OpenMeshTest *myApp;


private slots:
    void on_actionAbout_triggered();

    void on_actionE_xit_triggered();

private:
    Ui::MainWindow *ui;
    QWidget *centralWidget;
    MyOpenGLWidget *myOpenGLWidget;
    QTabWidget *tabWidget;
    QWidget *tabWidgetPage1;
    QWidget *tabWidgetPage2;
    QComboBox *cb;

    QSlider *slider1_;
    QSlider *slider2_;
    QSlider *slider3_;
    QPushButton *pb;

};

#endif // MAINWINDOW_H
