#include "mainwindow.h"
#include <QApplication>

#ifdef WIN32
#include <GL/glut.h>
#else
#include <GL/freeglut.h>
#endif

int main(int argc, char *argv[])
{
    //glutInit(&argc, argv);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
