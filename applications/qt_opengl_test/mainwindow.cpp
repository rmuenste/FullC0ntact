#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->myOpenGLWidget = ui->myOpenGLWidget;
    this->tabWidget = ui->tabWidget;
    this->tabWidgetPage1 = ui->tabWidgetPage1;
    tabWidgetPage2 = new QWidget;
    tabWidgetPage2->setObjectName(QStringLiteral("tabWidgetPage2"));
    tabWidget->addTab(tabWidgetPage2, QString());
    tabWidget->setTabText(tabWidget->indexOf(tabWidgetPage2), QApplication::translate("MainWindow", "Tab 2", 0));

    cb = new QComboBox;
    cb->addItem("Surface");
    cb->addItem("Wireframe");
    cb->setCurrentIndex(1);
    ui->mainToolBar->addWidget(cb);

    connect(cb, SIGNAL(currentIndexChanged(int)), myOpenGLWidget, SLOT(drawStyleChanged(int)));

    myApp = &myOpenGLWidget->myApp;
    myApp->init("start/sampleRigidBody.xml");
    myApp->loadClothMesh();
    myApp->initSprings();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About this glorious program"),
            tr("<b>A GUI for spring meshes</b>"));
}

void MainWindow::on_actionE_xit_triggered()
{
  QApplication::exit();
}
