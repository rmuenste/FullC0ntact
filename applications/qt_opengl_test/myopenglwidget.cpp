#include "myopenglwidget.h"
#include <QtWidgets>
#include<GL/glu.h>

MyOpenGLWidget::MyOpenGLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
  xRot = 0;
  yRot = 0;
  zRot = 0;
  timer = new QTimer(this);
  timer->start(16);
  time_ = new QTime();
  startTime = time_->currentTime();
  time_->start();
  connect(timer, SIGNAL(timeout()), this, SLOT(repaint()));
}

MyOpenGLWidget::~MyOpenGLWidget()
{
}

QSize MyOpenGLWidget::minimumSizeHint() const
{
  return QSize(50, 50);
}

QSize MyOpenGLWidget::sizeHint() const
{
  return QSize(400, 400);
}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360)
        angle -= 360 * 16;
}

void MyOpenGLWidget::setXRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != xRot) {
      xRot = angle;
      emit xRotationChanged(angle);
      updateGL();
  }
}

void MyOpenGLWidget::setYRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != yRot) {
      yRot = angle;
      emit yRotationChanged(angle);
      updateGL();
  }
}

void MyOpenGLWidget::setZRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != zRot) {
      zRot = angle;
      emit zRotationChanged(angle);
      updateGL();
  }
}

void MyOpenGLWidget::initializeGL()
{
  qglClearColor(Qt::black);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  static GLfloat lightPosition[4] = { 0, 0, 10, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  gluLookAt(0,0,5,0,0,0,0,1,0);
}

void MyOpenGLWidget::paintGL()
{

  time_->restart();
  glClearColor(0.4,0.4,0.4,1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glPushMatrix();
  glTranslatef(0.0, 0.0, -5.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
  draw();
  glPopMatrix();
  //drawAxes();
  //std::cout << "Time elapsed: " << time_->msecsTo(startTime) << " [msecs]" << std::endl;
  
}

void MyOpenGLWidget::repaint()
{
  paintGL();
}

void MyOpenGLWidget::resizeGL(int width, int height)
{
  if(height == 0)
    height = 1;

  glViewport(0,0,width,height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(70.0,float(width)/float(height),0.1, 1000.0);
  glMatrixMode(GL_MODELVIEW);
}

void MyOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
}

void MyOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->x() - lastPos.x();
  int dy = event->y() - lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
      setXRotation(xRot + 8 * dy);
      setYRotation(yRot + 8 * dx);
  } else if (event->buttons() & Qt::RightButton) {
      setXRotation(xRot + 8 * dy);
      setZRotation(zRot + 8 * dx);
  }

  lastPos = event->pos();
}

void MyOpenGLWidget::drawAxes()
{
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

  glTranslatef(0,0,-2);
  glBegin(GL_LINES);
   glColor3f(1,0,0);
   glVertex3f(-1,-1,0);
   glVertex3f(1,-1,0);
  glEnd();
  glBegin(GL_LINES);
   glColor3f(0,1,0);
   glVertex3f(-1,-1,0);
   glVertex3f(-1,1,0);
  glEnd();
  glBegin(GL_LINES);
    glColor3f(0,0,1);
    glVertex3f(-1,-1,0);
    glVertex3f(-1,-1,-1);
  glEnd();
}

void MyOpenGLWidget::draw()
{

    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    qglColor(Qt::red);
    glBegin(GL_QUADS);
        glNormal3f(0,0,-1);
        glVertex3f(-1,-1,0);
        glVertex3f(-1,1,0);
        glVertex3f(1,1,0);
        glVertex3f(1,-1,0);

    glEnd();
    glBegin(GL_TRIANGLES);
        glNormal3f(0,-1,0.707);
        glVertex3f(-1,-1,0);
        glVertex3f(1,-1,0);
        glVertex3f(0,0,1.2);
    glEnd();
    glBegin(GL_TRIANGLES);
        glNormal3f(1,0, 0.707);
        glVertex3f(1,-1,0);
        glVertex3f(1,1,0);
        glVertex3f(0,0,1.2);
    glEnd();
    glBegin(GL_TRIANGLES);
        glNormal3f(0,1,0.707);
        glVertex3f(1,1,0);
        glVertex3f(-1,1,0);
        glVertex3f(0,0,1.2);
    glEnd();
    glBegin(GL_TRIANGLES);
        glNormal3f(-1,0,0.707);
        glVertex3f(-1,1,0);
        glVertex3f(-1,-1,0);
        glVertex3f(0,0,1.2);
    glEnd();
}
