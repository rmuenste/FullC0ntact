#include "myopenglwidget.h"
#include <QtWidgets>
#include<GL/glu.h>

MyOpenGLWidget::MyOpenGLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
  xRot = 0;
  yRot = 0;
  zRot = 0;
  firstTime = true;
  timer = new QTimer(this);
  timer->start(16);
  time_ = new QTime();
  startTime = time_->currentTime();
  time_->start();
  drawMode_ = 1;
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

// slots for draw style    
void MyOpenGLWidget::drawStyleChanged(int _style)
{
  this->drawMode_ = _style;
}

void MyOpenGLWidget::initializeGL()
{

  qglClearColor(Qt::black);

  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  static GLfloat lightPosition[4] = { 0, 0, 10, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
//  i3d::Vec3 eye(0,6,1);
//  i3d::Vec3 center(0,0,0);
//  i3d::Vec3 up(center-eye);
//  up.normalize();

//  gluLookAt(eye.x, eye.y, eye.z,
//            center.x, center.y, center.z,
//            up.x, up.y, up.z);

  gluLookAt(0,20,5,0,0,0,0,1,0);

}

void MyOpenGLWidget::paintGL()
{

  time_->restart();
  int tToStart = time_->msecsTo(startTime);

  if(firstTime)
  {
    firstTime = false;
    myApp.setStartTime(tToStart);
  }
  else
  {
    myApp.calcTimeStep(tToStart);
  }
  myApp.simulate();

  glClearColor(0.4,0.4,0.4,1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
//  i3d::Vec3 eye(0,6,1);
//  i3d::Vec3 center(0,0,0);
//  i3d::Vec3 up(center-eye);
//  up.normalize();

  gluLookAt(0,2,5,0,0,0,0,1,0);
//  gluLookAt(eye.x, eye.y, eye.z,
//            center.x, center.y, center.z,
//            up.x, up.y, up.z);

  glTranslatef(0.0, 0.0, -1.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);


  if(drawMode_ == 0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  else if(drawMode_ == 1)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  draw();
  //drawAxes();
  
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

  gluPerspective(70.0,float(width)/float(height),0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  i3d::Vec3 eye(0,6,1);
  i3d::Vec3 center(0,0,0);
  i3d::Vec3 up(center-eye);
  up.normalize();

//  gluLookAt(eye.x, eye.y, eye.z,
//            center.x, center.y, center.z,
//            up.x, up.y, up.z);

  gluLookAt(0,20,5,0,0,0,0,1,0);

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

    i3d::PolyMesh::FaceIter f_it(myApp.polyMesh.faces_begin()),
                            f_end(myApp.polyMesh.faces_end());

    i3d::PolyMesh::FaceVertexIter fv_it;

    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    qglColor(Qt::red);
    int row = myApp.vrow;
    int offset=0;
    for(int j(0); j < row-1; ++j)
    {
      glBegin(GL_TRIANGLE_STRIP);
      for(int i(0); i < row-1; ++i)
      {
        i3d::PolyMesh::VertexHandle A = myApp.polyMesh.vertex_handle(offset + i);
        i3d::PolyMesh::VertexHandle B = myApp.polyMesh.vertex_handle(offset + i+1);
        i3d::PolyMesh::VertexHandle C = myApp.polyMesh.vertex_handle(offset + i+row);
        i3d::PolyMesh::VertexHandle D = myApp.polyMesh.vertex_handle(offset + i+row+1);
      
        glNormal3fv(&myApp.polyMesh.normal(A)[0]);
        glVertex3fv(&myApp.polyMesh.point(A)[0]);
      
        glNormal3fv(&myApp.polyMesh.normal(B)[0]);
        glVertex3fv(&myApp.polyMesh.point(B)[0]);

        glNormal3fv(&myApp.polyMesh.normal(C)[0]);
        glVertex3fv(&myApp.polyMesh.point(C)[0]);

        glNormal3fv(&myApp.polyMesh.normal(D)[0]);
        glVertex3fv(&myApp.polyMesh.point(D)[0]);

      }
      glEnd();
      offset += row;
    }
  
//    glBegin(GL_TRIANGLES);
//    for(; f_it!=f_end; ++f_it)
//    {
////      glNormal3fv(&myApp.polyMesh.normal(*f_it)[0]);
//      fv_it = myApp.polyMesh.fv_iter(*f_it);
//      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
//      ++fv_it;
//      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
//      ++fv_it;
//      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
//    }
//    glEnd();

//    glBegin(GL_QUADS);
//        glNormal3f(0,0,-1);
//        glVertex3f(-1,-1,0);
//        glVertex3f(-1,1,0);
//        glVertex3f(1,1,0);
//        glVertex3f(1,-1,0);
//
//    glEnd();
//    glBegin(GL_TRIANGLES);
//        glNormal3f(0,-1,0.707);
//        glVertex3f(-1,-1,0);
//        glVertex3f(1,-1,0);
//        glVertex3f(0,0,1.2);
//    glEnd();
//    glBegin(GL_TRIANGLES);
//        glNormal3f(1,0, 0.707);
//        glVertex3f(1,-1,0);
//        glVertex3f(1,1,0);
//        glVertex3f(0,0,1.2);
//    glEnd();
//    glBegin(GL_TRIANGLES);
//        glNormal3f(0,1,0.707);
//        glVertex3f(1,1,0);
//        glVertex3f(-1,1,0);
//        glVertex3f(0,0,1.2);
//    glEnd();
//    glBegin(GL_TRIANGLES);
//        glNormal3f(-1,0,0.707);
//        glVertex3f(-1,1,0);
//        glVertex3f(-1,-1,0);
//        glVertex3f(0,0,1.2);
//    glEnd();

}
