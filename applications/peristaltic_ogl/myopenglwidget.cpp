#include "myopenglwidget.h"
#include <QtWidgets>
#include <GL/glu.h>
#include <GL/freeglut.h>


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

  i3d::Vector3<float> s(-10, -4, -40);
  
  //for(int j(0); j < 21; ++j)
  for(int j(0); j < 41; ++j)
    for(int i(0); i < 21; i+=2)
    {
      i3d::Vector3<float> v0 = s + i3d::Vector3<float>(float(i),0,float(j)*2);
      i3d::Vector3<float> v1 = s + i3d::Vector3<float>(float(i),0,float(j)*2+2);
      gridVertices_.push_back(v0);
      gridVertices_.push_back(v1);
    }

  loadOpenMesh();


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


  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
//  i3d::Vec3 eye(0,6,1);
//  i3d::Vec3 center(0,0,0);
//  i3d::Vec3 up(center-eye);
//  up.normalize();

//  gluLookAt(eye.x, eye.y, eye.z,
//            center.x, center.y, center.z,
//            up.x, up.y, up.z);

//  gluLookAt(0,20,5,0,0,0,0,1,0);

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

  myApp.simulateGUI(kStruct, kShear, kBend);

  glClearColor(0.4,0.4,0.4,1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
//  i3d::Vec3 eye(0,6,1);
//  i3d::Vec3 center(0,0,0);
//  i3d::Vec3 up(center-eye);
//  up.normalize();

//  gluLookAt(eye.x, eye.y, eye.z,
//            center.x, center.y, center.z,
//            up.x, up.y, up.z);
  gluLookAt(0,2,5,0,0,0,0,1,0);





  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glDisable(GL_LIGHTING);
  
  //gluLookAt(0,2,20,0,0,0,0,1,0);
  //for(int j(0); j <= 220; j+=22)
  for(int j(0); j <= 440; j+=22)
  {
    glBegin(GL_QUAD_STRIP);
    glColor3f(0.3,0.3,0.3);
    for(int i(0); i < 21; i+=2)
    {
      glVertex3fv(&gridVertices_[j+i].m_dCoords[0]);
      glVertex3fv(&gridVertices_[j+i+1].m_dCoords[0]);
    }
    glEnd();
  }


//--------------------Draw-axes-mesh-----------------------


//  glTranslatef(-6.0, -5.0, -1.0);
//  glPushMatrix();
//
//  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
//  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
//  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
//  qglColor(Qt::green);
//  glutSolidCone(0.25,-0.25,20,20);
//
//
//  glPopMatrix();



//-------------------Draw-light-as-sphere--------------------

//  glPushMatrix();
//    qglColor(Qt::green);
//    glTranslatef(0, 0.0,-3.2);
//    glutSolidSphere(0.125, 20,20);
//  glPopMatrix();

//  glPushMatrix();
  GLfloat mat_spec[]={1.0,1.0,1.0,1.0};
  GLfloat mat_shininess[] = {50};
  GLfloat lightPosition[4] = { 0, 0, -3.2, 1.0 };
  glShadeModel(GL_SMOOTH);
  glMaterialfv(GL_FRONT, GL_SPECULAR,mat_spec);
  glMaterialfv(GL_FRONT, GL_SHININESS,mat_shininess);
  glColorMaterial(GL_FRONT, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
//  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//  glTranslatef(0,-5.0,-1.0);
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glEnable(GL_LIGHT0);
//  glutSolidSphere(0.125, 20,20);
//  glPopMatrix();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  qglColor(Qt::blue);
  drawMesh(polyMesh_);
  //
//--------------------Draw-spring-mesh-----------------------

  glPushMatrix();


  glTranslatef(0.0, 0.0, -1.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);


  if(drawMode_ == 0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  else if(drawMode_ == 1)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glDisable(GL_LIGHTING);
  draw();
  glEnable(GL_LIGHTING);
  glPopMatrix();
  
}

void MyOpenGLWidget::drawMesh(i3d::PolyMesh &pm)
{

    i3d::PolyMesh::FaceIter f_it(pm.faces_begin()),
                            f_end(pm.faces_end());

    i3d::PolyMesh::FaceVertexIter fv_it;

    glBegin(GL_TRIANGLES);
    for(; f_it != f_end; ++f_it)
    {
      glNormal3fv(&pm.normal(*f_it)[0]);
      fv_it = pm.fv_iter(*f_it);  
      glVertex3fv( &pm.point(*fv_it)[0]);
      fv_it++;
      glVertex3fv( &pm.point(*fv_it)[0]);
      fv_it++;
      glVertex3fv( &pm.point(*fv_it)[0]);
    }
    glEnd();

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

}


void MyOpenGLWidget::draw()
{

    i3d::PolyMesh::FaceIter f_it(myApp.polyMesh.faces_begin()),
                            f_end(myApp.polyMesh.faces_end());

    i3d::PolyMesh::FaceVertexIter fv_it;

    glColorMaterial(GL_FRONT, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    qglColor(Qt::red);

    glPushMatrix();
    glScalef(5.0, 5.0, 5.0);
  
    glBegin(GL_TRIANGLES);
    for(; f_it!=f_end; ++f_it)
    {
      //glNormal3fv(&myApp.polyMesh.normal(*f_it)[0]);
      fv_it = myApp.polyMesh.fv_iter(*f_it);
      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
      ++fv_it;
      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
      ++fv_it;
      glVertex3fv(&myApp.polyMesh.point(*fv_it)[0]);
    }
    glEnd();
    glPopMatrix();

}

void MyOpenGLWidget::structSlider_valueChanged(int value)
{

  //std::cout << "struct value: " << value << std::endl;
  kStruct = value;
}

void MyOpenGLWidget::shearSlider_valueChanged(int value)
{
  //std::cout << "shear value: " << value << std::endl;
  kShear = value;
}

void MyOpenGLWidget::bendSlider_valueChanged(int value)
{
  //std::cout << "bend value: " << value << std::endl;
  this->kBend = value;
}

void MyOpenGLWidget::reset()
{
  myApp.loadClothMesh();
  myApp.initSprings();
}

