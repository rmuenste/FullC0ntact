#include <cppinterface.h>
#include <fbm_func.hpp>
#include <soft_body_func.hpp>
#include <output_func.hpp>
#include <set_get_func.hpp>
#include <random>

#ifdef OPTIC_FORCES
  #include <optics_func.hpp>
#endif 

#ifdef FEATFLOWLIB
#ifdef FC_CUDA_SUPPORT
  #include <GL/glew.h>
  #if defined (_WIN32)
  #include <GL/wglew.h>
  #endif
  #if defined(__APPLE__) || defined(__MACOSX)
  #include <GLUT/glut.h>
  #else
  #include <GL/freeglut.h>
  #endif
#endif

#define GRID_SIZE       16

#ifdef FC_CUDA_SUPPORT
uint3 gridSize;
#endif

int mystep = 0;

extern "C" void brownianDisplacement()
{
  std::cout << myWorld.parInfo_.getId() << "> brownianDisp: " << std::endl;

  Real dt = 0.00001;

  const double k_b = 1.38064852e-8; // [Joule/Kelvin]

  const double T = 293.0; // [Kelvin]

  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> nd(0.0,1.0);

  double RX = nd(gen);

  Real pi = CMath<Real>::SYS_PI;

  Real D = k_b * T / (6.0 * pi * 1000.0 * 0.03);

  Real dx = std::sqrt(2.0 * D * dt) * RX;

  RigidBody *body = myWorld.rigidBodies_[0];

  if(myWorld.parInfo_.getId()==1 || myWorld.parInfo_.getId()==0)
  {
    std::cout << myWorld.parInfo_.getId() << "> RX: " << RX << std::endl;
    std::cout << myWorld.parInfo_.getId() << "> vel before brownian: " << body->velocity_;
    std::cout << myWorld.parInfo_.getId() << "> brownian dx: " << dx << std::endl;
  }

  body->velocity_.x += dx/0.00004;

  if(myWorld.parInfo_.getId()==1 || myWorld.parInfo_.getId()==0)
  {
    std::cout << myWorld.parInfo_.getId() << "> vel after brownian: " << body->velocity_;
  }

  std::cout << myWorld.parInfo_.getId() << "> End brownianDisp" << std::endl;

} 

extern "C" void cudaGLInit(int argc, char **argv);

extern "C" void velocityupdate()
{

  double *ForceX  = new double[myWorld.rigidBodies_.size()];
  double *ForceY  = new double[myWorld.rigidBodies_.size()];
  double *ForceZ  = new double[myWorld.rigidBodies_.size()];
  double *TorqueX = new double[myWorld.rigidBodies_.size()];
  double *TorqueY = new double[myWorld.rigidBodies_.size()];
  double *TorqueZ = new double[myWorld.rigidBodies_.size()];

  //get the forces from the cfd-solver
#ifdef WIN32
  COMMUNICATEFORCE(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
#else
  communicateforce_(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
#endif

  std::vector<VECTOR3> vForce;
  std::vector<VECTOR3> vTorque;  

  std::vector<RigidBody*>::iterator vIter;  
  int count = 0;

  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++,count++)
  {
    RigidBody *body = *vIter;
    vForce.push_back(VECTOR3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(VECTOR3(TorqueX[count],TorqueY[count],TorqueZ[count]));
  }

  int id = 0;

  RigidBody *body = myWorld.rigidBodies_[id];

  //#ifdef WITH_ODE
  BodyODE &b = myWorld.bodies_[body->odeIndex_];

  dBodyAddForce(b._bodyId, ForceX[id],
                           ForceY[id],
                           ForceZ[id]);

//  Vec3 maxForce(0,0,0);
//  int imax = 0;
//  for (int i = 0; i < vForce.size(); ++i)
//  {
//    if(maxForce.mag() < vForce[i].mag())
//    {
//      maxForce = vForce[i];
//      imax = i;
//    } 
//  }

  if(myWorld.parInfo_.getId()==1)
  {
    //    std::cout << "> count: " << count << std::endl;
    //    std::cout << "> Force max: " << maxForce << " (pg*micrometer)/s^2 " <<std::endl; 
    //    std::cout << "> Force max index: " << imax <<std::endl; 
    //    std::cout << "> body force: " << myWorld.rigidBodies_[49]->force_.z <<std::endl; 
    //std::cout << "> Force end2: " << vForce[99].z << " (pg*micrometer)/s^2 " <<std::endl; 
  }

  //calculate the forces in the current timestep by a semi-implicit scheme
  //myPipeline.integrator_->updateForces(vForce,vTorque);

  delete[] ForceX;
  delete[] ForceY;
  delete[] ForceZ;
  delete[] TorqueX;
  delete[] TorqueY;
  delete[] TorqueZ;      

}

#ifdef FC_CUDA_SUPPORT
// initialize OpenGL
void initGL(int *argc, char **argv)
{  
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(width, height);
  glutCreateWindow("CUDA Particles");

  glewInit();
  if (!glewIsSupported("GL_VERSION_2_0 GL_VERSION_1_5 GL_ARB_multitexture GL_ARB_vertex_buffer_object")) {
    fprintf(stderr, "Required OpenGL extensions missing.");
    exit(-1);
  }

#if defined (_WIN32)
  if (wglewIsSupported("WGL_EXT_swap_control")) {
    // disable vertical sync
    wglSwapIntervalEXT(0);
  }
#endif

  glEnable(GL_DEPTH_TEST);
  glClearColor(0.25, 0.25, 0.25, 1.0);
  glutReportErrors();

}
#endif

//-------------------------------------------------------------------------------------------------------

void addcylinderboundary()
{
  //initialize the cylinder shaped boundary
  myWorld.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_ = 0;
  body->volume_ = 0;
  body->invMass_ = 0;
  body->angle_ = VECTOR3(0, 0, 0);
  body->setAngVel(VECTOR3(0, 0, 0));
  body->velocity_ = VECTOR3(0, 0, 0);
  body->shapeId_ = RigidBody::CYLINDERBDRY;

  BoundaryCylr *cyl = new BoundaryCylr();
  cyl->boundingBox_.init(myParameters.extents_[0],
      myParameters.extents_[2],
      myParameters.extents_[4],
      myParameters.extents_[1],
      myParameters.extents_[3],
      myParameters.extents_[5]);

  cyl->cylinder_ = Cylinderr(cyl->boundingBox_.getCenter(), VECTOR3(0.0, 0.0, 1.0), cyl->boundingBox_.extents_[0], cyl->boundingBox_.extents_[2]);
  body->com_ = cyl->boundingBox_.getCenter();
  body->shape_ = cyl;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void intersecthexbody(double dMinMax[][3], int *iid, int *intersection)
{

  Real minx = Real(dMinMax[0][0]);
  Real miny = Real(dMinMax[0][1]);
  Real minz = Real(dMinMax[0][2]);
  Real maxx = Real(dMinMax[1][0]);
  Real maxy = Real(dMinMax[1][1]);
  Real maxz = Real(dMinMax[1][2]);

  AABB3r box(VECTOR3(minx,miny,minz),VECTOR3(maxx,maxy,maxz)); 
  int i = *iid;
  RigidBody *pBody  = myWorld.rigidBodies_[i];
  Shaper *pShape    = pBody->getWorldTransformedShape();
  AABB3r boxBody    = pShape->getAABB();
  CIntersector2AABB<Real> intersector(box,boxBody);
  bool bIntersect =  intersector.Intersection();
  delete pShape;
  if(bIntersect)
    *intersection=1;
  else
    *intersection=0;

}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettiming(double *time)
{
  double dtime=0.0;
  dtime = myTimer.GetTime();
  *time=dtime;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void starttiming()
{
  myTimer.Start();
}

//-------------------------------------------------------------------------------------------------------
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
      fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
    for (int i=0; i<n; i++) 
    {
      contact[i].surface.mode = 0;
      contact[i].surface.mu = 50.0; // was: dInfinity
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
  }
}

// simulation loop
void simulationLoop (int istep)
{
  dSpaceCollide (space,0,&nearCallback);

  dWorldQuickStep (world, 0.01); // 100 Hz

  dJointGroupEmpty (contactgroup);

  //printf("Time: %f |Step: %d |\n",simTime, istep);
  //simTime += dt;
}

extern "C" void startcollisionpipeline()
{
  //start the collision pipeline
  //myPipeline.startPipeline();
  simulationLoop(mystep);
  //myApp.step();
}
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------

extern "C" void clearcollisionpipeline()
{
  //erase the old info, if there is any
  //myResponses.clear();
  //make a copy of the responses for postprocessing
  //myResponses=myPipeline.m_Response->m_Responses;
  myPipeline.collInfo_.clear();
}

//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void creategrid()
{

}

//-------------------------------------------------------------------------------------------------------

void createcolltest()
{

}

////-------------------------------------------------------------------------------------------------------
//#include <fbm_func.cpp>
////-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------

void configureBoundary()
{
  //initialize the box shaped boundary
  myWorld.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_ = 0;
  body->volume_ = 0;
  body->invMass_ = 0;
  body->angle_ = VECTOR3(0, 0, 0);
  body->setAngVel(VECTOR3(0, 0, 0));
  body->velocity_ = VECTOR3(0, 0, 0);
  body->shapeId_ = RigidBody::BOUNDARYBOX;
  BoundaryBoxr *box = new BoundaryBoxr();
  box->boundingBox_.init(myParameters.extents_[0],
      myParameters.extents_[2],
      myParameters.extents_[4],
      myParameters.extents_[1],
      myParameters.extents_[3],
      myParameters.extents_[5]);
  box->calcValues();
  body->com_ = box->boundingBox_.getCenter();
  body->shape_ = box;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

//-------------------------------------------------------------------------------------------------------

void addboundary()
{
  //initialize the box shaped boundary
  myWorld.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_  = 0;
  body->volume_   = 0;
  body->invMass_     = 0;
  body->angle_    = VECTOR3(0,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_ = VECTOR3(0,0,0);
  body->shapeId_    = RigidBody::BOUNDARYBOX;
  BoundaryBoxr *box = new BoundaryBoxr();
  box->boundingBox_.init(xmin,ymin,zmin,xmax,ymax,zmax);
  box->calcValues();
  body->com_      = box->boundingBox_.getCenter();
  body->shape_      = box;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

//-------------------------------------------------------------------------------------------------------

void cleanup()
{
  std::vector<RigidBody*>::iterator vIter;
  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
  {
    RigidBody *body    = *vIter;
    delete body;
  }
  delete bdryParameterization;
}

//-------------------------------------------------------------------------------------------------------

void initphysicalparameters()
{

  std::vector<RigidBody*>::iterator vIter;

  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
  {
    RigidBody *body    = *vIter;
    body->density_    = myParameters.defaultDensity_;
    body->volume_     = body->shape_->getVolume();
    Real dmass          = body->density_ * body->volume_;
    body->invMass_    = 1.0/(body->density_ * body->volume_);
    body->angle_      = VECTOR3(0,0,0);
    body->setAngVel(VECTOR3(0,0,0));
    body->velocity_   = VECTOR3(0,0,0);
    body->com_        = VECTOR3(0,0,0);
    body->force_      = VECTOR3(0,0,0);
    body->torque_     = VECTOR3(0,0,0);
    body->restitution_ = 0.0;
    body->setOrientation(body->angle_);
    body->setTransformationMatrix(body->getQuaternion().GetMatrix());

    //calculate the inertia tensor
    //Get the inertia tensor
    body->generateInvInertiaTensor();
  }

}

//-------------------------------------------------------------------------------------------------------

void cupdynamics()
{
  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
  myWorld = myFactory.produceMesh("meshes/cup_small_high2.obj");
  Real extentBox[3]={0.25, 0.25, 0.025};
  myFactory.addBoxes(myWorld.rigidBodies_,1,extentBox);
  myFactory.addSpheres(myWorld.rigidBodies_,20,myParameters.defaultRadius_);

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.rigidBodies_[0]->translateTo(VECTOR3(0.49,0.25,0.378));
  myWorld.rigidBodies_[1]->translateTo(VECTOR3(0.75, 0.25, 0.28));
  myWorld.rigidBodies_[1]->affectedByGravity_=false;
  myWorld.rigidBodies_[1]->invMass_=0;
  myWorld.rigidBodies_[1]->invInertiaTensor_.SetZero();
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.rigidBodies_[0]->shape_);

  Model3D model_out(pMeshObject->m_Model);
  model_out.meshes_[0].transform_ =myWorld.rigidBodies_[0]->getTransformationMatrix();
  model_out.meshes_[0].com_ =myWorld.rigidBodies_[0]->com_;
  model_out.meshes_[0].TransformModelWorld();
  model_out.generateBoundingBox();
  model_out.meshes_[0].generateBoundingBox();
  std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.getBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();

  int offset = 2;
  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = 7;//myGrid.maxVertex_.x/(distbetween+d);
  //VECTOR3 pos(myGrid.minVertex_.x+drad+distbetween , myGrid.maxVertex_.y/2.0, (myGrid.maxVertex_.z/1.0)-d);
  Real xstart=myGrid.minVertex_.x + (myGrid.maxVertex_.x/2.5) - (drad+distbetween);
  Real ystart=myGrid.minVertex_.y+drad+distbetween+myGrid.maxVertex_.y/3.0;  
  VECTOR3 pos(xstart , ystart, (myGrid.maxVertex_.z/1.75)-d);
  //VECTOR3 pos(myGrid.maxVertex_.x-drad-distbetween , myGrid.maxVertex_.y/2.0, (myGrid.maxVertex_.z/1.5)-d);
  myWorld.rigidBodies_[offset]->translateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.0015;
  int count=0;
  for(int i=offset+1;i<myWorld.rigidBodies_.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = xstart;
      pos.y += d+distbetween;
      if(even)
      {
        ynoise = -ynoise;
      }
      if(++count==6)
      {
        pos.z -= d+distbetween;
        pos.y=ystart;
        count=0;
      }
    }
    VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
    myWorld.rigidBodies_[i]->translateTo(bodypos);
    pos.x+=d+distbetween;
    ynoise = -ynoise;
  }  

}

//-------------------------------------------------------------------------------------------------------

void createlineuptest()
{
  Real drad = radius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = 1.0/(distbetween+d);
  VECTOR3 pos(-0.5+drad+distbetween, 0.0, 3.0);
  myWorld.rigidBodies_[0]->translateTo(pos);
  distbetween = 0.5 * drad;
  pos.x+=d+distbetween;
  distbetween = 0.1 * drad;
  for(int i=1;i<myWorld.rigidBodies_.size();i++)
  {
    std::cout<<"position: "<<pos<<std::endl;    
    if((i+1)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = -0.5+drad+distbetween;
      pos.z -= d+distbetween;
    }
    myWorld.rigidBodies_[i]->translateTo(pos);
    pos.x+=d+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

void createstackingtest()
{

}

//-------------------------------------------------------------------------------------------------------

void pyramidtest()
{
  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
  myWorld = myFactory.produceBoxes(myParameters.bodies_, extends);

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  Real drad = extends[0];
  Real d    = 2.0 * drad;
  Real distbetween = drad * 0.05;
  Real delta = d+distbetween;

  Real ystart = 0.25;
  VECTOR3 pos(0.75, ystart, (1.0/3.0));
  int index = 0;
  for(int i=0;i<4;i++)
  {
    pos.y=ystart+Real(i)* (drad+distbetween/2.0);
    for(int j=i;j<4;j++)
    {
      myWorld.rigidBodies_[index]->translateTo(pos);
      pos.y+=delta;
      index++;
    }
    pos.z+=delta;
  }

  myWorld.rigidBodies_[index]->translateTo(VECTOR3(1.15,pos.y-delta,pos.z-2.5*delta));
  //myWorld.rigidBodies_[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  myWorld.rigidBodies_[index]->angle_=VECTOR3(0,1.75,0);
  //myWorld.rigidBodies_[index]->m_vAngle=VECTOR3(0,0.75,0);
  myWorld.rigidBodies_[index]->velocity_=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void createrestingtest()
{

  ParticleFactory myFactory;

  myWorld = myFactory.produceSpheres(myParameters.bodies_,myParameters.defaultRadius_);
  initphysicalparameters();
  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.5 * drad;
  int perrow = myGrid.maxVertex_.x/(distbetween+d);
  VECTOR3 pos(myGrid.minVertex_.x+drad+distbetween , myGrid.maxVertex_.y/2.0, (myGrid.minVertex_.z)+drad);
  myWorld.rigidBodies_[0]->translateTo(pos);
  pos.z+=d;//+distbetween;
  for(int i=1;i<myWorld.rigidBodies_.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.minVertex_.x+drad+distbetween;
      pos.z += d;
    }
    myWorld.rigidBodies_[i]->translateTo(pos);
    pos.z+=d;//+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void createbasf()
{

  ParticleFactory myFactory;
  myWorld = myFactory.produceTubes("meshes/myAllClumps.obj");
}

//-------------------------------------------------------------------------------------------------------

void addmesh()
{
  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};
  myWorld = myFactory.produceMesh("meshes/cup_small_high2.obj");

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.rigidBodies_[0]->translateTo(VECTOR3(0.25,0.25,0.8));
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.rigidBodies_[0]->shape_);

  Model3D model_out(pMeshObject->m_Model);
  model_out.meshes_[0].transform_ =myWorld.rigidBodies_[0]->getTransformationMatrix();
  model_out.meshes_[0].com_ =myWorld.rigidBodies_[0]->com_;
  model_out.meshes_[0].TransformModelWorld();
  model_out.generateBoundingBox();
  model_out.meshes_[0].generateBoundingBox();
  std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.getBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);

  //myWorld.rigidBodies_[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  //myWorld.rigidBodies_[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.rigidBodies_[index]->m_vAngle=VECTOR3(0,0.75,0);
  //myWorld.rigidBodies_[index]->m_vVelocity=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void addsphere_dt(int *itime)
{
  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  //VECTOR3 pos(myGrid.minVertex_.x+1.0*d, myGrid.maxVertex_.y/2.0, myGrid.maxVertex_.z/2.0);
  std::vector<VECTOR3> vPos;
  VECTOR3 pos(0.0,0.0,7.75);
  vPos.push_back(pos);
  pos = VECTOR3(0.17,0.0,7.75);
  vPos.push_back(pos);
  pos = VECTOR3(-0.17,0.0,7.75);
  vPos.push_back(pos);  
  pos = VECTOR3(0.,0.17,7.75);
  vPos.push_back(pos);  
  pos = VECTOR3(0.,-0.17,7.75);    
  vPos.push_back(pos);  


  int iadd = 5;
  int iStart = *itime;
  int iSeed = 1;

  //   if(iStart == 1)
  //     pos.y = -0.026 + drad + distbetween;
  //   else if(iStart == 2)
  //     pos.y = 0.026 - (drad + distbetween);


  Real noise = 0.0005;

  if(myWorld.rigidBodies_.size() < 1000)
  {
    ParticleFactory myFactory;

    int offset = myWorld.rigidBodies_.size();

    Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};

    myFactory.addSpheres(myWorld.rigidBodies_,iadd,myParameters.defaultRadius_);

    for(int i=0;i<iadd;i++)
    {
      RigidBody *body    = myWorld.rigidBodies_[offset+i];
      body->density_    = myParameters.defaultDensity_;
      body->volume_     = body->shape_->getVolume();
      Real dmass          = body->density_ * body->volume_;
      body->invMass_    = 1.0/(body->density_ * body->volume_);
      body->angle_      = VECTOR3(0,0,0);
      body->setAngVel(VECTOR3(0,0,0));
      body->velocity_   = VECTOR3(0,0,-1.05);
      body->com_        = VECTOR3(0,0,0);
      body->force_      = VECTOR3(0,0,0);
      body->torque_     = VECTOR3(0,0,0);
      body->restitution_ = 0.0;
      body->setOrientation(body->angle_);
      body->setTransformationMatrix(body->getQuaternion().GetMatrix());

      //calculate the inertia tensor
      //Get the inertia tensor
      body->generateInvInertiaTensor();
      pos = vPos[i];
      body->translateTo(pos);      
    }
  }//end if

  myPipeline.graph_->clear();

  //assign the rigid body ids
  for(int j=0;j<myWorld.rigidBodies_.size();j++)
    myWorld.rigidBodies_[j]->iID_ = j;

  std::cout<<"Added body, number of particles: "<<myWorld.rigidBodies_.size()<<std::endl;

}

//-------------------------------------------------------------------------------------------------------

void addobstacle()
{

  ObjLoader Loader;

  RigidBody *body = new RigidBody();
  CMeshObject<Real> *pMeshObject= new CMeshObject<Real>();

  Loader.readMultiMeshFromFile(&pMeshObject->m_Model,"meshes/fritten_final_mili.obj");

  pMeshObject->m_Model.generateBoundingBox();

  pMeshObject->SetFileName("meshes/fritten_final_mili.obj");

  body->shape_ = pMeshObject;
  body->shapeId_ = RigidBody::MESH;
  myWorld.rigidBodies_.push_back(body);

  //initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->affectedByGravity_ = false;
  body->density_  = 0;
  body->volume_   = 0;
  body->invMass_     = 0;
  body->angle_    = VECTOR3(3.14,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_ = VECTOR3(0,0,0);
  body->shapeId_    = RigidBody::MESH;

  body->com_      = VECTOR3(0,0,0);

  body->invInertiaTensor_.SetZero();

  body->restitution_ = 0.0;

  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
  body->translateTo(VECTOR3(0.13,0.2125,0.0155));

  Model3D model_out(pMeshObject->m_Model);
  model_out.generateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.meshes_.size();i++)
  {
    model_out.meshes_[i].transform_ =body->getTransformationMatrix();
    model_out.meshes_[i].com_ =body->com_;
    model_out.meshes_[i].TransformModelWorld();
    model_out.meshes_[i].generateBoundingBox();
  }

  std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.getBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();

}

//-------------------------------------------------------------------------------------------------------

void reactor()
{
  ParticleFactory myFactory;
  int offset = myWorld.rigidBodies_.size();
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};

  myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);

  RigidBody *body    = myWorld.rigidBodies_.back();
  body->density_    = myParameters.defaultDensity_;
  body->volume_     = body->shape_->getVolume();
  Real dmass          = body->density_ * body->volume_;
  body->invMass_    = 1.0/(body->density_ * body->volume_);
  body->angle_      = VECTOR3(0,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_   = VECTOR3(0,0,0);
  body->com_        = VECTOR3(0,0,0);
  body->force_      = VECTOR3(0,0,0);
  body->torque_     = VECTOR3(0,0,0);
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  VECTOR3 pos(0.25,0.3333,0.75);  

  body->translateTo(pos);

  body->velocity_=VECTOR3(0.0,0.0,0);  

  //addobstacle();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writeuniformgridlist()
{
}

//-------------------------------------------------------------------------------------------------------

inline float frand()
{
  return rand() / (float) RAND_MAX;
}

//-------------------------------------------------------------------------------------------------------

void SphereOfSpheres()
{

  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  //add the desired number of particles
  myFactory.addSpheres(myWorld.rigidBodies_,515,myParameters.defaultRadius_); //515
  initphysicalparameters();

  int r = 5, ballr = 5;
  // inject a sphere of particles
  float pr = myParameters.defaultRadius_;
  float tr = pr+(pr*2.0f)*ballr;
  float pos[4], vel[4];
  pos[0] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[1] = 1.0f - tr;
  pos[2] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[3] = 0.0f;
  //	vel[0] = vel[1] = vel[2] = vel[3] = 0.0f;

  float spacing = pr*2.0f;
  unsigned int index = 0;
  for(int z=-r; z<=r; z++) {
    for(int y=-r; y<=r; y++) {
      for(int x=-r; x<=r; x++) {
        float dx = x*spacing;
        float dy = y*spacing;
        float dz = z*spacing;
        float l = sqrtf(dx*dx + dy*dy + dz*dz);
        float jitter = myParameters.defaultRadius_*0.01f;
        if ((l <= myParameters.defaultRadius_*2.0f*r) && (index < myWorld.rigidBodies_.size())) {
          VECTOR3 position(pos[0] + dx + (frand()*2.0f-1.0f)*jitter,
              pos[1] + dy + (frand()*2.0f-1.0f)*jitter,
              pos[2] + dz + (frand()*2.0f-1.0f)*jitter);
          myWorld.rigidBodies_[index]->translateTo(position);
          myWorld.rigidBodies_[index]->color_ = position.x;
          index++;
        }
      }
    }
  }

}

//-------------------------------------------------------------------------------------------------------

void spherestack()
{

  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.25 * drad;
  Real distbetweenz = 0.5 * drad;
  int perrowx = myGrid.maxVertex_.x/(distbetween+d);
  int perrowy = myGrid.maxVertex_.y/(distbetween+d);  

  int numPerLayer = perrowx * perrowy;
  int layers =1;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
  initphysicalparameters();

  VECTOR3 pos(myGrid.minVertex_.x+drad+distbetween , myGrid.minVertex_.y+drad+distbetween+0.0025, (myGrid.maxVertex_.z-drad));

  Real ynoise = 0.0025;
  int count=0;

  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld.rigidBodies_[count]->translateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myGrid.minVertex_.x+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z-=d;
    pos.y=myGrid.minVertex_.y+drad+distbetween+0.0025;        
  }

}

//-------------------------------------------------------------------------------------------------------

void drivcav()
{

  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  Real myxmin =  0.0;  
  Real myymin =  0.0;  
  Real myzmin =  0.6;  

  Real myxmax = 1.0;  
  Real myymax = 0.25;  
  Real myzmax = 1.0;  


  Real drad  = myParameters.defaultRadius_;
  Real d     = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 1.0 * drad;
  Real distbetweenz = 0.5 * drad;

  Real extendX = myxmax - myxmin;  
  Real extendY = myymax - myymin;  
  Real extendZ = myzmax - myzmin;  

  int perrowx = extendX/(distbetween+d);
  int perrowy = extendY/(0.25*distbetween+d);  

  int numPerLayer = perrowx * perrowy;
  int layers = 8;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
  initphysicalparameters();

  VECTOR3 pos(myxmin+drad+distbetween , myymin+drad+0.25*distbetween+0.0025, (myzmin+drad));

  Real ynoise = 0.0025;
  int count=0;

  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld.rigidBodies_[count]->translateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myxmin+drad+distbetween;
      pos.y+=d+0.25*distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myymin+drad+0.25*distbetween+0.0025;        
  }

}

//-------------------------------------------------------------------------------------------------------

void sphericalstack()
{

  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.1 * drad;
  int perrowx = 1.0/(distbetween+d);
  int perrowy = perrowx-1;
  Real z=7.7;
  Real x=-0.5+drad+distbetween;
  Real y=-0.5+drad+distbetween;
  int layers = 50;
  std::vector<VECTOR3> vPos;

  for(int layer=0;layer<layers;layer++)
  {
    double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand()/(double)RAND_MAX);
    // make an x-row and rotate
    for(int i=0;i<perrowx;i++)
    {
      VECTOR3 pos(x,0,0);
      MATRIX3X3 rotmat;
      rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
      pos = rotmat * pos;
      pos.z=z;
      //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
      vPos.push_back(pos);
      x+=d+distbetween;
    }
    //yrow
    for(int i=0;i<perrowx-1;i++)
    {
      VECTOR3 pos(0,y,0);
      MATRIX3X3 rotmat;
      rotmat.MatrixFromAngles(VECTOR3(0,0,radian));
      pos = rotmat * pos;
      pos.z=z;
      //pos = VECTOR3(x+fabs(x)*cos(radian),fabs(x)*sin(radian),z);
      vPos.push_back(pos);
      if(i==3)
        y=(d+distbetween);
      else
        y+=d+distbetween;
    }
    y=-0.5+drad+distbetween;
    x=-0.5+drad+distbetween;
    z-=d+1.5*distbetween;
  }

  int numPerLayer = 2*perrowx - 1;
  myFactory.addSpheres(myWorld.rigidBodies_,numPerLayer*layers,myParameters.defaultRadius_);  
  initphysicalparameters();

  std::vector<RigidBody*>::iterator vIter;
  std::vector<VECTOR3>::iterator i;

  for(vIter=myWorld.rigidBodies_.begin(),i=vPos.begin();vIter!=myWorld.rigidBodies_.end();vIter++,i++)
  {
    RigidBody *body    = *vIter;
    VECTOR3 pos         = *i;
    body->translateTo(pos);
  }

}

//-------------------------------------------------------------------------------------------------------

void particlesinbox()
{

  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  Real myxmin =  0.0;  
  Real myymin =  0.0;  
  Real myzmin =  0.6;  

  Real myxmax = 1.0;  
  Real myymax = 0.25;  
  Real myzmax = 1.0;  


  Real drad  = myParameters.defaultRadius_;
  Real d     = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.25 * drad;
  Real distbetweenz = 0.5 * drad;

  Real extendX = myxmax - myxmin;  
  Real extendY = myymax - myymin;  
  Real extendZ = myzmax - myzmin;  

  int perrowx = extendX/(distbetween+d);
  int perrowy = extendY/(distbetween+d);  

  int layers = 7;
  int nTotal = 0;

  //define 2 planes in point-normal form

  VECTOR3 normal1(0.7071,0.0,0.7071);
  VECTOR3 normal2(-0.7071,0.0,0.7071);

  VECTOR3 origin1(0.2,0.125,0.8);
  VECTOR3 origin2(0.8,0.125,0.8);

  VECTOR3 pos(myxmin+drad+distbetween , myymin+drad+distbetween+0.0025, (myzmin+drad));

  Real ynoise = 0.0025;

  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++)
      {
        //one row in x
        //check if position is valid
        if((normal1 * (pos-origin1) > (d+distbetween)) && (normal2 * (pos-origin2) > (d+distbetween)))
        {
          nTotal++;
        }
        //nTotal++
        pos.x+=d+distbetween;
      }
      pos.x=myxmin+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myymin+drad+distbetween+0.0025;        
  }

  std::cout<<"Number particles: "<<nTotal<<std::endl;

  //add the desired number of particles
  myFactory.addSpheres(myWorld.rigidBodies_,nTotal,myParameters.defaultRadius_);  
  initphysicalparameters();

  pos=VECTOR3(myxmin+drad+distbetween , myymin+drad+distbetween+0.0025, (myzmin+drad));

  int count=0;

  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++)
      {
        //one row in x
        if((normal1 * (pos-origin1) > (d+distbetween)) && (normal2 * (pos-origin2) > (d+distbetween)))
        {
          VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
          myWorld.rigidBodies_[count]->translateTo(bodypos);
          count++;
        }
        pos.x+=d+distbetween;
      }
      pos.x=myxmin+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myymin+drad+distbetween+0.0025;        
  }

}


//-------------------------------------------------------------------------------------------------------

float randFloat(float LO, float HI)
{
  return (LO + (float)rand()/((float)RAND_MAX/(HI-LO)));
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getrandfloat(double point[])
{

  VECTOR3 bodypos = VECTOR3(randFloat(xmin,xmax),randFloat(ymin,ymax),randFloat(zmin,zmax));

  point[0] = bodypos.x;
  point[1] = bodypos.y;
  point[2] = bodypos.z;

}

//-------------------------------------------------------------------------------------------------------

void initrandompositions()
{
  ParticleFactory myFactory;
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,2.0*myParameters.defaultRadius_};

  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;

  VECTOR3 vMin = myGrid.getAABB().vertices_[0];
  VECTOR3 vMax = myGrid.getAABB().vertices_[1];

  //add the desired number of particles
  myFactory.addSpheres(myWorld.rigidBodies_,nTotal,drad);
  std::cout<<"Number of spheres: "<<nTotal<<std::endl;
  initphysicalparameters();
  VECTOR3 pos(0,0,0);

  int count=0;    
  for(int i=0;i<nTotal;i++)
  {

    Real randz=randFloat(drad,15.0-drad);

    Real randx=randFloat(drad,8.0-drad);
    randx-=4.0;

    //get a random float so that the distance to the
    //sides of the cylinder is less than the radius
    Real randy=randFloat(drad,8.0-drad);   
    randy-=4.0;

    bool valid=false;
    while( (randx*randx) + (randy*randy) >= (4.0-drad)*(4.0-drad) || !valid)
    {

      randx=randFloat(drad,8.0-drad);
      randy=randFloat(drad,8.0-drad);   
      randy-=4.0;
      randx-=4.0;
      VECTOR3 mypos = VECTOR3(randx,randy,randz);
      valid=true;
      for(int j=0;j<nTotal;j++)
      {
        if((mypos - myWorld.rigidBodies_[j]->com_).mag() <= 2.0 * drad)
        {
          valid=false;
          break;
        }
      }
    }        
    //one row in x
    VECTOR3 bodypos = VECTOR3(randx,randy,randz);
    myWorld.rigidBodies_[i]->translateTo(bodypos);

  }

}

//-------------------------------------------------------------------------------------------------------

void initrigidbodies()
{
  ParticleFactory myFactory;

  if(myParameters.bodyInit_ == 0)
  {
    myWorld = myFactory.produceFromParameters(myParameters);
  }
  else if(myParameters.bodyInit_ == 1)
  {
    myWorld = myFactory.produceFromFile(myParameters.bodyConfigurationFile_.c_str(),myTimeControl);
  }
  else
  {
    if(myParameters.bodyInit_ == 2)
    {
      initrandompositions();
    }

    if(myParameters.bodyInit_ == 3)
    {
      createlineuptest();
    }

    if(myParameters.bodyInit_ == 4)
    {
      reactor();
    }
    if(myParameters.bodyInit_ == 5)
    {
      createbasf();
    }

    if(myParameters.bodyInit_ == 6)
    {
      spherestack();
    }
    if(myParameters.bodyInit_ == 7)
    {
      //drivcav();
      particlesinbox();
      myFactory.addFromDataFile(myParameters, &myWorld);
    }

    if(myParameters.bodyInit_ == 8)
    {
      initrandompositions();
    }

  }

}

#ifdef FC_CUDA_SUPPORT
// initialize particle system
void initParticleSystem(int numParticles, uint3 gridSize)
{

  myWorld.psystem = new ParticleSystem(numParticles, gridSize, true);

  float *hPos = new float[numParticles*4];
  float *hVel = new float[numParticles*4];

  for(int i=0;i<numParticles;i++)
  {
    hPos[i*4]   = myWorld.rigidBodies_[i]->com_.x; 
    hPos[i * 4 + 1] = myWorld.rigidBodies_[i]->com_.y;
    hPos[i * 4 + 2] = myWorld.rigidBodies_[i]->com_.z;
    hPos[i*4+3] = 1.0;

    hVel[i*4]   = 0.0f;
    hVel[i*4+1] = 0.0f;
    hVel[i*4+2] = 0.0f;
    hVel[i*4+3] = 0.0f;
  }

  //create the particle configuration
  myWorld.psystem->setParticles(hPos,hVel);

  // simulation parameters
  //float timestep = 0.5f;
  //float damping = 1.0f;
  //float gravity = 0.0003f;
  //int iterations = 1;
  //int ballr = 10;
  //
  //float collideSpring = 0.5f;;
  //float collideDamping = 0.02f;;
  //float collideShear = 0.1f;
  //float collideAttraction = 0.0f;
  //
  //ParticleSystem *psystem = 0;

  myWorld.psystem->setIterations(1);
  myWorld.psystem->setDamping(1.0f);
  myWorld.psystem->setGravity(-9.81f);
  myWorld.psystem->setCollideSpring(2.75f);
  myWorld.psystem->setCollideDamping(0.02f);
  myWorld.psystem->setCollideShear(0.1f);
  myWorld.psystem->setCollideAttraction(0.0f);

  delete[] hPos;
  delete[] hVel;
}
#endif

void configureRigidBodies()
{

  ParticleFactory factory(myWorld, myParameters);
  myWorld.solverType_ = myParameters.solverType_;

}

void initsimulation()
{

  //first of all initialize the rigid bodies
  int id = myWorld.parInfo_.getId();
  configureRigidBodies();
  myWorld.parInfo_.setId(id);

  //initialize the box shaped boundary
  myBoundary.boundingBox_.init(myParameters.extents_[0],
      myParameters.extents_[2],
      myParameters.extents_[4],
      myParameters.extents_[1],
      myParameters.extents_[3],
      myParameters.extents_[5]);
  myBoundary.calcValues();

  //add the boundary as a rigid body
  //addcylinderboundary();
  configureBoundary();

  //assign the rigid body ids
  for(int j=0;j<myWorld.rigidBodies_.size();j++)
  {
    myWorld.rigidBodies_[j]->iID_ = j;
    myWorld.rigidBodies_[j]->elementsPrev_ = 0;
  }

  //Distance map initialization
  std::set<std::string> fileNames;

  for (auto &body : myWorld.rigidBodies_)
  {

    if (body->shapeId_ != RigidBody::MESH)
      continue;

    CMeshObjectr *meshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
    std::string objName = meshObject->GetFileName();
    fileNames.insert(objName);
  }

  int iHandle=0;
  for (auto const &myName : fileNames)
  {
    bool created = false;
    for (auto &body : myWorld.rigidBodies_)
    {
      if (body->shapeId_ != RigidBody::MESH)
        continue;

      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);

      std::string objName = pMeshObject->GetFileName();
      if (objName == myName)
      {
        if (created)
        {
          //if map created -> add reference
          body->map_ = myWorld.maps_.back();
        }
        else
        {
          //if map not created -> create and add reference
          body->buildDistanceMap();
          myWorld.maps_.push_back(body->map_);
          created = true;
          CVtkWriter writer;
          std::string n = myName;
          const size_t last = n.find_last_of("\\/");
          if(std::string::npos != last)
          {
            n.erase(0,last);
          }
          const size_t period = n.rfind(".");
          if(std::string::npos != period)
          {
            n.erase(period);
          }
          n.append(".ps");
          std::string dir("output/");
          dir.append(n);
        }
      }
    }
  }

  std::cout << myWorld.maps_.size() << std::endl;

  //set the timestep
  myTimeControl.SetDeltaT(myParameters.timeStep_);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

  //link the boundary to the world
  myWorld.setBoundary(&myBoundary);

  //set the time control
  myWorld.setTimeControl(&myTimeControl);

  //set the gravity
  myWorld.setGravity(myParameters.gravity_);

  //Set the collision epsilon
  myPipeline.setEPS(0.02);

  if(myWorld.parInfo_.getId() == 0)
  {
    std::cout  << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> No. rigid bodies: " <<
      termcolor::reset << myWorld.rigidBodies_.size()  << std::endl;
  }

  //initialize the collision pipeline 
  myPipeline.init(&myWorld,myParameters.solverType_,myParameters.maxIterations_,myParameters.pipelineIterations_);

  //set the broad phase to simple spatialhashing
  myPipeline.setBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  if(myParameters.solverType_==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new MotionIntegratorSI(&myWorld);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new RigidBodyMotion(&myWorld);
  }

  //set the integrator in the pipeline
  myPipeline.integrator_ = myMotion;

  myWorld.densityMedium_ = myParameters.densityMedium_;

  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;

  myPipeline.response_->m_pGraph = myPipeline.graph_;  

}

void continuesimulation()
{

  ParticleFactory myFactory;

  //Produces a domain
  //it is a bit unsafe, because the domain at this point is
  //not fully useable, because of non initialized values in it
  //string = ssolution
  myWorld = myFactory.produceFromFile(myParameters.solutionFile_.c_str(),myTimeControl);

  //initialize the box shaped boundary
  myBoundary.boundingBox_.init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.calcValues();

  //add the boundary as a rigid body
  addcylinderboundary();

  //assign the rigid body ids
  for(int j=0;j<myWorld.rigidBodies_.size();j++)
  {
    myWorld.rigidBodies_[j]->iID_ = j;
    myWorld.rigidBodies_[j]->elementsPrev_ = 0;
  }

  //set the timestep
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myParameters.nTimesteps_+=myTimeControl.GetTimeStep();

  //link the boundary to the world
  myWorld.setBoundary(&myBoundary);

  //set the time control
  myWorld.setTimeControl(&myTimeControl);

  //set the gravity
  myWorld.setGravity(myParameters.gravity_);

  //Set the collision epsilon
  myPipeline.setEPS(0.02);

  //initialize the collision pipeline 
  myPipeline.init(&myWorld,myParameters.solverType_,myParameters.maxIterations_,myParameters.pipelineIterations_);

  //set the broad phase to simple spatialhashing
  myPipeline.setBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  if(myParameters.solverType_==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new MotionIntegratorSI(&myWorld);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new RigidBodyMotion(&myWorld);
  }

  //set the integrator in the pipeline
  myPipeline.integrator_ = myMotion;

  myWorld.densityMedium_ = myParameters.densityMedium_;

  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;

  myPipeline.response_->m_pGraph = myPipeline.graph_;  

}

void writetimestep(int iout)
{
  std::ostringstream sName,sNameParticles,sContacts;
  std::string sModel("output/model.vtk");
  std::string sParticle("solution/particles.i3d");
  CVtkWriter writer;
  int iTimestep=iout;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sNameParticles<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sModel.append(sName.str());
  sParticle.append(sNameParticles.str());
  sContacts<<"output/contacts.vtk."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  //Write the grid to a file and measure the time
  //writer.WriteRigidBodies(myWorld.rigidBodies_,sModel.c_str());
  writer.WriteParticleFile(myWorld.rigidBodies_,sModel.c_str());
  RigidBodyIO rbwriter;
  myWorld.output_ = iTimestep;
  std::vector<int> indices;
  //indices.push_back(8);
  //indices.push_back(10);
  //indices.push_back(11);
  //rbwriter.Write(myWorld,indices,sParticle.c_str());
  //rbwriter.Write(myWorld,sParticle.c_str());
  //writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

  // if(iout==0)
  // {
  //   std::ostringstream sNameGrid;
  //   std::string sGrid("output/grid.vtk");
  //   sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  //   sGrid.append(sNameGrid.str());
  //   writer.WriteUnstr(myGrid,sGrid.c_str());
  // }
}

extern "C" void bndryproj(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz)
{
  RigidBody *body = bdryParameterization;  
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
  Real x=*dx;
  Real y=*dy;
  Real z=*dz;
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,VECTOR3(x,y,z));
  Real ddist = distMeshPoint.ComputeDistance();
  *dxx=distMeshPoint.m_Res.m_vClosestPoint.x;
  *dyy=distMeshPoint.m_Res.m_vClosestPoint.y;
  *dzz=distMeshPoint.m_Res.m_vClosestPoint.z;   
}

extern "C" void bndryprojid(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz,int *id)
{
  int bdryId = *id;
  bool found=false;
  for(int i=0;i<bdryParams.size();i++)
  {
    if(bdryParams[i]->iID_==bdryId)
    {
      RigidBody *body = bdryParams[i];
      if(body->shapeId_==RigidBody::MESH)
      {
        CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
        Real x=*dx;
        Real y=*dy;
        Real z=*dz;
        CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,VECTOR3(x,y,z));
        Real ddist = distMeshPoint.ComputeDistance();
        *dxx=distMeshPoint.m_Res.m_vClosestPoint.x;
        *dyy=distMeshPoint.m_Res.m_vClosestPoint.y;
        *dzz=distMeshPoint.m_Res.m_vClosestPoint.z; 
        found=true;
        break;
      }
      else if(body->shapeId_==RigidBody::PLINE)
      {
        ParamLiner *pLine = dynamic_cast<ParamLiner *>(body->shape_);
        Real x=*dx;
        Real y=*dy;
        Real z=*dz;
        CDistancePointPline<Real> distPointLine(VECTOR3(x,y,z),*pLine);
        Real ddist = distPointLine.ComputeDistance();
        *dxx=distPointLine.m_vClosestPoint1.x;
        *dyy=distPointLine.m_vClosestPoint1.y;
        *dzz=distPointLine.m_vClosestPoint1.z;
        found=true;
        break;
      }
    }
  }
  if(!found)
  {
    printf("Bndryprojid with ibnds %i failed because no matching parametrization object was found.\n",bdryId);
  }
}

#include <init_func.hpp>

#endif
