// extern "C" void initsimulation();
// extern "C" void initterminalvelbench();
// extern "C" void initdkt();
// extern "C" void initbasf();
// 
// 
// extern "C" void startcollisionpipeline();
// extern "C" void clearcollisionpipeline();
// extern "C" void logdistance();
// extern "C" void logposition();
// extern "C" void logvelocity();
// extern "C" void logangularvelocity();
// 
// extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin);
// extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin);
// 
// extern "C" void setdensity(double *ddens);
// extern "C" void setposition(double *dx,double *dy,double *dz);
// extern "C" void setrotation(double *dx,double *dy,double *dz);
// extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID);
// extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
// extern "C" void settimestep(double *dTime);
// extern "C" void settime(double *dTime);
// 
// extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime);
// extern "C" void writepvtu(int *iNodes,int *iTime);
// extern "C" void writeparticles(int *iout);
// extern "C" void writepolygon(int *iout);
// 
// extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist);
// extern "C" void getnumparticles(int *nParts);
// extern "C" void getradius(double *drad, int *iid);
// extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid);
// extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
// extern "C" void getpos(double *dx,double *dy,double *dz,int *iID);
// extern "C" void getvel(double *dx,double *dy,double *dz,int *iID);
// extern "C" void getdensity(double *ddens, int *iid);
//

#include <cppinterface.h>
#include <string>
#include <aabb3.h>
#include <iostream>
#include <genericloader.h>
#include <unstructuredgrid.h>
#include <distops3.h>
#include <triangulator.h>
#include <iomanip>
#include <sstream>
#include <intersectorray3tri3.h>
#include <vtkwriter.h>
#include <world.h>
#include <particlefactory.h>
#include <collisionpipeline.h>
//#include <mymath.h> asdf
#include <distancetriangle.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <log.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <reader.h>
#include <deformparameters.h>
#include <objloader.h>
#include <hspatialhash.h>
#include <broadphasestrategy.h>
#include <distancemeshpoint.h>
#include <distancetriangle.h>
#include <intersector2aabb.h>
#include <perftimer.h>
#include <motionintegratorsi.h>
#include <collisionpipelinegpu.h>
#include <uniformgrid.h>
#include <huniformgrid.h>
#include <boundarycyl.h>

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

using namespace i3d;
#define GRID_SIZE       16

#ifdef FC_CUDA_SUPPORT
uint3 gridSize;
#endif


extern "C" void cudaGLInit(int argc, char **argv);

struct funcx
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.x < elem2.vVec.x;
  }
};

struct funcy
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.y < elem2.vVec.y;
  }
};  

struct funcz
{
  bool operator()(const sPerm &elem1,const sPerm &elem2 ) 
  {
  return elem1.vVec.z < elem2.vVec.z;
  }
};  

Real a = CMath<Real>::MAXREAL;
CUnstrGrid myGrid;
CWorld myWorld;
#ifdef FC_CUDA_SUPPORT
CCollisionPipelineGPU myPipeline;
#else
CCollisionPipeline myPipeline;
#endif
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
CTimeControl myTimeControl;
CWorldParameters myParameters;
CDeformParameters myDeformParameters;
CPerfTimer myTimer;
CRigidBodyMotion *myMotion;
CDistanceMeshPointResult<Real> resMaxM1;
CDistanceMeshPointResult<Real> resMax0;
CDistanceMeshPointResult<Real> *resCurrent;
CHUniformGrid<Real,CUGCell> myUniformGrid;
std::vector<CRigidBody *> bdryParams;
CRigidBody *bdryParameterization;
std::list<int> g_iElements;

unsigned int processID;

#ifdef FEATFLOWLIB
extern "C" void communicateforce_(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz);
#endif

int nTotal = 300;
double xmin=-0.5;
double ymin=-0.5;
double zmin= 0;
double xmax= 0.5f;
double ymax= 0.5f;
double zmax= 4.0f;
Real radius = Real(0.075);
int iReadGridFromFile = 0;
const unsigned int width = 640, height = 480;

C3DModel Model;
CLog mylog;
CLog myPositionlog;
CLog myVelocitylog;
CLog myAngVelocitylog;
CLog myCollisionlog;
CAABB3r boxDomain;

#ifdef FEATFLOWLIB
extern "C" void velocityupdate()
{

  double *ForceX = new double[myWorld.m_vRigidBodies.size()];
  double *ForceY = new double[myWorld.m_vRigidBodies.size()];
  double *ForceZ = new double[myWorld.m_vRigidBodies.size()];
  double *TorqueX = new double[myWorld.m_vRigidBodies.size()];
  double *TorqueY = new double[myWorld.m_vRigidBodies.size()];
  double *TorqueZ = new double[myWorld.m_vRigidBodies.size()];
  
  //get the forces from the cfd-solver
  communicateforce_(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
  
  std::vector<VECTOR3> vForce;
  std::vector<VECTOR3> vTorque;  
  
  std::vector<CRigidBody*>::iterator vIter;  
  int count = 0;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++,count++)
  {
    CRigidBody *body    = *vIter;
    vForce.push_back(VECTOR3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(VECTOR3(TorqueX[count],TorqueY[count],TorqueZ[count]));
  }

  //calculate the forces in the current timestep by a semi-implicit scheme
  myPipeline.m_pIntegrator->UpdateForces(vForce,vTorque);

  delete[] ForceX;
  delete[] ForceY;
  delete[] ForceZ;
  delete[] TorqueX;
  delete[] TorqueY;
  delete[] TorqueZ;      
  
}
#endif

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
  //initialize the box shaped boundary
  myWorld.m_vRigidBodies.push_back(new CRigidBody());
  CRigidBody *body = myWorld.m_vRigidBodies.back();
  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::BOUNDARYBOX;
  CBoundaryCylr *cyl = new CBoundaryCylr();
  cyl->rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  cyl->CalcValues();
  cyl->SetBoundaryType(CBoundaryBoxr::CYLBDRY);
  cyl->m_Cylinder = CCylinderr(VECTOR3(0.0,0.0,10.0),VECTOR3(0.0,0.0,1.0),4.0,10.0);
  body->m_vCOM      = cyl->rBox.GetCenter();
  body->m_pShape      = cyl;
  body->m_InvInertiaTensor.SetZero();
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
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

 CAABB3r box(VECTOR3(minx,miny,minz),VECTOR3(maxx,maxy,maxz)); 
 int i = *iid;
 CRigidBody *pBody  = myWorld.m_vRigidBodies[i];
 CShaper *pShape    = pBody->GetWorldTransformedShape();
 CAABB3r boxBody    = pShape->GetAABB();
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

extern "C" void updateelementsprev(int *ibody)
{
  int i  = *ibody; 
  int e1 = myWorld.m_vRigidBodies[i]->m_iElements.size();
  int e2 = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
  int itotal = e1+e2;
  myWorld.m_vRigidBodies[i]->m_iElementsPrev = itotal;

}

//-------------------------------------------------------------------------------------------------------

extern "C" void setdomainbox(double vmin[3], double vmax[3])
{
  boxDomain = CAABB3r(VECTOR3(vmin[0],vmin[1],vmin[2]),VECTOR3(vmax[0],vmax[1],vmax[2]));
}

//-------------------------------------------------------------------------------------------------------

extern "C" void elementsize(double element[][3], double *size)
{
  
  VECTOR3 elementMin(element[0][0],element[0][1],element[0][2]);
  VECTOR3 elementMax(element[0][0],element[0][1],element[0][2]);  
  
  for(int i=1;i<8;i++)
  {
    if(elementMin.x > element[i][0])
      elementMin.x = element[i][0];

    if(elementMin.y > element[i][1])
      elementMin.y = element[i][1];
    
    if(elementMin.z > element[i][2])
      elementMin.z = element[i][2];    
    
    if(elementMax.x < element[i][0])
      elementMax.x = element[i][0];

    if(elementMax.y < element[i][1])
      elementMax.y = element[i][1];
    
    if(elementMax.z < element[i][2])
      elementMax.z = element[i][2];            
  }
  
  CAABB3r gridElement = CAABB3r(elementMin,elementMax);

  //printf("extends %f %f %f \n",gridElement.m_Extends[0],gridElement.m_Extends[1],gridElement.m_Extends[2]); 

  *size = gridElement.GetBoundingSphereRadius();
  
}

//-------------------------------------------------------------------------------------------------------

struct sortSizes {
  bool operator()(const std::pair<Real,int> &a, const std::pair<Real,int> &b)
  {
    return a.first < b.first;
  }
};

extern "C" void setelementarray(double elementsize[], int *iel)
{

  int isize = *iel;

  std::list< std::pair<Real,int> > sizes;

  for(int i=0;i<isize;i++)
    {
    sizes.push_back( std::pair<Real,int>(elementsize[i],i+1));
    }

  sizes.sort(sortSizes());
  std::vector<int> vDistribution;
  std::vector<Real> vGridSizes;
  double factor = 1.75;
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();

  double tsize = factor * ((*liter).first);
  liter++;
  int levels=0;
  int elemPerLevel=1;
  double lastsize=0.0;
  double dsize=0.0;
  for(;liter!=sizes.end();liter++)
    {
      dsize=((*liter).first);
      if(dsize > tsize)
	{
	  vGridSizes.push_back(lastsize);
	  tsize=factor*dsize;
          lastsize=dsize;
	  vDistribution.push_back(elemPerLevel);
	  elemPerLevel=1;

	}
      else
	{
          lastsize=dsize;
          elemPerLevel++;
	}
    }

  vGridSizes.push_back(lastsize);
  vDistribution.push_back(elemPerLevel);

  levels=vDistribution.size();

  int totalElements=0;
  for(int j=0;j<vDistribution.size();j++)
    {
      //      std::cout<<vDistribution[j]<< " elements on level: "<<j+1<<"\n";
      totalElements+=vDistribution[j];
    }

  CAABB3r boundingBox = boxDomain;
  
  myUniformGrid.InitGrid(boundingBox,levels);

  for(int j=0;j<vGridSizes.size();j++)
    {
      std::cout<<"Building level: "<<j+1<<" size: "<<vGridSizes[j]<<"\n";
      myUniformGrid.InitGridLevel(j,2.0*vGridSizes[j]);
    }

  std::cout<<"Total elements = "<<totalElements<<" = "<<isize<<"\n";

  CVtkWriter writer;
  for(int j=0;j<levels;j++)
  {
    std::ostringstream sGrid;
    std::string sNameGrid("_vtk/uniform_level");
    sGrid<<"."<<std::setfill('0')<<std::setw(2)<<j<<".node."<<std::setfill('0')<<std::setw(2)<<myWorld.m_myParInfo.GetID()<<".vtk";
    sNameGrid.append(sGrid.str());

    //Write the grid to a file and measure the time
    writer.WriteUniformGrid(myUniformGrid.m_pLevels[j],sNameGrid.c_str());
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void setelementarray2(double elementsize[], int *iel)
{

  int isize = *iel;

  std::list< std::pair<Real,int> > sizes;

  for(int i=0;i<isize;i++)
    {
    sizes.push_back( std::pair<Real,int>(elementsize[i],i+1));
    }

  sizes.sort(sortSizes());
  std::vector<int> vDistribution;
  std::vector<Real> vGridSizes;
  double factor = 1.75;
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();

  double tsize = factor * ((*liter).first);
  liter++;
  int levels=0;
  int elemPerLevel=1;
  double lastsize=0.0;
  double dsize=0.0;
  for(;liter!=sizes.end();liter++)
    {
      dsize=((*liter).first);
      if(dsize > tsize)
	{
	  vGridSizes.push_back(lastsize);
	  tsize=factor*dsize;
          lastsize=dsize;
	  vDistribution.push_back(elemPerLevel);
	  elemPerLevel=1;

	}
      else
	{
          lastsize=dsize;
          elemPerLevel++;
	}
    }

  vGridSizes.push_back(lastsize);
  vDistribution.push_back(elemPerLevel);

  levels=vDistribution.size();

  int totalElements=0;
  for(int j=0;j<vDistribution.size();j++)
    {
      //      std::cout<<vDistribution[j]<< " elements on level: "<<j+1<<"\n";
      totalElements+=vDistribution[j];
    }

  CAABB3r boundingBox = boxDomain;
  
  myUniformGrid.InitGrid(boundingBox,levels);

  for(int j=0;j<vGridSizes.size();j++)
    {
      std::cout<<"Building level: "<<j+1<<" size: "<<vGridSizes[j]<<"\n";
      myUniformGrid.InitGridLevel(j,2.0*vGridSizes[j]);
    }

  std::cout<<"Total elements = "<<totalElements<<" = "<<isize<<"\n";

  CVtkWriter writer;
  for(int j=0;j<levels;j++)
  {
    std::ostringstream sGrid;
    std::string sNameGrid("_vtk/uniform_level");
    sGrid<<"."<<std::setfill('0')<<std::setw(2)<<j<<".node."<<std::setfill('0')<<std::setw(2)<<myWorld.m_myParInfo.GetID()<<".vtk";
    sNameGrid.append(sGrid.str());

    //Write the grid to a file and measure the time
    writer.WriteUniformGrid(myUniformGrid.m_pLevels[j],sNameGrid.c_str());
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_querystatus()
{

  for(int j=0;j<myUniformGrid.m_iLevels;j++)
  {
    std::cout<<"Level: "<<j+1<<" Element check: "<<myUniformGrid.m_pLevels[j].GetNumEntries()<<"\n";
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_resetuniformgrid()
{
  myUniformGrid.Reset();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_insertelement(int *iel, double center[3], double *size)
{

  int myiel = *iel;
  VECTOR3 ele(center[0],center[1],center[2]);
  Real mysize = *size;

  myUniformGrid.InsertElement(myiel,ele,2.0*mysize);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_pointquery(double center[3], int *iiel)
{
  
  g_iElements.clear();
  VECTOR3 q(center[0],center[1],center[2]);  
  myUniformGrid.PointQuery(q,g_iElements);
  *iiel=g_iElements.size();
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_getelements(int ielem[])
{
  
  std::list<int>::iterator i = g_iElements.begin();
  for(int j=0;i!=g_iElements.end();i++,j++)
  {
    ielem[j]=(*i);
  }
  g_iElements.clear();
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void inituniformgrid(double vmin[3], double vmax[3], double element[][3])
{
  CAABB3r boundingBox = CAABB3r(VECTOR3(vmin[0],vmin[1],vmin[2]),VECTOR3(vmax[0],vmax[1],vmax[2]));
  
  VECTOR3 elementMin(element[0][0],element[0][1],element[0][2]);
  VECTOR3 elementMax(element[0][0],element[0][1],element[0][2]);  
  
  for(int i=1;i<8;i++)
  {
    if(elementMin.x > element[i][0])
      elementMin.x = element[i][0];

    if(elementMin.y > element[i][1])
      elementMin.y = element[i][1];
    
    if(elementMin.z > element[i][2])
      elementMin.z = element[i][2];    
    
    if(elementMax.x < element[i][0])
      elementMax.x = element[i][0];

    if(elementMax.y < element[i][1])
      elementMax.y = element[i][1];
    
    if(elementMax.z < element[i][2])
      elementMax.z = element[i][2];            
  }
  
  CAABB3r gridElement = CAABB3r(elementMin,elementMax);
  //myUniformGrid.InitGrid(boundingBox,gridElement);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void uniformgridinsert(int *iel, double center[3])
{
  int elem = *iel;
  //myUniformGrid.Insert(elem,VECTOR3(center[0],center[1],center[2]));    
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setmyid(int *myid)
{
  int id = *myid;
  myWorld.m_myParInfo.SetID(id);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void intersectdomainbody(int *ibody,int *domain,int *intersection)
{
  // call the intersector for the bounding box of the body and 
  // and the domain bounding box
  int i = *ibody;
  CRigidBody *pBody  = myWorld.m_vRigidBodies[i];
  CAABB3r boxBody    = pBody->GetAABB();
  CIntersector2AABB<Real> intersector(boxDomain,boxBody);
  bool bIntersect =  intersector.Intersection();
  if(bIntersect)
    *intersection=1;
  else
    *intersection=0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void vertexorderxyz(int *invt,int iorder[], double dcorvg[][3])
{
  int nvt = *invt;
  std::vector<sPerm> permArray;
  
  for(int i=0;i<nvt;i++)
  { 
    sPerm perm;
    Real x = dcorvg[0][i];
    Real y = dcorvg[1][i];
    Real z = dcorvg[2][i];
    CVector3<Real> vec(x,y,z);        
    perm.vVec =  vec;
    perm.index  = i;
    permArray.push_back(perm);
  }
  
  std::stable_sort(permArray.begin(),permArray.end(),funcz());
  std::stable_sort(permArray.begin(),permArray.end(),funcy());
  std::stable_sort(permArray.begin(),permArray.end(),funcx());
  
  for(int i=0;i<nvt;i++)
  {
    iorder[i]=permArray[i].index;
  }  
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void updateMax0(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  
  // compute distance point triangle
  int k = resMaxM1.iTriangleID;
  CTriangle3<Real> &tri3 = resMaxM1.pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();  
  
  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent           = &resMax0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setMaxM1(double *dx,double *dy,double *dz,double *dist)
{ 
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);  
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  
  resMaxM1.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMaxM1.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMaxM1.pNode       = distMeshPoint.m_Res.pNode;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setstartbb(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);  
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  
  
  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent           = &resMax0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistancebbid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  
  // compute the distance to the triangle found for the reference point
  int k = resCurrent->iTriangleID;
  CTriangle3<Real> &tri3 = resCurrent->pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();    
  
  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void clearelementlists(int *ibody)
{
  int i = *ibody;
  myWorld.m_vRigidBodies[i]->m_iElements.clear();
  myWorld.m_vRigidBodies[i]->m_iBoundaryElements.clear();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void checkuniformgrid(int *ibody)
{
  int i = *ibody;
  myWorld.m_vRigidBodies[i]->m_iElements.clear();
  
}

//-------------------------------------------------------------------------------------------------------

void addelement2list(int *iel, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  myWorld.m_vRigidBodies[i]->m_iElements.push_back(ielc);
}

//-------------------------------------------------------------------------------------------------------

void addelement2bndlist(int *iel, int *idofs, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  int idofsc = *idofs;
  myWorld.m_vRigidBodies[i]->m_iBoundaryElements.push_back(std::pair<int,int>(ielc,idofsc));
}

//-------------------------------------------------------------------------------------------------------

void getelementarray(int* elements, int *idofselement, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.begin();
  for(int j=0;iter!=myWorld.m_vRigidBodies[i]->m_iBoundaryElements.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
    idofselement[j]= mypair.second;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettotalelements(int* nel, int* ibody)
{
  int i = *ibody;
  int e1 = myWorld.m_vRigidBodies[i]->m_iElements.size();
  int e2 = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
  int itotal = e1+e2;
  *nel = itotal;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsprev(int* nel, int* ibody)
{
  int i = *ibody;
  *nel  = myWorld.m_vRigidBodies[i]->m_iElementsPrev;
}

//-------------------------------------------------------------------------------------------------------

void getallelements(int* elements, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.begin();
  std::list<int>::iterator iter2 = myWorld.m_vRigidBodies[i]->m_iElements.begin();
  int j;
  for(j=0;iter!=myWorld.m_vRigidBodies[i]->m_iBoundaryElements.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
  }

  for(;iter2!=myWorld.m_vRigidBodies[i]->m_iElements.end();iter2++,j++)
  {
    int iel = *iter2;
    elements[j]= iel;
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelements(int* elements, int* ibody)
{
  int i = *ibody;
  std::list<int>::iterator iter = myWorld.m_vRigidBodies[i]->m_iElements.begin();
  int j;
  for(j=0;iter!=myWorld.m_vRigidBodies[i]->m_iElements.end();iter++,j++)
  {
    int iel = *iter;
    elements[j]= iel;
  }
}
//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsinside(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.m_vRigidBodies[i]->m_iElements.size();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsbndry(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.m_vRigidBodies[i]->m_iBoundaryElements.size();
}

//-------------------------------------------------------------------------------------------------------

void getforce(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID;
  CVector3<double> force(myWorld.m_vRigidBodies[i]->m_vForce.x,
                         myWorld.m_vRigidBodies[i]->m_vForce.y,
                         myWorld.m_vRigidBodies[i]->m_vForce.z);
  *dx=force.x;
  *dy=force.y;
  *dz=force.z;
}

//-------------------------------------------------------------------------------------------------------

void gettorque(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID; 
  CVector3<double> torque(myWorld.m_vRigidBodies[i]->m_vTorque.x,
                          myWorld.m_vRigidBodies[i]->m_vTorque.y,
                          myWorld.m_vRigidBodies[i]->m_vTorque.z);
  *dx=torque.x;
  *dy=torque.y;
  *dz=torque.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumpworld()
{
  std::cout<<myWorld<<std::endl;
  mylog.Write(myWorld.ToString().c_str());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumprigidbodies()
{
  CRigidBodyIO writer;
  writer.Write(myWorld,"particles.i3d");
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logposition()
{
  std::vector<CRigidBody*>::iterator vIter;
  //Check every pair
  mylog.Write("Simulation time: %f",myTimeControl.GetTime());
  int count = 0;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body =*vIter;
    mylog.Write("Position of body %i: %f %f %f:",count,body->m_vCOM.x,body->m_vCOM.y,body->m_vCOM.z);
    count++;
  } 
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logvelocity()
{

}

//-------------------------------------------------------------------------------------------------------

extern "C" void logangularvelocity()
{

}

//-------------------------------------------------------------------------------------------------------

extern "C" void logdistance()
{

}

//-------------------------------------------------------------------------------------------------------

extern "C" void logcollision()
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettype(int *itype, int *iid)
{
  int i = *iid;
  *itype = myWorld.m_vRigidBodies[i]->m_iShape;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void startcollisionpipeline()
{
  //start the collision pipeline
  myPipeline.StartPipeline();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void clearcollisionpipeline()
{
  //erase the old info, if there is any
  //myResponses.clear();
  //make a copy of the responses for postprocessing
  //myResponses=myPipeline.m_Response->m_Responses;
  myPipeline.m_CollInfo.clear();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writeparticles(int *iout)
{
  int iTimestep=*iout;
  std::ostringstream sName,sNameParticles;
  std::string sModel("_vtk/model.vtk");
  std::string sParticle("solution/particles.i3d");
  CVtkWriter writer;
  sName<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sNameParticles<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sModel.append(sName.str());
  sParticle.append(sNameParticles.str());
  
  //Write the grid to a file and measure the time
  //writer.WriteParticleFile(myWorld.m_vRigidBodies,sModel.c_str());
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());

  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
  rbwriter.Write(myWorld,sParticle.c_str(),false);
  
/*  std::ostringstream sNameHGrid;  
  std::string sHGrid("_gmv/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->ConvertToUnstructuredGrid(hgrid);

  writer.WriteUnstr(hgrid,sHGrid.c_str());  */
  
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writeuniformgrid()
{
//   std::ostringstream sName;
//   sName<<"."<<std::setfill('0')<<std::setw(3)<<myWorld.m_myParInfo.GetID();

//   std::string sGrid("_vtk/uniformgrid");
//   sGrid.append(sName.str());
//   sGrid.append(".vtk");
//   CVtkWriter writer;
  
//   //Write the grid to a file and measure the time
//   writer.WriteUniformGrid(myUniformGrid,sGrid.c_str());  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writepvtu(int *iNodes,int *iTime)
{
  int nodes=*iNodes;
  int time =*iTime;
  CVtkWriter writer;
  writer.WritePVTU(nodes,time);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writetri(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode)
{
  int NEL=*iNEL;
  int inode=*iNode;
  
  if(inode==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;

    std::string strEnding(".tri");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  using namespace std;

	std::ostringstream sName;
	sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;
	
	string strEnding(".tri");
	string strFileName("_gmv/res_");
	strFileName.append(sName.str());
	strFileName.append(strEnding);
	
	//open file for writing
	FILE * myfile = fopen(strFileName.c_str(),"w");

  //check
	fprintf(myfile,"Coarse mesh exported by stdQ2P1 \n");
	fprintf(myfile,"Version 0.1a \n");

	fprintf(myfile,"    %i %i 0 8 12 6     NEL,NVT,NBCT,NVE,NEE,NAE\n",NEL,8*NEL);
  //write the points data array to the file
  fprintf(myfile,"DCORVG\n");
  for(int i=0;i<8*NEL;i++)
  {
    fprintf(myfile,"%f %f %f\n",dcorvg[i][0],dcorvg[i][1],dcorvg[i][2]);
  }//end for

  fprintf(myfile,"KVERT\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"%i %i %i %i %i %i %i %i\n",iKVERT[i][0],iKVERT[i][1],iKVERT[i][2],iKVERT[i][3],iKVERT[i][4],iKVERT[i][5],iKVERT[i][6],iKVERT[i][7]);
  }

  fprintf(myfile,"KNPR\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"0\n");
  }

	fclose( myfile );
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime)
{
  int NEL=*iNEL;
  int NVT=*iNVT;
  int node=*iNode;
  int time=*iTime;
  
  if(node==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<node<<"."<<std::setfill('0')<<std::setw(4)<<time;

    std::string strEnding(".vtu");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  CVtkWriter writer;
  writer.WritePUXML(NEL,NVT,iKVERT,dcorvg,vu,vv,vw,vp,dist,node,time);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

void creategrid()
{

}

//-------------------------------------------------------------------------------------------------------

void queryuniformgrid(int* ibody)
{
  int id = *ibody;
  myWorld.m_vRigidBodies[id]->m_iElements.clear();
  myUniformGrid.Query(myWorld.m_vRigidBodies[id]);
  Real avgElements = 0.0;
//   if(id==80885)
//   {
//      for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
//      {

//      }
//   }

//   if(boxDomain.Inside(myWorld.m_vRigidBodies[id]->m_vCOM))
//   {
//     for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
//     {
//       if(myWorld.m_vRigidBodies[j]->m_iElements.size() > 20)
// 	{
// 	  //std::cout<<"Element id: "<<j<<std::endl;
// 	  //std::cout<<myWorld.m_vRigidBodies[j]->m_vCOM;
// 	}
//       avgElements+=myWorld.m_vRigidBodies[j]->m_iElements.size();   
//     }  
//   }
  //std::cout<<"Average Elements to check: "<<avgElements/Real(myWorld.m_vRigidBodies.size())<<" myid: "<<myWorld.m_myParInfo.GetID()<<"\n";
  //std::cout<<"Average Elements: "<<avgElements<<std::endl;
}

//-------------------------------------------------------------------------------------------------------

void createcolltest()
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist)
{

  CDistOps3 op;
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  CVector3<Real> vec(x,y,z);
  double dist=op.BruteForceDistance(Model, vec);
  *ddist=dist;

}//end getcenter

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistanceid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[id];
  CMeshObject<Real> *pMeshObjectOrig = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  ddist = distMeshPoint.ComputeDistance();
  *dist=ddist;
}//end getdistance

//-------------------------------------------------------------------------------------------------------

void intersecbodyelement(int *ibody,int *iel,double vertices[][3])
{

/*    if(body->m_iShape == CRigidBody::BOUNDARYBOX)
    {
      return;
    }*/
    int i = *ibody;
    i--;
    CRigidBody *body = myWorld.m_vRigidBodies[i];
    VECTOR3 verts[8];
    for(int i=0;i<8;i++)
    {
      verts[i]=VECTOR3(Real(vertices[i][0]),Real(vertices[i][1]),Real(vertices[i][2]));
    }
    int in = body->NDofsHexa(verts);
    
    if(in > 0)body->m_iElement = *iel;
    
}

//-------------------------------------------------------------------------------------------------------

void isboundarybody(int* isboundary, int* ibodyc)
{
  int i = *ibodyc;
  if(myWorld.m_vRigidBodies[i]->m_iShape==CRigidBody::BOUNDARYBOX)
  {
    *isboundary = 1;
  }
  else
  {
    *isboundary = 0;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin)
{
  CDistOps3 op;
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  CVector3<Real> vec(x,y,z);
  CRigidBody *pBody = myWorld.m_vRigidBodies[0];
  CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  C3DModel &model = object->m_Model;
  int in=op.BruteForceInnerPointsStatic(model,vec);
  *isin=in;
}//end isinelement

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelementperf(double *dx,double *dy,double *dz,int *isin)
{

  CDistOps3 op;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  CVector3<Real> point(x,y,z);
  int in=0;

  //locate the cell in that the point is
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());    
  
  for(int level=0;level<=pHash->GetMaxLevel();level++)
  {
    if(pHash->IsBoundaryLevel(level))
      continue;

    //get the cell on the current level
    CCellCoords cell = pHash->GetCell(point,level);

    //get the entries of the cell
    std::vector<CSpatialHashEntry> *vEntries = pHash->GetCellEntries(cell);
    
    //test for all entries if the point is inside
    //loop through the entries of the hash bucket
    std::vector<CSpatialHashEntry>::iterator viter = vEntries->begin();
    
    //check cell 
    for(;viter!=vEntries->end();viter++)
    {
      //get the rigid body
      CRigidBody *pBody = viter->m_pBody;
      
      //check if inside, if so then leave the function
      if(pBody->IsInBody(point))
      {
        in=1;
        *isin=pBody->m_iID;
        return;
      }

    }//end for viter
        
  }//end for level
  
}//end isinelementperf

//-------------------------------------------------------------------------------------------------------

extern "C" void isinobstacle(double *dx,double *dy,double *dz,int *isin)
{
  int inside = 0;
  CDistOps3 op;
  int in[3];
  CVector3<float> vec(*dx,*dy,*dz);
  CRigidBody *pBody0 = myWorld.m_vRigidBodies[0];
  CMeshObject<i3d::Real> *object0 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody0->m_pShape);
  C3DModel &model0 = object0->m_Model;

  CRigidBody *pBody1 = myWorld.m_vRigidBodies[1];
  CMeshObject<i3d::Real> *object1 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody1->m_pShape);
  C3DModel &model1 = object1->m_Model;

  CRigidBody *pBody2 = myWorld.m_vRigidBodies[2];
  CMeshObject<i3d::Real> *object2 = dynamic_cast< CMeshObject<i3d::Real> *>(pBody2->m_pShape);
  C3DModel &model2 = object2->m_Model;

  VECTOR3 orig(vec.x,vec.y,vec.z);
  CRay3r ray0(orig,VECTOR3(0,0,1));
  CRay3r ray1(orig,VECTOR3(0,0,-1));
  CRay3r ray2(orig,VECTOR3(0,1,0));

  in[0]=op.BruteForcefbm(model0, orig, ray0);
  in[1]=op.BruteForcefbm(model1, orig, ray1);
  in[2]=op.BruteForcefbm(model2, orig, ray2);
  

  for(int i=0;i<3;i++)
  {
    if(in[i]==1)
    {
      inside=1;
      break;
    }
  }
  *isin=inside;
}//end isinelement

//-------------------------------------------------------------------------------------------------------

extern "C" void setposition(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  CVector3<Real> vNewPos(x,y,z);

  Model.m_vMeshes[0].MoveToPosition(vNewPos);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setrotation(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  

  CVector3<Real> vNewRot(x,y,z);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void writepolygon(int *iout)
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin)
{

  CDistOps3 op;
  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  CVector3<Real> vec(x,y,z);
  int in=0;
  if(myWorld.m_vRigidBodies[id]->IsInBody(vec))
  {
    in=1;
  }
  *isin=in;

}//end isinelement

//-------------------------------------------------------------------------------------------------------

void setelement(int* iel, int* iID)
{
  int i = *iID;
  myWorld.m_vRigidBodies[i]->m_iElement = *iel;
  myWorld.m_vRigidBodies[i]->m_iElementsPrev = 1;
}

//-------------------------------------------------------------------------------------------------------

void getelement(int* iel, int* iID)
{
  int i = *iID;
  *iel = myWorld.m_vRigidBodies[i]->m_iElement;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  int id = *iID;
  CVector3<Real> vNewPos(x,y,z);
  myWorld.m_vRigidBodies[id]->TranslateTo(vNewPos);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;
  CVector3<Real> vNewRot(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vAngle = vNewRot;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;  
  CVector3<Real> vNewVel(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vVelocity = vNewVel;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  i3d::Real x = *dangvelx;
  i3d::Real y = *dangvely;
  i3d::Real z = *dangvelz;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewAngVel(x,y,z);
  myWorld.m_vRigidBodies[id]->GetAngVel() = vNewAngVel;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setforce(double *dforcex,double *dforcey,double *dforcez,int *iid)
{
  i3d::Real x = *dforcex;
  i3d::Real y = *dforcey;
  i3d::Real z = *dforcez;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewForce(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vForce = vNewForce;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settorque(double *dtorquex,double *dtorquey,double *dtorquez,int *iid)
{
  i3d::Real x = *dtorquex;
  i3d::Real y = *dtorquey;
  i3d::Real z = *dtorquez;
  
  int id = *iid;  
  CVector3<i3d::Real> vNewTorque(x,y,z);
  myWorld.m_vRigidBodies[id]->m_vTorque = vNewTorque;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getresponse(double *dux,double *duy,double *duz,
                            double *dcorrx,double *dcorry,double *dcorrz,
                            double *domx,double *domy,double *domz,int *iID)
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getpos(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID;
  CVector3<double> pos(myWorld.m_vRigidBodies[i]->m_vCOM.x,myWorld.m_vRigidBodies[i]->m_vCOM.y,myWorld.m_vRigidBodies[i]->m_vCOM.z);
  *dx=pos.x;
  *dy=pos.y;
  *dz=pos.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getvel(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID; 
  CVector3<double> vel(myWorld.m_vRigidBodies[i]->m_vVelocity.x,myWorld.m_vRigidBodies[i]->m_vVelocity.y,myWorld.m_vRigidBodies[i]->m_vVelocity.z);
  *dx=vel.x;
  *dy=vel.y;
  *dz=vel.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getnumparticles(int *nParts)
{
  
  *nParts=myWorld.m_vRigidBodies.size();
}

extern "C" void getradius(double *drad, int *iid)
{
  int i = *iid;
  CAABB3r box = myWorld.m_vRigidBodies[i]->m_pShape->GetAABB();
  double rad  = box.m_Extends[0];
  *drad = rad;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid)
{
  int i = *iid; 
  CVector3<double> angle(myWorld.m_vRigidBodies[i]->m_vAngle.x,myWorld.m_vRigidBodies[i]->m_vAngle.y,myWorld.m_vRigidBodies[i]->m_vAngle.z);
  *dangx=angle.x;
  *dangy=angle.y;
  *dangz=angle.z;
}

extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  int i = *iid;
  CVector3<double> angvel(myWorld.m_vRigidBodies[i]->GetAngVel().x,myWorld.m_vRigidBodies[i]->GetAngVel().y,myWorld.m_vRigidBodies[i]->GetAngVel().z);
  *dangvelx=angvel.x;
  *dangvely=angvel.y;
  *dangvelz=angvel.z;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getdensity(double *ddens, int *iid)
{
  int i = *iid;
  *ddens=myWorld.m_vRigidBodies[i]->m_dDensity;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settimestep(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetDeltaT(dT);
  myTimeControl.SetPreferredTimeStep(dT);
  //myPipeline.m_pTimeControl->SetDeltaT(dT);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void settime(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetTime(dT);

}

//-------------------------------------------------------------------------------------------------------

void addboundary()
{
  //initialize the box shaped boundary
  myWorld.m_vRigidBodies.push_back(new CRigidBody());
  CRigidBody *body = myWorld.m_vRigidBodies.back();
  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::BOUNDARYBOX;
  CBoundaryBoxr *box = new CBoundaryBoxr();
  box->rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  box->CalcValues();
  body->m_vCOM      = box->rBox.GetCenter();
  body->m_pShape      = box;
  body->m_InvInertiaTensor.SetZero();
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
}

//-------------------------------------------------------------------------------------------------------

void cleanup()
{
  std::vector<CRigidBody*>::iterator vIter;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    delete body;
  }
  delete bdryParameterization;
}

//-------------------------------------------------------------------------------------------------------

void initphysicalparameters()
{

  std::vector<CRigidBody*>::iterator vIter;

  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    body->m_dDensity    = myParameters.m_dDefaultDensity;
    body->m_dVolume     = body->m_pShape->Volume();
    Real dmass          = body->m_dDensity * body->m_dVolume;
    body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
    body->m_vAngle      = VECTOR3(0,0,0);
    body->SetAngVel(VECTOR3(0,0,0));
    body->m_vVelocity   = VECTOR3(0,0,0);
    body->m_vCOM        = VECTOR3(0,0,0);
    body->m_vForce      = VECTOR3(0,0,0);
    body->m_vTorque     = VECTOR3(0,0,0);
    body->m_Restitution = 0.0;
    body->SetOrientation(body->m_vAngle);
    body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());

    //calculate the inertia tensor
    //Get the inertia tensor
    body->GenerateInvInertiaTensor();
  }

}

//-------------------------------------------------------------------------------------------------------

void cupdynamics()
{
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceMesh("meshes/cup_small_high2.obj");
  Real extentBox[3]={0.25, 0.25, 0.025};
  myFactory.AddBoxes(myWorld.m_vRigidBodies,1,extentBox);
  myFactory.AddSpheres(myWorld.m_vRigidBodies,20,myParameters.m_dDefaultRadius);

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.m_vRigidBodies[0]->TranslateTo(VECTOR3(0.49,0.25,0.378));
  myWorld.m_vRigidBodies[1]->TranslateTo(VECTOR3(0.75, 0.25, 0.28));
  myWorld.m_vRigidBodies[1]->m_bAffectedByGravity=false;
  myWorld.m_vRigidBodies[1]->m_dInvMass=0;
  myWorld.m_vRigidBodies[1]->m_InvInertiaTensor.SetZero();
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.m_vRigidBodies[0]->m_pShape);

  C3DModel model_out(pMeshObject->m_Model);
  model_out.m_vMeshes[0].m_matTransform =myWorld.m_vRigidBodies[0]->GetTransformationMatrix();
  model_out.m_vMeshes[0].m_vOrigin =myWorld.m_vRigidBodies[0]->m_vCOM;
  model_out.m_vMeshes[0].TransformModelWorld();
  model_out.GenerateBoundingBox();
  model_out.m_vMeshes[0].GenerateBoundingBox();
  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();
  
  int offset = 2;
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = 7;//myGrid.m_vMax.x/(distbetween+d);
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  Real xstart=myGrid.m_vMin.x + (myGrid.m_vMax.x/2.5) - (drad+distbetween);
  Real ystart=myGrid.m_vMin.y+drad+distbetween+myGrid.m_vMax.y/3.0;  
  VECTOR3 pos(xstart , ystart, (myGrid.m_vMax.z/1.75)-d);
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
  myWorld.m_vRigidBodies[offset]->TranslateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.0015;
  int count=0;
  for(int i=offset+1;i<myWorld.m_vRigidBodies.size();i++)
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
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
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
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  distbetween = 0.5 * drad;
  pos.x+=d+distbetween;
  distbetween = 0.1 * drad;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    std::cout<<"position: "<<pos<<std::endl;    
    if((i+1)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = -0.5+drad+distbetween;
      pos.z -= d+distbetween;
    }
    myWorld.m_vRigidBodies[i]->TranslateTo(pos);
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
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceBoxes(myParameters.m_iBodies, extends);
  
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
      myWorld.m_vRigidBodies[index]->TranslateTo(pos);
      pos.y+=delta;
      index++;
    }
    pos.z+=delta;
  }

  myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(1.15,pos.y-delta,pos.z-2.5*delta));
  //myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,0.75,0);
  myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void createrestingtest()
{
  
  CParticleFactory myFactory;
  
  myWorld = myFactory.ProduceSpheres(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.5 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMin.z)+drad);
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  pos.z+=d;//+distbetween;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.z += d;
    }
    myWorld.m_vRigidBodies[i]->TranslateTo(pos);
    pos.z+=d;//+distbetween;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void createbasf()
{

  CParticleFactory myFactory;
       myWorld = myFactory.ProduceTubes("meshes/myAllClumps.obj");
}

//-------------------------------------------------------------------------------------------------------

void addmesh()
{
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceMesh("meshes/cup_small_high2.obj");

  //assign the physical parameters of the rigid bodies
  initphysicalparameters();

  myWorld.m_vRigidBodies[0]->TranslateTo(VECTOR3(0.25,0.25,0.8));
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.m_vRigidBodies[0]->m_pShape);

  C3DModel model_out(pMeshObject->m_Model);
  model_out.m_vMeshes[0].m_matTransform =myWorld.m_vRigidBodies[0]->GetTransformationMatrix();
  model_out.m_vMeshes[0].m_vOrigin =myWorld.m_vRigidBodies[0]->m_vCOM;
  model_out.m_vMeshes[0].TransformModelWorld();
  model_out.GenerateBoundingBox();
  model_out.m_vMeshes[0].GenerateBoundingBox();
  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);

  //myWorld.m_vRigidBodies[index]->TranslateTo(VECTOR3(0.9,pos.y-delta,pos.z-2.5*delta));
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,0.75,0);
  //myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(-0.9,0.0,0.1);
}

//-------------------------------------------------------------------------------------------------------

void addsphere_dt(int *itime)
{
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  //VECTOR3 pos(myGrid.m_vMin.x+1.0*d, myGrid.m_vMax.y/2.0, myGrid.m_vMax.z/2.0);
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
  
  if(myWorld.m_vRigidBodies.size() < 1000)
  {
    CParticleFactory myFactory;

    int offset = myWorld.m_vRigidBodies.size();

    Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};

    myFactory.AddSpheres(myWorld.m_vRigidBodies,iadd,myParameters.m_dDefaultRadius);
    
    for(int i=0;i<iadd;i++)
    {
      CRigidBody *body    = myWorld.m_vRigidBodies[offset+i];
      body->m_dDensity    = myParameters.m_dDefaultDensity;
      body->m_dVolume     = body->m_pShape->Volume();
      Real dmass          = body->m_dDensity * body->m_dVolume;
      body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
      body->m_vAngle      = VECTOR3(0,0,0);
      body->SetAngVel(VECTOR3(0,0,0));
      body->m_vVelocity   = VECTOR3(0,0,-1.05);
      body->m_vCOM        = VECTOR3(0,0,0);
      body->m_vForce      = VECTOR3(0,0,0);
      body->m_vTorque     = VECTOR3(0,0,0);
      body->m_Restitution = 0.0;
      body->SetOrientation(body->m_vAngle);
      body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
      
      //calculate the inertia tensor
      //Get the inertia tensor
      body->GenerateInvInertiaTensor();
      pos = vPos[i];
      body->TranslateTo(pos);      
    }
  }//end if

  myPipeline.m_pGraph->Clear();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
    myWorld.m_vRigidBodies[j]->m_iID = j;

  std::cout<<"Added body, number of particles: "<<myWorld.m_vRigidBodies.size()<<std::endl;

}

//-------------------------------------------------------------------------------------------------------

void addobstacle()
{

  CObjLoader Loader;

  CRigidBody *body = new CRigidBody();
  CMeshObject<Real> *pMeshObject= new CMeshObject<Real>();

  Loader.ReadMultiMeshFromFile(&pMeshObject->m_Model,"meshes/fritten_final_mili.obj");

  pMeshObject->m_Model.GenerateBoundingBox();

  pMeshObject->SetFileName("meshes/fritten_final_mili.obj");

  body->m_pShape = pMeshObject;
  body->m_iShape = CRigidBody::MESH;
  myWorld.m_vRigidBodies.push_back(body);

  //initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(3.14,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::MESH;

  body->m_vCOM      = VECTOR3(0,0,0);

  body->m_InvInertiaTensor.SetZero();

  body->m_Restitution = 0.0;

  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  body->TranslateTo(VECTOR3(0.13,0.2125,0.0155));

  C3DModel model_out(pMeshObject->m_Model);
  model_out.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    model_out.m_vMeshes[i].m_matTransform =body->GetTransformationMatrix();
    model_out.m_vMeshes[i].m_vOrigin =body->m_vCOM;
    model_out.m_vMeshes[i].TransformModelWorld();
    model_out.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
  subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  pMeshObject->m_BVH.GenTreeStatistics();

}

//-------------------------------------------------------------------------------------------------------

void reactor()
{
  CParticleFactory myFactory;
  int offset = myWorld.m_vRigidBodies.size();
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};

  myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
  
  CRigidBody *body    = myWorld.m_vRigidBodies.back();
  body->m_dDensity    = myParameters.m_dDefaultDensity;
  body->m_dVolume     = body->m_pShape->Volume();
  Real dmass          = body->m_dDensity * body->m_dVolume;
  body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
  body->m_vAngle      = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity   = VECTOR3(0,0,0);
  body->m_vCOM        = VECTOR3(0,0,0);
  body->m_vForce      = VECTOR3(0,0,0);
  body->m_vTorque     = VECTOR3(0,0,0);
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  //VECTOR3 pos(0.0+distbetween+drad, 0, 0);
  
  //VECTOR3 pos(VECTOR3(0.125,0.0,0.0));  
  VECTOR3 pos(0.25,0.3333,0.75);  
  
  body->TranslateTo(pos);
  
  body->m_vVelocity=VECTOR3(0.0,0.0,0);  
  
  //addobstacle();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writeuniformgridlist()
{
//   using namespace std;
  
//   std::ostringstream sName;
//   sName<<"uniformgrid."<<std::setfill('0')<<std::setw(3)<<myWorld.m_myParInfo.GetID();
//   string name;
//   name.append(sName.str());
//   name.append(".grid");
//   ofstream myfile(name.c_str());

//   //check
//   if(!myfile.is_open())
//   {
// 	cout<<"Error opening file: "<<"uniformgrid.grid"<<endl;
// 	exit(0);
//   }//end if
    
//   int x,y,z;
//   for(int z=0;z<myUniformGrid.m_iDimension[2];z++)
//   {
//     for(int y=0;y<myUniformGrid.m_iDimension[1];y++)
//     {
//       for(int x=0;x<myUniformGrid.m_iDimension[0];x++)
//       {
//         std::list<int>::iterator i; //m_lElements
//         int index = z*myUniformGrid.m_iDimension[1]*myUniformGrid.m_iDimension[0]+y*myUniformGrid.m_iDimension[0]+x;
//         myfile<<index;
//         for(i=myUniformGrid.m_pCells[index].m_lElements.begin();i!=myUniformGrid.m_pCells[index].m_lElements.end();i++)
//         {
//           myfile<<" "<<(*i);
//         }
//         myfile<<"\n";
//       }
//     }    
//   }

//   myfile.close();  
}

//-------------------------------------------------------------------------------------------------------

inline float frand()
{
    return rand() / (float) RAND_MAX;
}

//-------------------------------------------------------------------------------------------------------

void SphereOfSpheres()
{
	
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,515,myParameters.m_dDefaultRadius); //515
  initphysicalparameters();
	
	int r = 5, ballr = 5;
	// inject a sphere of particles
	float pr = myParameters.m_dDefaultRadius;
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
							float jitter = myParameters.m_dDefaultRadius*0.01f;
							if ((l <= myParameters.m_dDefaultRadius*2.0f*r) && (index < myWorld.m_vRigidBodies.size())) {
								  VECTOR3 position(pos[0] + dx + (frand()*2.0f-1.0f)*jitter,
																	 pos[1] + dy + (frand()*2.0f-1.0f)*jitter,
																	 pos[2] + dz + (frand()*2.0f-1.0f)*jitter);
								  myWorld.m_vRigidBodies[index]->TranslateTo(position);
									myWorld.m_vRigidBodies[index]->m_dColor = position.x;
									index++;
							}
					}
			}
	}

}

//-------------------------------------------------------------------------------------------------------

void spherestack()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.25 * drad;
  Real distbetweenz = 0.5 * drad;
  int perrowx = myGrid.m_vMax.x/(distbetween+d);
  int perrowy = myGrid.m_vMax.y/(distbetween+d);  
  
  int numPerLayer = perrowx * perrowy;
  int layers =1;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();
  
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+0.0025, (myGrid.m_vMax.z-drad));
  
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
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
        myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myGrid.m_vMin.x+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z-=d;
    pos.y=myGrid.m_vMin.y+drad+distbetween+0.0025;        
  }

}

//-------------------------------------------------------------------------------------------------------

void drivcav()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real myxmin =  0.0;  
  Real myymin =  0.0;  
  Real myzmin =  0.6;  

  Real myxmax = 1.0;  
  Real myymax = 0.25;  
  Real myzmax = 1.0;  


  Real drad  = myParameters.m_dDefaultRadius;
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
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
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
        myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);
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
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real drad = myParameters.m_dDefaultRadius;
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
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();

  std::vector<CRigidBody*>::iterator vIter;
  std::vector<VECTOR3>::iterator i;

  for(vIter=myWorld.m_vRigidBodies.begin(),i=vPos.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++,i++)
  {
    CRigidBody *body    = *vIter;
    VECTOR3 pos         = *i;
    body->TranslateTo(pos);
  }

}

//-------------------------------------------------------------------------------------------------------

void particlesinbox()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real myxmin =  0.0;  
  Real myymin =  0.0;  
  Real myzmin =  0.6;  

  Real myxmax = 1.0;  
  Real myymax = 0.25;  
  Real myzmax = 1.0;  


  Real drad  = myParameters.m_dDefaultRadius;
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
  myFactory.AddSpheres(myWorld.m_vRigidBodies,nTotal,myParameters.m_dDefaultRadius);  
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
          myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);
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
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  
  VECTOR3 vMin = myGrid.GetAABB().m_Verts[0];
  VECTOR3 vMax = myGrid.GetAABB().m_Verts[1];
  
  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,nTotal,drad);
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
	      if((mypos - myWorld.m_vRigidBodies[j]->m_vCOM).mag() <= 2.0 * drad)
		{
		  valid=false;
		  break;
		}
	    }
	}        
      //one row in x
      VECTOR3 bodypos = VECTOR3(randx,randy,randz);
      myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);

    }

}

//-------------------------------------------------------------------------------------------------------

void initrigidbodies()
{
  CParticleFactory myFactory;

  if(myParameters.m_iBodyInit == 0)
  {
    myWorld = myFactory.ProduceFromParameters(myParameters);
  }
  else if(myParameters.m_iBodyInit == 1)
  {
    myWorld = myFactory.ProduceFromFile(myParameters.m_sBodyFile.c_str(),myTimeControl);
  }
  else
  {
    if(myParameters.m_iBodyInit == 2)
    {
      initrandompositions();
    }

    if(myParameters.m_iBodyInit == 3)
    {
      createlineuptest();
    }

    if(myParameters.m_iBodyInit == 4)
    {
      reactor();
    }
    if(myParameters.m_iBodyInit == 5)
    {
      createbasf();
    }
    
    if(myParameters.m_iBodyInit == 6)
    {
      spherestack();
    }
    if(myParameters.m_iBodyInit == 7)
    {
      //drivcav();
      particlesinbox();
      myFactory.AddFromDataFile(myParameters, &myWorld);
    }

    if(myParameters.m_iBodyInit == 8)
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
      hPos[i*4]   = myWorld.m_vRigidBodies[i]->m_vCOM.x; 
      hPos[i*4+1] = myWorld.m_vRigidBodies[i]->m_vCOM.y; 
      hPos[i*4+2] = myWorld.m_vRigidBodies[i]->m_vCOM.z; 
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

void initsimulation()
{

  //first of all initialize the rigid bodies
  initrigidbodies();

#ifdef FC_CUDA_SUPPORT
  initParticleSystem(myWorld.m_vRigidBodies.size(),gridSize);
#endif

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();

  //add the boundary as a rigid body
  addcylinderboundary();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
  {
    myWorld.m_vRigidBodies[j]->m_iID = j;
    myWorld.m_vRigidBodies[j]->m_iElementsPrev = 0;
  }

  //set the timestep
  myTimeControl.SetDeltaT(myParameters.m_dTimeStep);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

  //link the boundary to the world
  myWorld.SetBoundary(&myBoundary);

  //set the time control
  myWorld.SetTimeControl(&myTimeControl);

  //set the gravity
  myWorld.SetGravity(myParameters.m_vGrav);

  //Set the collision epsilon
  myPipeline.SetEPS(0.02);

  //initialize the collision pipeline 
  myPipeline.Init(&myWorld,myParameters.m_iSolverType,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  if(myParameters.m_iSolverType==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CMotionIntegratorSI(&myWorld);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CRigidBodyMotion(&myWorld);
  }

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

}

void continuesimulation()
{
  
  CParticleFactory myFactory;

  //Produces a domain
  //it is a bit unsafe, because the domain at this point is
  //not fully useable, because of non initialized values in it
  //string = ssolution
  myWorld = myFactory.ProduceFromFile(myParameters.m_sSolution.c_str(),myTimeControl);

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();

  //add the boundary as a rigid body
  addcylinderboundary();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
  {
    myWorld.m_vRigidBodies[j]->m_iID = j;
    myWorld.m_vRigidBodies[j]->m_iElementsPrev = 0;
  }
  
  //set the timestep
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myParameters.m_iTotalTimesteps+=myTimeControl.GetTimeStep();

  //link the boundary to the world
  myWorld.SetBoundary(&myBoundary);

  //set the time control
  myWorld.SetTimeControl(&myTimeControl);

  //set the gravity
  myWorld.SetGravity(myParameters.m_vGrav);

  //Set the collision epsilon
  myPipeline.SetEPS(0.02);

  //initialize the collision pipeline 
  //initialize the collision pipeline 
  myPipeline.Init(&myWorld,myParameters.m_iSolverType,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  if(myParameters.m_iSolverType==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CMotionIntegratorSI(&myWorld);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CRigidBodyMotion(&myWorld);
  }

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  
  
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
  //writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());
  writer.WriteParticleFile(myWorld.m_vRigidBodies,sModel.c_str());
  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
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
  CRigidBody *body = bdryParameterization;  
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->m_pShape);
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
  for(int i=0;i<bdryParams.size();i++)
  {
    if(bdryParams[i]->m_iID==bdryId)
    {
      CRigidBody *body = bdryParams[i];  
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->m_pShape);
      Real x=*dx;
      Real y=*dy;
      Real z=*dz;
      CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,VECTOR3(x,y,z));
      Real ddist = distMeshPoint.ComputeDistance();
      *dxx=distMeshPoint.m_Res.m_vClosestPoint.x;
      *dyy=distMeshPoint.m_Res.m_vClosestPoint.y;
      *dzz=distMeshPoint.m_Res.m_vClosestPoint.z; 
      break;
    }
  }
}

extern "C" void initbdryparam()
{
  bdryParameterization = new CRigidBody();
  bdryParameterization->m_vVelocity       = VECTOR3(0,0,0);
  bdryParameterization->m_dDensity        = 1.0;
  bdryParameterization->m_Restitution     = 0.0;
  bdryParameterization->m_vAngle          = VECTOR3(0,0,0);
  bdryParameterization->SetAngVel(VECTOR3(0,0,0));
  bdryParameterization->m_iShape          = CRigidBody::MESH;
  bdryParameterization->m_iID             = -1;
  bdryParameterization->m_vCOM            = VECTOR3(0,0,0);
  bdryParameterization->m_vForce          = VECTOR3(0,0,0);
  bdryParameterization->m_vTorque         = VECTOR3(0,0,0);
  bdryParameterization->m_dDampening      = 1.0;  
  bdryParameterization->m_iElementsPrev   = 0;
  bdryParameterization->m_bRemote         = false;
  bdryParameterization->SetOrientation(bdryParameterization->m_vAngle);
  bdryParameterization->m_bAffectedByGravity = false;

  bdryParameterization->m_pShape = new CMeshObject<Real>();
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(bdryParameterization->m_pShape);
  std::string fileName;
  pMeshObject->SetFileName(fileName.c_str());
  bdryParameterization->m_dVolume   = bdryParameterization->m_pShape->Volume();
  bdryParameterization->m_dInvMass  = 0.0;

  CGenericLoader Loader;
  Loader.ReadModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

  pMeshObject->m_Model.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
  }
  
  C3DModel model_out(pMeshObject->m_Model);
  model_out.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    model_out.m_vMeshes[i].m_matTransform = bdryParameterization->GetTransformationMatrix();
    model_out.m_vMeshes[i].m_vOrigin = bdryParameterization->m_vCOM;
    model_out.m_vMeshes[i].TransformModelWorld();
    model_out.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,9,0,model_out.GetBox(),&pTriangles);
  CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);      
  bdryParameterization->m_InvInertiaTensor.SetZero();
}

extern "C" void fallingparticles()
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  CReader reader;
  std::string meshFile;

  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);

  int argc=1;
  char *argv[1]={"./stdQ2P1"};
     
#ifdef FC_CUDA_SUPPORT
  initGL(&argc,argv);
  cudaGLInit(argc,argv);
	
  uint gridDim = GRID_SIZE;
  gridSize.x = gridSize.y = gridSize.z = gridDim;
#endif

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.InitMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.InitCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.m_iStartType == 0)
  {
    initsimulation();
  }
  else
  {
    continuesimulation();
  }
    
  CRigidBody *body = myWorld.m_vRigidBodies[0];

  printf("BoundingSphereRadius: %f \n",body->m_pShape->GetAABB().GetBoundingSphereRadius());
}

extern "C" void initaneurysm()
{
  CParticleFactory factory;
  myWorld = factory.ProduceMesh("meshes/aneurysma3.obj");
  CRigidBody *body = myWorld.m_vRigidBodies[0];

  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->m_pShape);
  pMeshObject->SetFileName("meshes/aneurysma3.obj");
  body->m_dVolume   = body->m_pShape->Volume();
  body->m_dInvMass  = 0.0;

  pMeshObject->m_Model.GenerateBoundingBox();
  pMeshObject->m_Model.GetBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = pMeshObject->m_Model.GenTriangleVector();
  //std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,5,0,pMeshObject->m_Model.GetBox(),&pTriangles);
  CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);
  
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,VECTOR3(0,0,0));
  Real ddist = distMeshPoint.ComputeDistance();
  printf("Distance: %f \n",ddist);
  printf("ClosestPoint: %f %f %f\n",distMeshPoint.m_Res.m_vClosestPoint.x,
                                    distMeshPoint.m_Res.m_vClosestPoint.y,
                                    distMeshPoint.m_Res.m_vClosestPoint.z);
  
  CVtkWriter writer;
  //writer.WriteModel(pMeshObject->m_Model,"output/model.vtk");
}

extern "C" void initdeform()
{

  CReader reader;  
  CParticleFactory factory;

  //read the user defined configuration file
  reader.ReadParametersDeform(std::string("start/data.TXT"),myDeformParameters);  
  
  myWorld = factory.ProduceFromDeformParameters(myDeformParameters);  
  
}

extern "C" void initpointlocation()
{

  CReader reader;  

  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);  
  
  CParticleFactory factory;  
  
  myWorld = factory.ProduceFromParameters(myParameters);  
  
}

extern "C" void addbdryparam(int *iBnds,char *name, int length)
{
  
  //null terminate string
  name[length--]='\0';
  int ilength=strlen(name);
  std::string fileName(name);
  //printf("Name of file: %s, Length of string: %i \n",name,ilength);  
  std::cout<<"Name of file: "<<fileName<<" Length of string: "<<fileName.size()<<std::endl;
  
  CRigidBody *param = new CRigidBody();
  param->m_vVelocity       = VECTOR3(0,0,0);
  param->m_dDensity        = 1.0;
  param->m_Restitution     = 0.0;
  param->m_vAngle          = VECTOR3(0,0,0);
  param->SetAngVel(VECTOR3(0,0,0));
  param->m_iShape          = CRigidBody::MESH;
  param->m_iID             = *iBnds;
  param->m_vCOM            = VECTOR3(0,0,0);
  param->m_vForce          = VECTOR3(0,0,0);
  param->m_vTorque         = VECTOR3(0,0,0);
  param->m_dDampening      = 1.0;  
  param->m_iElementsPrev   = 0;
  param->m_bRemote         = false;
  param->SetOrientation(param->m_vAngle);
  param->m_bAffectedByGravity = false;

  param->m_pShape = new CMeshObject<Real>();
  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(param->m_pShape);
  pMeshObject->SetFileName(fileName.c_str());
  param->m_dVolume   = param->m_pShape->Volume();
  param->m_dInvMass  = 0.0;

  CGenericLoader Loader;
  Loader.ReadModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

  pMeshObject->m_Model.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
  }
  
  C3DModel model_out(pMeshObject->m_Model);
  model_out.GenerateBoundingBox();
  for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
  {
    model_out.m_vMeshes[i].m_matTransform = param->GetTransformationMatrix();
    model_out.m_vMeshes[i].m_vOrigin = param->m_vCOM;
    model_out.m_vMeshes[i].TransformModelWorld();
    model_out.m_vMeshes[i].GenerateBoundingBox();
  }

  std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
  CSubDivRessources myRessources(1,7,0,model_out.GetBox(),&pTriangles);
  CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
  pMeshObject->m_BVH.InitTree(&subdivider);      
  param->m_InvInertiaTensor.SetZero();
  
  CRigidBody *body = param;  
  CMeshObjectr *pMeshObject2 = dynamic_cast<CMeshObjectr *>(body->m_pShape);
  bdryParams.push_back(param);
  printf("Boundary parameterization file %s initialized successfully.\n",fileName.c_str());
  
}