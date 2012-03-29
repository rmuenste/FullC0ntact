/***************************************************************************
 *   Copyright (C) 2006-2010 by Raphael Muenster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

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
#include <rigidbodymotion.h>
#include <mymath.h>
#include <distancetriangle.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <matrixnxn.h>
#include <vectorn.h>
#include <linearsolvergauss.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <distancefuncgridmodel.h>
#include <limits>
#include <perftimer.h>
#include <reader.h>
#include <hspatialhash.h>
#include <broadphasestrategy.h>
#include <objloader.h>
#include <collisionpipeline.h>

using namespace i3d;

Real a = CMath<Real>::MAXREAL;
CUnstrGrid myGrid;
CWorld myWorld;
CCollisionPipeline myPipeline;
CRigidBodyMotion myMotion;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
CTimeControl myTimeControl;
CWorldParameters myParameters;
Real startTime=0.0;
Real *array;
std::vector<Real> myarray;
int perrowx;
int perrowy;
int perrowz;

float xmin=0.0f;
float ymin=0.0f;
float zmin=0.0f;
float xmax=1.0f;
float ymax=1.0f;
float zmax=1.0f;


int iReadGridFromFile = 0;

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

void cleanup()
{
  std::vector<CRigidBody*>::iterator vIter;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    delete body;
  }
}


void initphysicalparameters()
{

  std::vector<CRigidBody*>::iterator vIter;

  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    if(!body->m_bAffectedByGravity)
      continue;
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
  body->GenerateInvInertiaTensor();  
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  VECTOR3 pos(0.6,0.6,0.6);
  body->TranslateTo(pos);
  body->m_vVelocity=VECTOR3(0.0,0.0,0.0);

  //addobstacle();

}

void initrigidbodies()
{
  CParticleFactory myFactory;

  //Produces a domain
  //it is a bit unsafe, because the domain at this point is
  //not fully useable, because of non initialized values in it
  //myWorld = myFactory.ProduceSphericalWithObstacles(iPart);
  //myWorld = myFactory.ProduceSpherical(iPart);

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
      reactor();
    }

    if(myParameters.m_iBodyInit == 3)
    {
    }

    if(myParameters.m_iBodyInit == 4)
    {
      myWorld = myFactory.ProduceFromParameters(myParameters);      
    }
  }

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();

  //add the boundary as a rigid body
  addboundary();
  
}

void initsimulation()
{

  //first of all initialize the rigid bodies
  initrigidbodies();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
    myWorld.m_vRigidBodies[j]->m_iID = j;

  //set the timestep
  myTimeControl.SetDeltaT(0.00125);
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
  myPipeline.Init(&myWorld,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion=CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = &myMotion;

  
  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
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
  myPipeline.Init(&myWorld,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion=CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = &myMotion;

  
  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

  CRigidBody *body    = myWorld.m_vRigidBodies[4];
  //body->m_InvInertiaTensor.SetZero();
  body->SetAngVel(VECTOR3(0,0,0));

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
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());
  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
  std::vector<int> indices;
  indices.push_back(12);
  indices.push_back(13);
  indices.push_back(15);
  rbwriter.Write(myWorld,indices,sParticle.c_str());
  rbwriter.Write(myWorld,sParticle.c_str());
  writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

  std::ostringstream sNameHGrid;
  std::string sHGrid("output/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->ConvertToUnstructuredGrid(hgrid);

  writer.WriteUnstr(hgrid,sHGrid.c_str());  
  
  //if(iout==0)
  //{
    std::ostringstream sNameGrid;
    std::string sGrid("output/grid.vtk");
    sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
    sGrid.append(sNameGrid.str());
    writer.WriteUnstr(myGrid,myarray, sGrid.c_str());
  //}
}

void startsimulation()
{
  myGrid.InitCube(xmin,ymin,zmin,xmax,ymax,zmax);
  //myGrid.InitMeshFromFile("meshes/basf0254.tri");
  //myGrid.InitMeshFromFile("meshes/mesh3.tri");
  //myGrid.InitMeshFromFile("meshes/mesh2.tri");
  //myGrid.InitMeshFromFile("meshes/testgrid4x4.tri");
  
  double dx=0;
  double dy=0;
  double dz=0;
  int    in=0;
  int i;
  
  myGrid.InitStdMesh();
  for(int i=0;i<6;i++)
  {
    myGrid.Refine();
    std::cout<<"Generating Grid level"<<i+1<<std::endl;
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
    myGrid.InitStdMesh();
  } 

  CUnstrGrid::VertexIter vIter;
  std::cout<<"Computing FBM information and distance..."<<std::endl;
  for(vIter=myGrid.VertexBegin();vIter!=myGrid.VertexEnd();vIter++)
  {
    VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
    CVector3f vecf = CVector3f((*vIter).x,(*vIter).y,(*vIter).z);
    int in;
    int id = vIter.GetPos();
    myGrid.m_myTraits[vIter.GetPos()].iTag=0;
    myGrid.m_myTraits[vIter.GetPos()].distance=1000;
    myGrid.m_myTraits[vIter.GetPos()].iX=0;
  }

  CDistOps3 op;
  CRigidBody *pBody = myWorld.m_vRigidBodies[0];
  CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
  C3DModel &model = object->m_Model;

  std::cout<<"Starting FMM..."<<std::endl;
  CDistanceFuncGridModel<Real> distGModel(&myGrid,model);
  //distGModel.ComputeDistance();
  CPerfTimer timer0;
  timer0.Start();
  for(vIter=myGrid.VertexBegin();vIter!=myGrid.VertexEnd();vIter++)
  {
    VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
    int in;
    int id = vIter.GetPos();
    in = distGModel.PointInside(object->m_BVH.GetChild(0),vec);
    myGrid.m_myTraits[vIter.GetPos()].iTag=in;
    //myGrid.m_myTraits[vIter.GetPos()].distance=std::numeric_limits<Real>::max();
    myGrid.m_myTraits[vIter.GetPos()].distance=0;
    myGrid.m_myTraits[vIter.GetPos()].iX=0;
  }
  std::cout<<"Time FBM: "<<timer0.GetTime()<<std::endl; 
  std::cout<<"finished..."<<std::endl;

  std::vector<C3DModel> vModels;
  vModels.push_back(model);
  CVtkWriter writer;
  writer.WriteUnstrXML(myGrid,"output/grid.vtu");
  writer.WriteBasf(vModels,"output/model.vtk");
}

void ComputeElements(CRigidBody *body, int iel, bool *visited) 
{
  visited[iel]=true;

  for(int i=0;i<6;i++)
  {
    int ielN = myGrid.m_pHexas[iel].m_iAdj[i];
    if(visited[ielN])
      continue;
    
    VECTOR3 verts[8];
    for(int j=0;j<8;j++)
      verts[j] = myGrid.m_pVertexCoords[myGrid.m_pHexas[ielN].m_iVertInd[j]];

    int in = body->NDofsHexa(verts);
    if(in > 0)
    {
      body->m_iElements.push_back(ielN);
      ComputeElements(body,ielN,visited);
    }
  }
}

int main()
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  CReader reader;
  std::string meshFile=std::string("meshes/mesh01.tri");
  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);

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

  myGrid.InitStdMesh();
  for(int i=0;i<4;i++)
  {
    myGrid.Refine();
    std::cout<<"Generating Grid level"<<i+1<<std::endl;
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
    myGrid.InitStdMesh();
  } 

  array = new Real[myGrid.m_iNEL];
  
  //start the main simulation loop
  for(;myWorld.m_pTimeControl->m_iTimeStep<=myParameters.m_iTotalTimesteps;myWorld.m_pTimeControl->m_iTimeStep++)
  {
    Real simTime = myTimeControl.GetTime();
    energy0=myWorld.GetTotalEnergy();
    cout<<"------------------------------------------------------------------------"<<endl;
    //g_Log.Write("## Timestep Nr.: %d | Simulation time: %lf | time step: %lf \n Energy: %lf",
    //            myWorld.m_pTimeControl->m_iTimeStep,myTimeControl.GetTime(),myTimeControl.GetDeltaT(),energy0);

    cout<<"## Timestep Nr.: "<<myWorld.m_pTimeControl->m_iTimeStep<<" | Simulation time: "<<myTimeControl.GetTime()
      <<" | time step: "<<myTimeControl.GetDeltaT() <<endl;
    cout<<"Energy: "<<energy0<<endl;
    cout<<"------------------------------------------------------------------------"<<endl;
    cout<<endl;
    //addsphere_dt();
    myPipeline.StartPipeline();
    energy1=myWorld.GetTotalEnergy();
    cout<<"Energy after collision: "<<energy1<<endl;
    cout<<"Energy difference: "<<energy0-energy1<<endl;
    
  CUnstrGrid::ElementIter iel;
  std::cout<<"Computing FBM information and distance..."<<std::endl;

  CRigidBody *body = myWorld.m_vRigidBodies.front();
  body->m_iElements.clear();
  std::cout<<"Starting FMM..."<<std::endl;
  CPerfTimer timer0;
  timer0.Start();
  
  for(iel=myGrid.ElemBegin();iel!=myGrid.ElemEnd();iel++)
  {
     int id = iel.GetInt();        
     array[id]=0.0;
     CHexa &hexa = *iel;
     CUnstrGrid::VertElemIter ive = myGrid.VertElemBegin(&hexa);
     VECTOR3 verts[8];
     int vertex;
     for(vertex=0;ive!=myGrid.VertElemEnd(&hexa);ive++,vertex++)
     {
       VECTOR3 vec = VECTOR3((*ive).x,(*ive).y,(*ive).z);
       verts[vertex]=vec;
     }

     int in=0;

     in = body->NDofsHexa(verts);

     if(in > 0)
     {
       body->m_iElement = id;
     }
  }

  //body fill elements
  bool *visited = new bool[myGrid.m_iNEL];
  for(int k=0;k<myGrid.m_iNEL;k++)
  {
     visited[k]=false;
  }
  
  body->m_iElements.push_back(body->m_iElement);
  ComputeElements(body,body->m_iElement,visited);

  for(int k=0;k<myGrid.m_iNEL;k++)
  {
    if(visited[k])
      array[k]=1.0;

    myarray.push_back(array[k]);
  }

  delete[] visited;
  
  std::cout<<"Time FBM: "<<timer0.GetTime()<<std::endl; 
  std::cout<<"finished..."<<std::endl;
    
  std::cout<<"Timestep finished... writing vtk."<<std::endl;
  writetimestep(iOut);
  std::cout<<"Finished writing vtk."<<std::endl;
  iOut++;
  dTimePassed = 0.f;
  
  myTimeControl.SetTime(simTime+myTimeControl.GetDeltaT());
  dTimePassed += myTimeControl.GetDeltaT();
  
  }//end for

  cleanup();

  return 1;
}


