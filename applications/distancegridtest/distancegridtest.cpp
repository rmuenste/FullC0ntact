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
World myWorld;
CollisionPipeline myPipeline;
RigidBodyMotion myMotion;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
TimeControl myTimeControl;
WorldParameters myParameters;
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
  myWorld.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_  = 0;
  body->volume_   = 0;
  body->invMass_     = 0;
  body->angle_    = VECTOR3(0,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_ = VECTOR3(0,0,0);
  body->shape_    = RigidBody::BOUNDARYBOX;
  CBoundaryBoxr *box = new CBoundaryBoxr();
  box->rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  box->CalcValues();
  body->com_      = box->rBox.GetCenter();
  body->shape_      = box;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

void cleanup()
{
  std::vector<RigidBody*>::iterator vIter;
  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
  {
    RigidBody *body    = *vIter;
    delete body;
  }
}


void initphysicalparameters()
{

  std::vector<RigidBody*>::iterator vIter;

  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
  {
    RigidBody *body    = *vIter;
    if(!body->affectedByGravity_)
      continue;
    body->density_    = myParameters.defaultDensity_;
    body->volume_     = body->shape_->Volume();
    Real dmass          = body->density_ * body->volume_;
    body->invMass_    = 1.0/(body->density_ * body->volume_);
    body->angle_      = VECTOR3(0,0,0);
    body->setAngVel(VECTOR3(0,0,0));
    body->velocity_   = VECTOR3(0,0,0);
    body->com_        = VECTOR3(0,0,0);
    body->m_vForce      = VECTOR3(0,0,0);
    body->torque_     = VECTOR3(0,0,0);
    body->restitution_ = 0.0;
    body->setOrientation(body->angle_);
    body->setTransformationMatrix(body->getQuaternion().GetMatrix());
    //calculate the inertia tensor
    //Get the inertia tensor
    body->generateInvInertiaTensor();
  }

}

void reactor()
{
  ParticleFactory myFactory;
  int offset = myWorld.rigidBodies_.size();
  Real extends[3]={myParameters.defaultRadius_,myParameters.defaultRadius_,myParameters.defaultRadius_};

  myFactory.addSpheres(myWorld.rigidBodies_,1,myParameters.defaultRadius_);
  
  RigidBody *body    = myWorld.rigidBodies_.back();
  body->density_    = myParameters.defaultDensity_;
  body->volume_     = body->shape_->Volume();
  Real dmass          = body->density_ * body->volume_;
  body->invMass_    = 1.0/(body->density_ * body->volume_);
  body->angle_      = VECTOR3(0,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_   = VECTOR3(0,0,0);
  body->com_        = VECTOR3(0,0,0);
  body->m_vForce      = VECTOR3(0,0,0);
  body->torque_     = VECTOR3(0,0,0);
  body->restitution_ = 0.0;
  body->generateInvInertiaTensor();  
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->getQuaternion().GetMatrix());

  Real drad = myParameters.defaultRadius_;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  VECTOR3 pos(0.6,0.6,0.6);
  body->translateTo(pos);
  body->velocity_=VECTOR3(0.0,0.0,0.0);

  //addobstacle();

}

void initrigidbodies()
{
  ParticleFactory myFactory;

  //Produces a domain
  //it is a bit unsafe, because the domain at this point is
  //not fully useable, because of non initialized values in it
  //myWorld = myFactory.ProduceSphericalWithObstacles(iPart);
  //myWorld = myFactory.ProduceSpherical(iPart);

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
      reactor();
    }

    if(myParameters.bodyInit_ == 3)
    {
    }

    if(myParameters.bodyInit_ == 4)
    {
      myWorld = myFactory.produceFromParameters(myParameters);      
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
  for(int j=0;j<myWorld.rigidBodies_.size();j++)
    myWorld.rigidBodies_[j]->iID_ = j;

  //set the timestep
  myTimeControl.SetDeltaT(0.00125);
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

  //initialize the collision pipeline 
  myPipeline.init(&myWorld,myParameters.maxIterations_,myParameters.pipelineIterations_);

  //set the broad phase to simple spatialhashing
  myPipeline.setBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion=RigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.integrator_ = &myMotion;

  
  myWorld.densityMedium_ = myParameters.densityMedium_;
  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;
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
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();
  
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
  myPipeline.init(&myWorld,myParameters.maxIterations_,myParameters.pipelineIterations_);

  //set the broad phase to simple spatialhashing
  myPipeline.setBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion=RigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.integrator_ = &myMotion;

  
  myWorld.densityMedium_ = myParameters.densityMedium_;
  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;
  myWorld.densityMedium_ = myParameters.densityMedium_;
  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;
  
  myPipeline.response_->m_pGraph = myPipeline.graph_;  

  RigidBody *body    = myWorld.rigidBodies_[4];
  //body->m_InvInertiaTensor.SetZero();
  body->setAngVel(VECTOR3(0,0,0));

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
  writer.WriteRigidBodies(myWorld.rigidBodies_,sModel.c_str());
  RigidBodyIO rbwriter;
  myWorld.output_ = iTimestep;
  std::vector<int> indices;
  indices.push_back(12);
  indices.push_back(13);
  indices.push_back(15);
  rbwriter.write(myWorld,indices,sParticle.c_str());
  rbwriter.write(myWorld,sParticle.c_str());
  writer.WriteContacts(myPipeline.contacts_,sContacts.str().c_str());

  std::ostringstream sNameHGrid;
  std::string sHGrid("output/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  SpatialHashHierarchy *pHash = dynamic_cast<SpatialHashHierarchy*>(myPipeline.broadPhase_->strategy_->implicitGrid_->getSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->convertToUnstructuredGrid(hgrid);

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
  RigidBody *pBody = myWorld.rigidBodies_[0];
  CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->shape_);
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

void ComputeElements(RigidBody *body, int iel, bool *visited) 
{
  visited[iel]=true;

  for(int i=0;i<6;i++)
  {
    int ielN = myGrid.m_pHexas[iel].hexaNeighborIndices_[i];
    if(visited[ielN])
      continue;
    
    VECTOR3 verts[8];
    for(int j=0;j<8;j++)
      verts[j] = myGrid.m_pVertexCoords[myGrid.m_pHexas[ielN].hexaVertexIndices_[j]];

    int in = body->nDofsHexa(verts);
    if(in > 0)
    {
      body->elements_.push_back(ielN);
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
  Reader reader;
  std::string meshFile=std::string("meshes/mesh01.tri");
  //read the user defined configuration file
  reader.readParameters(string("start/data.TXT"),myParameters);

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
  if(myParameters.startType_ == 0)
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
  for(;myWorld.timeControl_->m_iTimeStep<=myParameters.nTimesteps_;myWorld.timeControl_->m_iTimeStep++)
  {
    Real simTime = myTimeControl.GetTime();
    energy0=myWorld.getTotalEnergy();
    cout<<"------------------------------------------------------------------------"<<endl;
    //g_Log.Write("## Timestep Nr.: %d | Simulation time: %lf | time step: %lf \n Energy: %lf",
    //            myWorld.m_pTimeControl->m_iTimeStep,myTimeControl.GetTime(),myTimeControl.GetDeltaT(),energy0);

    cout<<"## Timestep Nr.: "<<myWorld.timeControl_->m_iTimeStep<<" | Simulation time: "<<myTimeControl.GetTime()
      <<" | time step: "<<myTimeControl.GetDeltaT() <<endl;
    cout<<"Energy: "<<energy0<<endl;
    cout<<"------------------------------------------------------------------------"<<endl;
    cout<<endl;
    //addsphere_dt();
    myPipeline.startPipeline();
    energy1=myWorld.getTotalEnergy();
    cout<<"Energy after collision: "<<energy1<<endl;
    cout<<"Energy difference: "<<energy0-energy1<<endl;
    
  CUnstrGrid::ElementIter iel;
  std::cout<<"Computing FBM information and distance..."<<std::endl;

  RigidBody *body = myWorld.rigidBodies_.front();
  body->elements_.clear();
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

     in = body->nDofsHexa(verts);

     if(in > 0)
     {
       body->element_ = id;
     }
  }

  //body fill elements
  bool *visited = new bool[myGrid.m_iNEL];
  for(int k=0;k<myGrid.m_iNEL;k++)
  {
     visited[k]=false;
  }
  
  body->elements_.push_back(body->element_);
  ComputeElements(body,body->element_,visited);

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


