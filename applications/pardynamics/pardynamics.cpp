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
#include <collisionpipeline.h>
#include <rigidbodymotionmesh.h>
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
#include <reader.h>
#include <worldparameters.h>
#include <globalheader.h>
#include <meshobject.h>
#include <subdivisioncreator.h>
#include <boundingvolumetree3.h>
#include <hspatialhash.h>
#include <broadphasestrategy.h>
#include <objloader.h>
#include <motionintegratorsi.h>

using namespace i3d;

Real a = CMath<Real>::MAXREAL;
CUnstrGrid myGrid;

CUnstrGrid domain0;
CUnstrGrid domain1;

CWorld myWorld0;
CWorld myWorld1;
CCollisionPipeline myPipeline;
CRigidBodyMotion *myMotion;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
CTimeControl myTimeControl;
CWorldParameters myParameters;
Real startTime=0.0;

int perrowx;
int perrowy;
int perrowz;

double xmin=0;
double ymin=0;
double zmin=0;
double xmax=1.0f;
//double ymax=0.35f;
double ymax=1.0f;
double zmax=5.0f;
Real radius = Real(0.05);
int iReadGridFromFile = 0;
int *islots=NULL;

void addboundary(CWorld &myWorld)
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

void cleanup(CWorld &myWorld)
{
  std::vector<CRigidBody*>::iterator vIter;
  for(vIter=myWorld.m_vRigidBodies.begin();vIter!=myWorld.m_vRigidBodies.end();vIter++)
  {
    CRigidBody *body    = *vIter;
    delete body;
  }
  if(islots !=NULL)
  {
    delete []islots;
    islots=NULL;
  }
}

void initphysicalparameters(CWorld &myWorld)
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

//-------------------------------------------------------------------------------------------------------

void spherestack0()
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
  int layers = 2;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld0.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters(myWorld0);

  Real ynoise = 0;//0.0025;

  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+ynoise, myGrid.m_vMin.z+drad);
  
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);

  int count=0;
    
  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld0.m_vRigidBodies[count]->TranslateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myGrid.m_vMin.x+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myGrid.m_vMin.y+drad+distbetween+ynoise;        
  }

}

void spherestack1()
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
  int layers = 2;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld1.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters(myWorld1);

  Real ynoise = 0;//0.0025;

  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+ynoise, myGrid.m_vMin.z+drad+2*d);
  
  int count=0;
    
  for(int z=0;z<layers;z++)
  {
    for(int j=0;j<perrowy;j++)
    {
      for(int i=0;i<perrowx;i++,count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
        myWorld1.m_vRigidBodies[count]->TranslateTo(bodypos);
        pos.x+=d+distbetween;
      }
      pos.x=myGrid.m_vMin.x+drad+distbetween;
      pos.y+=d+distbetween;    
    }
    ynoise = -ynoise;        
    pos.z+=d;
    pos.y=myGrid.m_vMin.y+drad+distbetween+ynoise;        
  }

}
 
void initrigidbodies()
{
  CParticleFactory myFactory;

  spherestack0();
  spherestack1();

  //initialize the box shaped boundary
  myBoundary.rBox.Init(xmin,ymin,zmin,xmax,ymax,zmax);
  myBoundary.CalcValues();

  //add the boundary as a rigid body
  addboundary(myWorld0);
  addboundary(myWorld1);
  
}

void initsimulation()
{

  //first of all initialize the rigid bodies
  initrigidbodies();

  //assign the rigid body ids
  for(int j=0;j<myWorld0.m_vRigidBodies.size();j++)
    myWorld0.m_vRigidBodies[j]->m_iID = j;

  //assign the rigid body ids
  for(int j=0;j<myWorld1.m_vRigidBodies.size();j++)
    myWorld1.m_vRigidBodies[j]->m_iID = j;


  //set the timestep
  myTimeControl.SetDeltaT(myParameters.m_dTimeStep);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

  //link the boundary to the world
  myWorld0.SetBoundary(&myBoundary);
  myWorld1.SetBoundary(&myBoundary);

  //set the time control
  myWorld0.SetTimeControl(&myTimeControl);
  myWorld1.SetTimeControl(&myTimeControl);

  //set the gravity
  myWorld0.SetGravity(myParameters.m_vGrav);
  myWorld1.SetGravity(myParameters.m_vGrav);

  //Set the collision epsilon
  myPipeline.SetEPS(0.02);

  //initialize the collision pipeline 
  myPipeline.Init(&myWorld0,myParameters.m_iSolverType,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();

  if(myParameters.m_iSolverType==2)
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CMotionIntegratorSI(&myWorld0);
  }
  else
  {
    //set which type of rigid motion we are dealing with
    myMotion = new CRigidBodyMotion(&myWorld0);
  }

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;
 
  myWorld0.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

}

void writetimestep(int iout)
{
  std::ostringstream sName,sNameParticles,sContacts;
  std::string sModel("output/model0.vtk");
  CVtkWriter writer;
  int iTimestep=iout;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sModel.append(sName.str());

  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld0.m_vRigidBodies,sModel.c_str());

  sModel="output/model1.vtk";
  sModel.append(sName.str());

  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld1.m_vRigidBodies,sModel.c_str());

  myWorld0.m_iOutput = iTimestep;
  std::ostringstream sNameHGrid;
  std::string sHGrid("output/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->ConvertToUnstructuredGrid(hgrid);

  writer.WriteUnstr(hgrid,sHGrid.c_str());  
    
  if(iout==0)
  {
    std::ostringstream sNameGrid;
    std::string sGrid("output/grid.vtk");
    sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
    sGrid.append(sNameGrid.str());
    writer.WriteUnstr(myGrid,sGrid.c_str());

    sGrid="output/domain0.vtk";
    writer.WriteUnstr(domain0,sGrid.c_str());
    sGrid="output/domain1.vtk";
    writer.WriteUnstr(domain1,sGrid.c_str());
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
  std::string meshFile=std::string("meshes/mesh.tri");
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

  domain0.InitCube(0,0,0,1,1,1.8);
  domain1.InitCube(0,0,1.8,1,1,4);

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.m_iStartType == 0)
  {
    initsimulation();
  }

  //start the main simulation loop
  for(;myWorld0.m_pTimeControl->m_iTimeStep<=myParameters.m_iTotalTimesteps;myWorld0.m_pTimeControl->m_iTimeStep++)
  {
    Real simTime = myTimeControl.GetTime();
    cout<<"------------------------------------------------------------------------"<<endl;
    cout<<"## Timestep Nr.: "<<myWorld0.m_pTimeControl->m_iTimeStep<<" | Simulation time: "<<myTimeControl.GetTime()
      <<" | time step: "<<myTimeControl.GetDeltaT() <<endl;
    cout<<"------------------------------------------------------------------------"<<endl;
    cout<<endl;

    myPipeline.StartPipeline();
    //if(dTimePassed >= myTimeControl.GetPreferredTimeStep())
    //{
      std::cout<<"Timestep finished... writing vtk."<<std::endl;
      writetimestep(iOut);
      writetimestep(iOut);
      std::cout<<"Finished writing vtk."<<std::endl;
      iOut++;
      dTimePassed = 0.f;
//    }
    myTimeControl.SetTime(simTime+myTimeControl.GetDeltaT());
    dTimePassed += myTimeControl.GetDeltaT();
  }//end for

  cleanup(myWorld0);
  cleanup(myWorld1);

  return 0;
}

