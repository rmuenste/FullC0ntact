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
CWorld myWorld;
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

double xmin=-7;
double ymin=-7;
double zmin=-35;
double xmax=7;
double ymax=10;
double zmax=60;
Real radius = Real(0.075);
int iReadGridFromFile = 1;
int *islots=NULL;

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
  if(islots !=NULL)
  {
    delete []islots;
    islots=NULL;
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

void addsphere_dt()
{
  bool addsphere=false;
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  VECTOR3 pos(myGrid.m_vMin.x+1.0*d, myGrid.m_vMax.y/2.0, myGrid.m_vMax.z/2.0);

  //VECTOR3 pos(0.0+distbetween+drad, 0, 0);

  Real noise = 0.0005;
  if(myTimeControl.GetTimeStep() == 0)
    addsphere=false;
  else if(myTimeControl.GetTimeStep()%200 == 0)
    addsphere=true;

  if((addsphere) && (myWorld.m_vRigidBodies.size() < 256))
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

    //calculate the inertia tensor
    //Get the inertia tensor
    body->GenerateInvInertiaTensor();
    int n = rand()%100;
    if(n <50)
      body->TranslateTo(VECTOR3(pos.x-noise,pos.y,pos.z));
    else
      body->TranslateTo(VECTOR3(pos.x+noise,pos.y,pos.z));

  }//end if

  myPipeline.m_pGraph->Clear();

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
    myWorld.m_vRigidBodies[j]->m_iID = j;

}

void drivcav()
{
  
  CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};

  Real myxmin = -0.7;  
  Real myymin = -0.7;  
  Real myzmin =  -5.0;  

  Real myxmax = 0.7;  
  Real myymax = 0.7;  
  Real myzmax = 6.0;  


  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.5 * drad;
  Real distbetweenz = 0.5 * drad;

  Real extendX = myxmax - myxmin;  
  Real extendY = myymax - myymin;  
  Real extendZ = myzmax - myzmin;  

  int perrowx = extendX/(distbetween+d);
  int perrowy = extendY/(distbetween+d);  
  
  int numPerLayer = perrowx * perrowy;
  int layers = 5;
  int nTotal = numPerLayer * layers;

  //add the desired number of particles
  myFactory.AddSpheres(myWorld.m_vRigidBodies,numPerLayer*layers,myParameters.m_dDefaultRadius);  
  initphysicalparameters();
  
  VECTOR3 pos(myxmin+drad+distbetween , myymin+drad+distbetween+0.0025, (myzmin+drad));
  
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
        CQuaternionr q;
        q.CreateFromEulerAngles(0,0,0.785398163);
        //q.CreateFromEulerAngles(0,0,1.570796327);
        MATRIX3X3 mat  = q.GetMatrix();  
        bodypos = mat * bodypos;
        
        bodypos = bodypos + VECTOR3(0,9,-18);
        //VECTOR3 trans = bodypos - VECTOR3(0,10,-18);
        //bodypos = trans + VECTOR3(0,10,-18);;        
        
        myWorld.m_vRigidBodies[count]->TranslateTo(bodypos);        
        
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


void addobstacle()
{

  CObjLoader Loader;

  CRigidBody *body = new CRigidBody();
  CMeshObject<Real> *pMeshObject= new CMeshObject<Real>();

  Loader.ReadMultiMeshFromFile(&pMeshObject->m_Model,"meshes/fritten_scale.obj");

  pMeshObject->m_Model.GenerateBoundingBox();

  pMeshObject->SetFileName("meshes/fritten_scale.obj");

  body->m_pShape = pMeshObject;
  body->m_iShape = CRigidBody::MESH;
  myWorld.m_vRigidBodies.push_back(body);

  //initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(1.57,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::MESH;

  body->m_vCOM      = VECTOR3(0,0,0);

  body->m_InvInertiaTensor.SetZero();

  body->m_Restitution = 0.0;

  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  body->TranslateTo(VECTOR3(0.0,myGrid.m_vMax.y/2.0+0.15,0.0));

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

void createlineuptest()
{
  CParticleFactory myFactory;
  int offset = myWorld.m_vRigidBodies.size();
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+1.0*drad, myGrid.m_vMax.y/2.0, myGrid.m_vMin.z+6.0*drad);
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  distbetween = 0.0 * drad;
  pos.z+=d;
  distbetween = 0.0 * drad;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    myWorld.m_vRigidBodies[i]->TranslateTo(pos);
    pos.z+=d+distbetween;
  }


/*  myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
  CRigidBody *body    = myWorld.m_vRigidBodies.back();
  body->m_dDensity    = myParameters.m_dDefaultDensity;
  body->m_dVolume     = body->m_pShape->Volume();
  Real dmass          = body->m_dDensity * body->m_dVolume;
  body->m_dInvMass    = 1.0/(body->m_dDensity * body->m_dVolume);
  body->m_vAngle      = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity   = VECTOR3(0.0,0,-0.25);
  body->m_vCOM        = VECTOR3(0,0,0);
  body->m_vForce      = VECTOR3(0,0,0);
  body->m_vTorque     = VECTOR3(0,0,0);
  body->m_Restitution = 0.0;
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  body->GenerateInvInertiaTensor();
  distbetween = 9.0 * drad;
  pos.z+=d+distbetween;
  body->TranslateTo(pos);*/
  
  addobstacle();

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
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  
  //VECTOR3 pos(myGrid.m_vMin.x+1.0*drad, myGrid.m_vMax.y/2.0, myGrid.m_vMin.z+6.0*drad);

  VECTOR3 pos(0.13+0.006,0.195,0.0);

  body->TranslateTo(pos);
  body->m_vVelocity=VECTOR3(10,0,0);

  addobstacle();

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
      //meshtorus();
    }

    if(myParameters.m_iBodyInit == 3)
    {
      reactor();
    }

    if(myParameters.m_iBodyInit == 4)
    {
      drivcav();
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
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  
  
  std::cout<<"Number of rigid bodies: "<<myWorld.m_vRigidBodies.size()<<std::endl;

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
  addboundary();

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
  myPipeline.Init(&myWorld,myParameters.m_iMaxIterations,myParameters.m_iPipelineIterations);

  //set the broad phase to simple spatialhashing
  myPipeline.SetBroadPhaseHSpatialHash();
  //myPipeline.SetBroadPhaseNaive();
  //myPipeline.SetBroadPhaseSpatialHash();

  //set which type of rigid motion we are dealing with
  myMotion = new CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  
  myPipeline.m_Response->m_pGraph = myPipeline.m_pGraph;  

  //assign the rigid body ids
  for(int j=0;j<myWorld.m_vRigidBodies.size();j++)
    myWorld.m_vRigidBodies[j]->m_iID = j;


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
  indices.push_back(12);
  indices.push_back(13);
  indices.push_back(15);
  //rbwriter.Write(myWorld,indices,sParticle.c_str());
  //rbwriter.Write(myWorld,sParticle.c_str(),false);
  //writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

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

  //start the main simulation loop
  for(;myWorld.m_pTimeControl->m_iTimeStep<=myParameters.m_iTotalTimesteps;myWorld.m_pTimeControl->m_iTimeStep++)
  {
    Real simTime = myTimeControl.GetTime();
    //energy0=myWorld.GetTotalEnergy();
    cout<<"------------------------------------------------------------------------"<<endl;
    //g_Log.Write("## Timestep Nr.: %d | Simulation time: %lf | time step: %lf \n Energy: %lf",
    //            myWorld.m_pTimeControl->m_iTimeStep,myTimeControl.GetTime(),myTimeControl.GetDeltaT(),energy0);

    cout<<"## Timestep Nr.: "<<myWorld.m_pTimeControl->m_iTimeStep<<" | Simulation time: "<<myTimeControl.GetTime()
      <<" | time step: "<<myTimeControl.GetDeltaT() <<endl;
    //cout<<"Energy: "<<energy0<<endl;
    cout<<"------------------------------------------------------------------------"<<endl;
    cout<<endl;
    //addsphere_dt();
    myPipeline.StartPipeline();
    //energy1=myWorld.GetTotalEnergy();
    //cout<<"Energy after collision: "<<energy1<<endl;
    //cout<<"Energy difference: "<<energy0-energy1<<endl;
    if(myWorld.m_pTimeControl->m_iTimeStep%10==0)
    {
      std::cout<<"Timestep finished... writing vtk."<<std::endl;
      writetimestep(iOut);
      std::cout<<"Finished writing vtk."<<std::endl;
      iOut++;
    }
    myTimeControl.SetTime(simTime+myTimeControl.GetDeltaT());
    dTimePassed += myTimeControl.GetDeltaT();
  }//end for

  cleanup();

  return 1;
}
