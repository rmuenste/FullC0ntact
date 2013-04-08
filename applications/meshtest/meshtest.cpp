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
#include <uniformgrid.h>
#include <huniformgrid.h>
#include <perftimer.h>

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
CHUniformGrid<Real,CUGCell> myUniformGrid;

int perrowx;
int perrowy;
int perrowz;
int nTotal = 1000;

double xmin =-0.25f;
double ymin =-0.25f;
double zmin = 0;
double xmax = 0.25f;
double ymax = 0.25f;
double zmax = 1.0f;
Real radius = Real(0.05);
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

    //calculate the inertia tensor
    //Get the inertia tensor
    body->GenerateInvInertiaTensor();
  }

}


void createlineuptest()
{
	Real drad = radius;
	Real d    = 2.0 * drad;
	Real distbetween = 0.25 * drad;
	int perrow = myGrid.m_vMax.x/(distbetween+d);
	VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween, myGrid.m_vMax.y/2.0, myGrid.m_vMax.z/4-d);
	myWorld.m_vRigidBodies[0]->TranslateTo(pos);
	distbetween = 5 * drad;
	pos.x+=d+distbetween;
	distbetween = 0.1 * drad;
	for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
	{
		if((i+1)%(perrow) == 0)
		{
			//advance in y and reset x
			pos.x = myGrid.m_vMin.x+drad+distbetween;
			pos.z -= d+distbetween;
		}
		myWorld.m_vRigidBodies[i]->TranslateTo(pos);
		pos.x+=d+distbetween;
	}
}

void createstackingtest()
{
  CParticleFactory myFactory;
  Real extends[3] = {myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myWorld = myFactory.ProduceSpheres(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween, (myGrid.m_vMax.z/2.0)-d);
  myWorld.m_vRigidBodies[0]->TranslateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.005;
  for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.z -= d+distbetween;
      if(even)
      {
        ynoise = -ynoise;
      }
    }
    VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
    pos.x+=d+distbetween;
    ynoise = -ynoise;
  }
}

void randposition()
{
bool finish=false;
int x,y;
CParticleFactory myFactory;

	while(!finish)
	{
		x = rand()%perrowx;
		y = rand()%perrowy;
    //check if there is a sphere in this slot
    if(islots[x*perrowy+y]==0)
    {
      myFactory.AddSpheres(myWorld.m_vRigidBodies,1,myParameters.m_dDefaultRadius);
			//move to center of grid cell 
      islots[x*perrowy+y]=1;
			finish=true;
    }
	}
}

void addobject()
{

  Real timeElapsed = myTimeControl.GetTime() - startTime;
  if((timeElapsed > 0.1) && (myWorld.m_vRigidBodies.size() < 256))
  {
    //add a few spheres
    for(int i=0;i<4;i++)
    {
      randposition();
    }//end for
  }//end if

  for(int x=0;x<perrowx;x++)
    for(int y=0;y<perrowy;y++)
      islots[x*perrowy+y]=0;

}
 
void addspheres()
{
  
  CParticleFactory myFactory;
  int offset = myWorld.m_vRigidBodies.size();
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
  //myFactory.AddCylinders(myWorld.m_vRigidBodies,24,extends);
  //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
  myFactory.AddSpheres(myWorld.m_vRigidBodies,512,myParameters.m_dDefaultRadius);
  //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
  //myFactory.AddCylinders(myWorld.m_vRigidBodies,24,extends);
  //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
  VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween, (myGrid.m_vMax.z/1.0)-d);
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
  myWorld.m_vRigidBodies[offset]->TranslateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.005;
  int count=0;
  for(int i=offset+1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
      pos.y += d+distbetween;
      if(even)
      {
        ynoise = -ynoise;
      }
      if(++count==3)
      {
        pos.z -= d+distbetween;
        pos.y=myGrid.m_vMin.y+drad+distbetween;
        count=0;
      }
    }
    VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
    pos.x+=d+distbetween;
    ynoise = -ynoise;
  }

} 
 
void meshtorus()
{
	CParticleFactory myFactory;
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  myWorld = myFactory.ProduceMesh("meshes/cup_small_high.obj");
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
  VECTOR3 pos(xstart , ystart, (myGrid.m_vMax.z/1.7)-d);
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

/*  myWorld.m_vRigidBodies.back()->TranslateTo(VECTOR3(0.5,0.25,0.08));

  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real distbetween = 0.25 * drad;

  perrowx = myGrid.m_vMax.x/(distbetween+d);
  perrowy = myGrid.m_vMax.y/(distbetween+d);

  islots = new int[perrowx*perrowy];
  for(int x=0;x<perrowx;x++)
    for(int y=0;y<perrowy;y++)
      islots[x*perrowy+y]=0;*/
     
  
}

float randFloat(float LO, float HI)
{
  return (LO + (float)rand()/((float)RAND_MAX/(HI-LO)));
}

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
    //one row in x
    VECTOR3 bodypos = VECTOR3(randFloat(vMin.x,vMax.x),randFloat(vMin.y,vMax.y),randFloat(vMin.z,vMax.z));
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
  }

}

void add()
{
  
  CParticleFactory myFactory;
  int offset = myWorld.m_vRigidBodies.size();
  Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,2.0*myParameters.m_dDefaultRadius};
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,175,myParameters.m_dDefaultRadius);
  myFactory.AddCylinders(myWorld.m_vRigidBodies,24,extends);

  //Real extends[3]={myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius,myParameters.m_dDefaultRadius};
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,myParameters.m_dDefaultRadius);
  //myWorld = myFactory.ProduceCylinders(myParameters.m_iBodies,extends);
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,124,myParameters.m_dDefaultRadius);
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,4,0.05);
  initphysicalparameters();
  Real drad = myParameters.m_dDefaultRadius;
  Real d    = 2.0 * drad;
  Real dz    = 4.0 * drad;
  Real distbetween = 0.25 * drad;
  Real distbetweenz = 0.5 * drad;
  int perrow = myGrid.m_vMax.x/(distbetween+d);
  //VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.0)-d);
	VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMin.y+drad+distbetween+0.002, (myGrid.m_vMax.z/2.0)-dz);
  //VECTOR3 pos(myGrid.m_vMax.x-drad-distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/1.5)-d);
  myWorld.m_vRigidBodies[offset]->TranslateTo(pos);
  pos.x+=d+distbetween;
  bool even=(perrow%2==0) ? true : false;
  Real ynoise = 0.0025;
	int count=0;
  for(int i=offset+1;i<myWorld.m_vRigidBodies.size();i++)
  {
    if((i)%(perrow) == 0)
    {
      //advance in y and reset x
      pos.x = myGrid.m_vMin.x+drad+distbetween;
			pos.y += d+distbetween;
      if(even)
      {
        ynoise = -ynoise;
      }
      if(++count==3)
			{
				pos.z -= dz+distbetweenz;
				pos.y=myGrid.m_vMin.y+drad+distbetween+0.002;
				count=0;
			}
    }
    VECTOR3 bodypos = VECTOR3(pos.x,pos.y+ynoise,pos.z);
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
    pos.x+=d+distbetween;
    ynoise = -ynoise;
  }

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
  	//meshtorus();
	}
	else if(myParameters.m_iBodyInit == 1)
	{
		myWorld = myFactory.ProduceFromFile(myParameters.m_sBodyFile.c_str(),myTimeControl);

	}
	else
	{
		if(myParameters.m_iBodyInit == 2)
		{
			meshtorus();
		}

		if(myParameters.m_iBodyInit == 3)
		{
			createstackingtest();
		}

		if(myParameters.m_iBodyInit == 4)
		{
			initrandompositions();
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

  //set which type of rigid motion we are dealing with
  myMotion = new CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = myMotion;

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
	//writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());
  writer.WriteParticleFile(myWorld.m_vRigidBodies,sModel.c_str());
	CRigidBodyIO rbwriter;
	myWorld.m_iOutput = iTimestep;
  std::vector<int> indices;
  //indices.push_back(8);
  //indices.push_back(0);
  //indices.push_back(11);
  //rbwriter.Write(myWorld,indices,sParticle.c_str());
  rbwriter.Write(myWorld,sParticle.c_str());
  writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

	if(iout==0)
	{
		std::ostringstream sNameGrid;
		std::string sGrid("output/grid.vtk");
		sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
		sGrid.append(sNameGrid.str());
		writer.WriteUnstr(myGrid,sGrid.c_str());
	}

  //Write the grid to a file and measure the time
  writer.WriteUniformGrid(myUniformGrid.m_pLevels[0],"output/uniformgrid_level0.vtk");
  writer.WriteUniformGrid(myUniformGrid.m_pLevels[1],"output/uniformgrid_level1.vtk");
  writer.WriteUniformGrid(myUniformGrid.m_pLevels[2],"output/uniformgrid_level2.vtk");

}

double elementsize(VECTOR3 element[])
{

  VECTOR3 elementMin(element[0]);
  VECTOR3 elementMax(element[0]);

  for(int i=1;i<8;i++)
  {
    if(elementMin.x > element[i].x)
      elementMin.x = element[i].x;

    if(elementMin.y > element[i].y)
      elementMin.y = element[i].y;

    if(elementMin.z > element[i].z)
      elementMin.z = element[i].z;

    if(elementMax.x < element[i].x)
      elementMax.x = element[i].x;

    if(elementMax.y < element[i].y)
      elementMax.y = element[i].y;

    if(elementMax.z < element[i].z)
      elementMax.z = element[i].z;
  }

  CAABB3r gridElement = CAABB3r(elementMin,elementMax);
  double size = gridElement.GetBoundingSphereRadius();
  return size;

}

struct sortSizes {
  bool operator()(const std::pair<double,int> &a, const std::pair<double,int> &b)
  {
    return a.first < b.first;
  }
};

int main()
{
	using namespace std;
	int iOut=0;
	Real dTimePassed=1;
	Real energy0=0.0;
	Real energy1=0.0;
	CReader reader;
	std::string meshFile("meshes/fbm_fac.tri");

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
  for(int i=0;i<1;i++)
  {
    myGrid.Refine();
    std::cout<<"Generating Grid level"<<i+1<<std::endl;
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
    myGrid.InitStdMesh();
  } 

  std::list< std::pair<double,int> > sizes;
  CUnstrGrid::ElementIter iel;
  for(iel=myGrid.ElemBegin();iel!=myGrid.ElemEnd();iel++)
  {
    int id = iel.GetInt();        
    //std::cout<<"IEL="<<id<<std::endl;

    CHexa &hexa = *iel;
    CUnstrGrid::VertElemIter ive = myGrid.VertElemBegin(&hexa);
    VECTOR3 verts[8];
    int vertex;

    for(vertex=0;ive!=myGrid.VertElemEnd(&hexa);ive++,vertex++)
    {
      VECTOR3 vec = VECTOR3((*ive).x,(*ive).y,(*ive).z);
      verts[vertex]=vec;
    }

    double size=elementsize(verts);
    sizes.push_back(std::pair<double,int>(size,id));
  }

  sizes.sort(sortSizes());
  std::vector<int> vDistribution;
  std::vector<double> vGridSizes;
  double factor = 2.5;
  std::list< std::pair<double,int> >::iterator liter = sizes.begin();  
  double tsize = factor * ((*liter).first);
  std::cout<<"New size: "<<tsize<<"\n";
  liter++;
  int levels=0;
  int elemPerLevel=1;
  vDistribution.push_back(elemPerLevel);
  vGridSizes.push_back(tsize);
  for(;liter!=sizes.end();liter++)
  {
    double dsize=((*liter).first);
    if(dsize > tsize)
    {
      tsize=factor*dsize;
      std::cout<<"New size: "<<tsize<<"\n";
      elemPerLevel=1;
      vDistribution.push_back(elemPerLevel);
      vGridSizes.push_back(tsize);
    }
    else
    {
      int &myback = vDistribution.back();
      myback++;
    }
  }

  levels=vDistribution.size();

  std::cout<<"Number of levels: "<<levels<<"\n";
  int totalElements=0;
  for(int j=0;j<vDistribution.size();j++)
  {
    std::cout<<vDistribution[j]<< " elements on level: "<<j+1<<"\n";
    totalElements+=vDistribution[j];
  }

  CAABB3r boundingBox = myGrid.GetAABB();
  
  myUniformGrid.InitGrid(boundingBox,levels);

  for(int j=0;j<vGridSizes.size();j++)
  {
    std::cout<<"Building level: "<<j+1<<"\n";
    myUniformGrid.InitGridLevel(j,vGridSizes[j]);
  }

  myUniformGrid.InsertElements(sizes,myGrid);

  for(int j=0;j<vGridSizes.size();j++)
  {
    std::cout<<"Level: "<<j+1<<" Element check: "<<myUniformGrid.m_pLevels[j].GetNumEntries()<<" "<<vDistribution[j]<<"\n";
  }

  CPerfTimer timer0;
  timer0.Start();  
  for(int j=0;j<nTotal;j++)
  {
    myUniformGrid.Query(myWorld.m_vRigidBodies[j]);
  }
  Real time = timer0.GetTime();    
  std::cout<<"Time: "<<time<<std::endl;

	std::cout<<"Timestep finished... writing vtk."<<std::endl;
	writetimestep(iOut);
	std::cout<<"Finished writing vtk."<<std::endl;

  cleanup();

	return 1;
}
