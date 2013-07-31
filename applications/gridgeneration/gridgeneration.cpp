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
#include <ugsizeheuristicstd.h>
#include <distancemeshpoint.h>
#include <distancemap.h>

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

double xmin =-0.35f;
double ymin =-0.35f;
double zmin =-0.212f;
double xmax = 0.35f;
double ymax = 0.35f;
double zmax = 0.488f;
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
  
  int count=50;      
  
  //add the desired number of particles
  //myFactory.AddSpheres(myWorld.m_vRigidBodies,nTotal,drad);
  std::string meshFile("meshes/swimmer_export.obj");
  myFactory.AddMeshObjects(myWorld.m_vRigidBodies, count, meshFile.c_str());
  
  std::cout<<"Number of mesh objects: "<<nTotal<<std::endl;
  initphysicalparameters();
  
  VECTOR3 pos(0,0,0);

  for(int i=0;i<count;i++)
  {
    
    drad = myWorld.m_vRigidBodies[i]->GetBoundingSphereRadius();
    Real randx=randFloat(vMin.x+drad,vMax.x-drad);
    Real randy=randFloat(vMin.y+drad,vMax.y-drad);
    Real randz=randFloat(vMin.z+drad,vMax.z-drad);
    
//     //get a random float so that the distance to the
//     //sides of the cylinder is less than the radius
//     Real randy=randFloat(drad,8.0-drad);   
//     randy-=4.0;  
//     while( (randx*randx) + (randy*randy) >= (4.0-drad)*(4.0-drad) )
//     {
//       randx=randFloat(drad,8.0-drad);
//       randy=randFloat(drad,8.0-drad);   
//       randy-=4.0;
//       randx-=4.0;          
//     }
        
    //one row in x
    VECTOR3 bodypos = VECTOR3(randx,randy,randz);
    myWorld.m_vRigidBodies[i]->TranslateTo(bodypos);
    
    CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(myWorld.m_vRigidBodies[i]->m_pShape);
    C3DModel model_out(pMeshObject->m_Model);
    model_out.GenerateBoundingBox();
    for(int j=0;j< pMeshObject->m_Model.m_vMeshes.size();j++)
    {
      model_out.m_vMeshes[j].m_matTransform = myWorld.m_vRigidBodies[i]->GetTransformationMatrix();
      model_out.m_vMeshes[j].m_vOrigin = myWorld.m_vRigidBodies[i]->m_vCOM;
      model_out.m_vMeshes[j].TransformModelWorld();
      model_out.m_vMeshes[j].GenerateBoundingBox();
    }

    std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
    CSubDivRessources myRessources(1,2,0,model_out.GetBox(),&pTriangles);
    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
    pMeshObject->m_BVH.InitTree(&subdivider);    
        
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
	//addboundary();
	
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
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,sModel.c_str());
  //writer.WriteParticleFile(myWorld.m_vRigidBodies,sModel.c_str());
  CRigidBodyIO rbwriter;
  myWorld.m_iOutput = iTimestep;
  std::vector<int> indices;
  //indices.push_back(8);
  //indices.push_back(0);
  //indices.push_back(11);
  //rbwriter.Write(myWorld,indices,sParticle.c_str());
  //rbwriter.Write(myWorld,sParticle.c_str());
  //writer.WriteContacts(myPipeline.vContacts,sContacts.str().c_str());

  if(iout==0)
  {
    std::ostringstream sNameGrid;
    std::string sGrid("output/grid.vtk");
    sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
    sGrid.append(sNameGrid.str());
    writer.WriteUnstr(myGrid,sGrid.c_str());
  }

  writer.WriteGrid2Tri(myGrid,"output/case2a.tri");

//   writer.WriteUniformGrid(myUniformGrid.m_pLevels[1],"output/uniformgrid_level1.vtk");
//   writer.WriteUniformGrid(myUniformGrid.m_pLevels[2],"output/uniformgrid_level2.vtk");

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
  std::string meshFile("meshes/EvenBetterMesh.tri");

  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);
  CPerfTimer timer0;

  timer0.Start();

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
  std::cout<<"Finished distance computation in: "<<timer0.GetTime()<<std::endl;    
  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.InitMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    Real size = myWorld.m_vRigidBodies[0]->GetBoundingSphereRadius();
    Real size2 = myWorld.m_vRigidBodies[0]->m_pShape->GetAABB().m_Extends[myWorld.m_vRigidBodies[0]->m_pShape->GetAABB().LongestAxis()] + 0.02f;
    VECTOR3 boxCenter = myWorld.m_vRigidBodies[0]->m_pShape->GetAABB().m_vCenter;

    Real extends[3];
    extends[0]=size;
    extends[1]=size;
    extends[2]=size;
    CAABB3r myBox(boxCenter,size);
    myGrid.InitCubeFromAABB(myBox);
  }

  CUnstrGrid::VertexIter ive;
  std::cout<<"Computing FBM information and distance..."<<std::endl;
 
  CRigidBody *body = myWorld.m_vRigidBodies.front();
 
//   body->m_iElements.clear();
//   
  CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(body->m_pShape);
  C3DModel &model = object->m_Model;
	
	
  myGrid.InitStdMesh();
  
  for(ive=myGrid.VertexBegin();ive!=myGrid.VertexEnd();ive++)
  {
    int id = ive.GetPos();
    VECTOR3 vQuery((*ive).x,(*ive).y,(*ive).z);
    //if(body->IsInBody(vQuery))
    if(myGrid.m_piVertAtBdr[id]==1)
    {
      myGrid.m_myTraits[id].iTag=1;
    }
    else
    {
      myGrid.m_myTraits[id].iTag=0;      
    }
        
    if(id%1000==0)
    {
      std::cout<<"Progress: "<<id<<"/"<<myGrid.m_iNVT<<std::endl;        
    }        
        
    if(myGrid.m_piVertAtBdr[id]==1)        
    {
      CDistanceMeshPoint<Real> distMeshPoint(&object->m_BVH,vQuery);
      myGrid.m_myTraits[id].distance = distMeshPoint.ComputeDistance();          
      myGrid.m_myTraits[id].vNormal = distMeshPoint.m_Res.m_vClosestPoint - vQuery;
      //if(myGrid.m_myTraits[id].distance > 0.02)
      {
        myGrid.m_pVertexCoords[id]= distMeshPoint.m_Res.m_vClosestPoint;
      }
    }
    else
    {
      myGrid.m_myTraits[id].distance = 1.0;    
      
      myGrid.m_myTraits[id].vNormal = VECTOR3(0,0,0);      
    }         
  }  

  for(int i=0;i<2;i++)
  {
    myGrid.Refine();
    
    std::cout<<"Generating Grid level"<<i+1<<std::endl;
    std::cout<<"---------------------"<<std::endl;
    std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
    myGrid.InitStdMesh();
    
    for(ive=myGrid.VertexBegin();ive!=myGrid.VertexEnd();ive++)
    {
      int id = ive.GetPos();
      VECTOR3 vQuery((*ive).x,(*ive).y,(*ive).z);
      //if(body->IsInBody(vQuery))
      if(myGrid.m_piVertAtBdr[id]==1)
      {
        myGrid.m_myTraits[id].iTag=1;
      }
      else
      {
        myGrid.m_myTraits[id].iTag=0;      
      }
          
      if(id%1000==0)
      {
        std::cout<<"Progress: "<<id<<"/"<<myGrid.m_iNVT<<std::endl;        
      }        
          
      if(myGrid.m_piVertAtBdr[id]==1)        
      {
        CDistanceMeshPoint<Real> distMeshPoint(&object->m_BVH,vQuery);
        myGrid.m_myTraits[id].distance = distMeshPoint.ComputeDistance();          
        myGrid.m_myTraits[id].vNormal = distMeshPoint.m_Res.pNode->m_Traits.m_vTriangles[distMeshPoint.m_Res.iTriangleID].GetNormal();
        //if(myGrid.m_myTraits[id].distance > 0.02)
        {
          myGrid.m_pVertexCoords[id]= distMeshPoint.m_Res.m_vClosestPoint;
        }
      }
      else
      {
        myGrid.m_myTraits[id].distance = 1.0;    
        
        myGrid.m_myTraits[id].vNormal = VECTOR3(0,0,0);      
      }         
    }
  }
  
  writetimestep(0);
  
  //Write the grid to a file and measure the time
//   CUnstrGridr hgrid;
//   myWorld.m_vRigidBodies[0]->m_Map->ConvertToUnstructuredGrid(hgrid);
//   CVtkWriter writer; 
//   writer.WriteUnstr(hgrid,"output/distancemap.vtk");    
//   
  cleanup();

  return 1;
}
