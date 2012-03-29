

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

double xmin=0;
double ymin=0;
double zmin=0;
double xmax=2.0f;
//double ymax=0.35f;
double ymax=1.0f;
double zmax=2.2f;
Real radius = Real(0.075);
int iReadGridFromFile = 0;

void addboundary()
{
	//initialize the box shaped boundary
	myWorld.m_vRigidBodies.push_back(new CRigidBody());
	CRigidBody *body = myWorld.m_vRigidBodies.back();
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
		body->m_Restitution = 0.5;
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
	Real drad = myWorld.m_vRigidBodies[0]->m_pShape->GetAABB().m_Extends[0];
	Real d    = 2.0 * drad;
	Real distbetween = 0.5 * drad;
	int perrow = myGrid.m_vMax.x/(distbetween+d);
	VECTOR3 pos(myGrid.m_vMin.x+drad+distbetween , myGrid.m_vMax.y/2.0, (myGrid.m_vMax.z/4.0)-d);
	myWorld.m_vRigidBodies[0]->TranslateTo(pos);
	pos.x+=d+distbetween;
	for(int i=1;i<myWorld.m_vRigidBodies.size();i++)
	{
		if((i)%(perrow) == 0)
		{
			//advance in y and reset x
			pos.x = myGrid.m_vMin.x+drad+distbetween;
			pos.z -= d+distbetween;
		}
		myWorld.m_vRigidBodies[i]->TranslateTo(pos);
		pos.x+=d+distbetween;
	}
}

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
	VECTOR3 pos(0.75, ystart, (0.075));
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
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,1.75,0);
  //myWorld.m_vRigidBodies[index]->m_vAngle=VECTOR3(0,0.75,0);
  myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(-1.9,0.0,0.0);
  //myWorld.m_vRigidBodies[index]->m_vVelocity=VECTOR3(0.0,0.0,0.0);
}

void createrestingtest()
{
	Real drad = myWorld.m_vRigidBodies[0]->m_pShape->GetAABB().m_Extends[0];
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
 			pyramidtest();
		}

		if(myParameters.m_iBodyInit == 3)
		{
 			createlineuptest();
		}

		if(myParameters.m_iBodyInit == 4)
		{
 			createrestingtest();
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

  //set the timestep
  myTimeControl.SetDeltaT(0.005);
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

  //set which type of rigid motion we are dealing with
  myMotion=CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = &myMotion;

  
  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;
  myWorld.m_dDensityMedium = myParameters.m_dDensityMedium;
  myWorld.m_bLiquidSolid   = (myParameters.m_iLiquidSolid == 1) ? true : false;

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

  //set which type of rigid motion we are dealing with
  myMotion=CRigidBodyMotion(&myWorld);

  //set the integrator in the pipeline
  myPipeline.m_pIntegrator = &myMotion;


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
  //indices.push_back(8);
  //indices.push_back(10);
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
}

int main()
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

  cout<<myWorld<<endl;
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
		myPipeline.StartPipeline();
		energy1=myWorld.GetTotalEnergy();
    cout<<"Energy after collision: "<<energy1<<endl;
    cout<<"Energy difference: "<<energy0-energy1<<endl;
		//if(dTimePassed >= myTimeControl.GetPreferredTimeStep())
		//{
			std::cout<<"Timestep finished... writing vtk."<<std::endl;
			writetimestep(iOut);
			std::cout<<"Finished writing vtk."<<std::endl;
			iOut++;
			dTimePassed = 0.f;
//		}
		myTimeControl.SetTime(simTime+myTimeControl.GetDeltaT());
		dTimePassed += myTimeControl.GetDeltaT();
	}//end for

  cleanup();

	return 1;
}
