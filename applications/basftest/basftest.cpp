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
#include <objloader.h>
#include <reader.h>
#include <perftimer.h>
#include <distancemeshpoint.h>
#include <collisionpipeline.h>

using namespace i3d;

CLog mylog;
CLog myPositionlog;
CLog myVelocitylog;
CLog myAngVelocitylog;
CUnstrGrid myGrid;
World myWorld;
CollisionPipeline myPipeline;
CollisionPipeline myPipelineRigid;
RigidBodyMotion myMotion;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
TimeControl myTimeControl;
WorldParameters myParameters;

int iPart = 20;
float xmin=0;
float ymin=0;
float zmin=0;
float xmax=2.0f;
float ymax=2.0f;
float zmax=6.0f;

void init()
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
  body->shape_    = RigidBody::BVH;

  body->com_      = VECTOR3(0,0,0);

  body->invInertiaTensor_.SetZero();

  body->restitution_ = 0.0;

  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
}

void initsimulation()
{

  ParticleFactory myFactory;
  CReader reader;
  
  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);
  
  myWorld = myFactory.produceFromFile(myParameters.bodyConfigurationFile_.c_str(),myTimeControl);
  

//0.866025404
  RigidBody *body    = myWorld.rigidBodies_.back();  
	//initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->affectedByGravity_ = false;
  body->density_  = 0;
  body->volume_   = 0;
  body->invMass_     = 0;
  body->angle_    = VECTOR3(0,0,0);
  body->setAngVel(VECTOR3(0,0,0));
  body->velocity_ = VECTOR3(0,0,0);
  body->com_      = VECTOR3(0,0,0);
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
  body->generateInvInertiaTensor();
  body->restitution_ = 0.0;

//   C3DModel model_out(pMeshObject->m_Model);
//   model_out.GenerateBoundingBox();
// 	for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
// 	{
//     model_out.m_vMeshes[i].m_matTransform =myWorld.m_vRigidBodies[0]->GetTransformationMatrix();
// 	  model_out.m_vMeshes[i].m_vOrigin =myWorld.m_vRigidBodies[0]->m_vCOM;
//     model_out.m_vMeshes[i].TransformModelWorld();
//     model_out.m_vMeshes[i].GenerateBoundingBox();
// 	}
// 
//   std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
//   CSubDivRessources myRessources(1,6,0,model_out.GetBox(),&pTriangles);
//   subdivider = CSubdivisionCreator(&myRessources);
//   pMeshObject->m_BVH.InitTree(&subdivider);
//   pMeshObject->m_BVH.GenTreeStatistics();

}

void startsimulation()
{
	//myGrid.InitMeshFromFile("meshes/RAPH1.tri");
	//myGrid.InitMeshFromFile("meshes/mesh4x4.tri");
  //myGrid.InitMeshFromFile("meshes/basfcut.tri");
	//myGrid.InitMeshFromFile("meshes/mesh.tri");
	//myGrid.InitMeshFromFile("meshes/testgrid4x4.tri");
  //myGrid.InitCube(0,0,0,1,1,1);

// 	double dx=0;
// 	double dy=0;
// 	double dz=0;
// 	int    in=0;
// 	int i;
// 	
// 	myGrid.InitStdMesh();
// 	for(int i=0;i<1;i++)
// 	{
// 	  myGrid.Refine();
// 		std::cout<<"Generating Grid level"<<i+1<<std::endl;
// 	  std::cout<<"---------------------"<<std::endl;
// 	  std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
// 	  myGrid.InitStdMesh();
// 	}
// 
// 	CVtkWriter writer;
// 	writer.WriteUnstrXML(myGrid,"output/grid0.vtu");
//   writer.WriteGrid2Tri(myGrid,"output/mesh.tri");

}


int main()
{
	using namespace std;
  
	initsimulation();
	startsimulation();

	return 1;
}

