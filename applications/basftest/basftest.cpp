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
CWorld myWorld;
CCollisionPipeline myPipeline;
CCollisionPipeline myPipelineRigid;
CRigidBodyMotion myMotion;
CSubdivisionCreator subdivider;
CBoundaryBoxr myBoundary;
CTimeControl myTimeControl;
CWorldParameters myParameters;

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
  myWorld.m_vRigidBodies.push_back(new CRigidBody());
  CRigidBody *body = myWorld.m_vRigidBodies.back();
  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_iShape    = CRigidBody::BVH;

  body->m_vCOM      = VECTOR3(0,0,0);

  body->m_InvInertiaTensor.SetZero();

  body->m_Restitution = 0.0;

  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
}

void initsimulation()
{

  CParticleFactory myFactory;
  CReader reader;
  
  //read the user defined configuration file
  reader.ReadParameters(string("start/data.TXT"),myParameters);
  myWorld = myFactory.ProduceFromParameters(myParameters);  
  

//0.866025404
  CRigidBody *body    = myWorld.m_vRigidBodies.back();  
	//initialize the simulation with some useful physical parameters
  //initialize the box shaped boundary

  body->m_bAffectedByGravity = false;
  body->m_dDensity  = 0;
  body->m_dVolume   = 0;
  body->m_dInvMass     = 0;
  body->m_vAngle    = VECTOR3(0,0,0);
  body->SetAngVel(VECTOR3(0,0,0));
  body->m_vVelocity = VECTOR3(0,0,0);
  body->m_vCOM      = VECTOR3(0,0,0);
  body->SetOrientation(body->m_vAngle);
  body->SetTransformationMatrix(body->GetQuaternion().GetMatrix());
  body->GenerateInvInertiaTensor();
  body->m_Restitution = 0.0;

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
	myGrid.InitMeshFromFile("meshes/meshD.tri");
	//myGrid.InitMeshFromFile("meshes/testgrid4x4.tri");
  //myGrid.InitCube(0,0,0,1,1,1);

	double dx=0;
	double dy=0;
	double dz=0;
	int    in=0;
	int i;
	
	myGrid.InitStdMesh();
	for(int i=0;i<0;i++)
	{
	  myGrid.Refine();
		std::cout<<"Generating Grid level"<<i+1<<std::endl;
	  std::cout<<"---------------------"<<std::endl;
	  std::cout<<"NVT="<<myGrid.m_iNVT<<" NEL="<<myGrid.m_iNEL<<std::endl;
	  myGrid.InitStdMesh();
	}

	CUnstrGrid::VertexIter vIter;

  std::cout<<"Computing FBM information and distance..."<<std::endl;
	//std::cout<<"Starting FMM..."<<std::endl;
	CDistOps3 op;

  std::cout<<"Computing FBM..."<<std::endl;
	for(vIter=myGrid.VertexBegin();vIter!=myGrid.VertexEnd();vIter++)
	{
		VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
		CVector3f vecf = CVector3f((*vIter).x,(*vIter).y,(*vIter).z);
		int in;
		int id = vIter.GetPos();
		myGrid.m_myTraits[vIter.GetPos()].iTag=0;
    myGrid.m_myTraits[vIter.GetPos()].distance=CMath<Real>::MAXREAL;
    myGrid.m_myTraits[vIter.GetPos()].dist2=CMath<Real>::MAXREAL;
		myGrid.m_myTraits[vIter.GetPos()].iX=0;
    std::vector<CRigidBody*>::iterator mIter;
    for(mIter=myWorld.m_vRigidBodies.begin();mIter!=myWorld.m_vRigidBodies.end();mIter++)
    {
      CRigidBody *pBody = *mIter;  
      CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
      C3DModel &model = object->m_Model;
      myGrid.m_myTraits[vIter.GetPos()].iTag = pBody->IsInBody(vec);
      if(myGrid.m_myTraits[vIter.GetPos()].iTag==1)break;
    }
	}
	std::cout<<"finished..."<<std::endl;

//  std::cout<<"Computing brute force distance..."<<std::endl;
  CPerfTimer timer;
 // timer.Start();
	//for(vIter=myGrid.VertexBegin();vIter!=myGrid.VertexEnd();vIter++)
	//{
	//	VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
 //   myGrid.m_myTraits[vIter.GetPos()].distance= op.BruteForceDistance(model,vec);
	//}
 // std::cout<<"finished..."<<"time brute force: "<<timer.GetTime()<<std::endl;
  //myGrid.VertexOrderXYZ();
  ////Get a pointer to the mesh object
  //CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(pBody->m_pShape);
  //std::cout<<"Computing B&B distance..."<<std::endl;
  //timer.Start();
  //for(vIter=myGrid.VertexBegin();vIter!=myGrid.VertexEnd();vIter++)
  //{
  //  VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
  //  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  //  myGrid.m_myTraits[vIter.GetPos()].dist2= distMeshPoint.ComputeDistance();
  //}
  //std::cout<<"finished..."<<"time B&B: "<<timer.GetTime()<<std::endl;

  //std::cout<<"Computing B&BCo distance..."<<std::endl;  
  //timer.Start();
  //
  //VECTOR3 vec = myGrid.m_pVertexCoords[myGrid.m_piVertexOrder[0]];
  //CDistanceMeshPoint<Real> distMeshPoint0(&pMeshObjectOrig->m_BVH,vec);
  //myGrid.m_myTraits[myGrid.m_piVertexOrder[0]].distance = distMeshPoint0.ComputeDistanceCoSqr(1000.0);
  //CDistanceMeshPointResult<Real> res;
  //res.iTriangleID = distMeshPoint0.m_Res.iTriangleID;
  //res.m_pBVH      = distMeshPoint0.m_Res.m_pBVH;
  //res.pNode       = distMeshPoint0.m_Res.pNode;
  //
  //for(int ive=1;ive<myGrid.m_iNVT;ive++)
  //{
  //  vec = myGrid.m_pVertexCoords[myGrid.m_piVertexOrder[ive]];
  //  // Compute distance from point to triangle
  //  CTriangle3<Real> &tri3 = res.pNode->m_Traits.m_vTriangles[res.iTriangleID];
  //  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  //  Real dist = distPointTri.ComputeDistance();
  //      
  //  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->m_BVH,vec);
  //  myGrid.m_myTraits[myGrid.m_piVertexOrder[ive]].distance = distMeshPoint.ComputeDistanceCoSqr(dist);
  //  
  //  res.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  //  res.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  //  res.pNode       = distMeshPoint.m_Res.pNode;
  //}  
  //std::cout<<"finished..."<<"time B&BCo: "<<timer.GetTime()<<std::endl;  
  
	//CRigidBody *pBody = myWorld.m_vRigidBodies[0];
	//CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(pBody->m_pShape);
	//C3DModel &model = object->m_Model;

	//std::cout<<"Starting FMM..."<<std::endl;
  //CDistanceFuncGridModel<Real> distGModel(&myGrid,model);
	//distGModel.ComputeDistance();

// 	vModels.push_back(model);
	CVtkWriter writer;
	writer.WriteUnstrXML(myGrid,"output/grid0.vtu");
  writer.WriteRigidBodies(myWorld.m_vRigidBodies,"output/model.vtk");

}


int main()
{
	using namespace std;
  
	initsimulation();
	startsimulation();

	return 1;
}

