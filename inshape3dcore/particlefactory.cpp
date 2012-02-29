/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "particlefactory.h"
#include <tubeloader.h>
#include <world.h>
#include <genericloader.h>
#include <3dmodel.h>
#include <string>
#include <sphere.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <cylinder.h>

namespace i3d {

CWorld CParticleFactory::ProduceSpheres(int iCount, Real rad)
{
  CWorld myWorld;

  std::vector<CRigidBody*>::iterator rIter;
  for(int i=0;i<iCount;i++)
  {
    myWorld.m_vRigidBodies.push_back(new CRigidBody());
  }

  for(rIter=myWorld.m_vRigidBodies.begin();rIter!=myWorld.m_vRigidBodies.end();rIter++)
  {
    CRigidBody *body = *rIter;
    body->m_pShape = new CSpherer(VECTOR3(0,0,0),rad);
    body->m_iShape = CRigidBody::SPHERE;
  }
  return myWorld;
}

void CParticleFactory::AddSpheres(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real rad)
{
  std::vector<CRigidBody*>::iterator rIter;
  for(int i=0;i<iCount;i++)
  {
    CRigidBody *body = new CRigidBody();
    body->m_pShape = new CSpherer(VECTOR3(0,0,0),rad);
    body->m_iShape = CRigidBody::SPHERE;
    vRigidBodies.push_back(body);
  }
}

void CParticleFactory::AddCylinders(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real extends[3])
{

  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
  std::vector<CRigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    CRigidBody *body = new CRigidBody();
    //const CVector3<T> &center, const CVector3<T> u, T radius, T h2
    body->m_pShape = new CCylinderr(center, vUVW[2], extends[0], extends[2]);
    body->m_iShape = CRigidBody::CYLINDER;
    vRigidBodies.push_back(body);
  }

}

void CParticleFactory::BuildSpheres(std::vector<CRigidBody*> &vBodies, Real dRad)
{
	std::vector<CRigidBody*>::iterator rIter;
	
	int id=0;
	for(rIter=vBodies.begin();rIter!=vBodies.end();rIter++)
	{
		CRigidBody *body = *rIter;
		body->m_pShape = new CSpherer(VECTOR3(0,0,0),dRad);
		body->m_iShape = CRigidBody::SPHERE;
	}
}

void CParticleFactory::AddBoxes(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real extends[3])
{
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
	std::vector<CRigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    CRigidBody *body = new CRigidBody();
    body->m_pShape = new COBB3r(center, vUVW, extends);
    body->m_iShape = CRigidBody::BOX;
    vRigidBodies.push_back(body);
  }

}

CWorld CParticleFactory::ProduceBoxes(int iCount, Real extends[3])
{
  CWorld myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
	std::vector<CRigidBody*>::iterator rIter;

	for(int i=0;i<iCount;i++)
	{
		myWorld.m_vRigidBodies.push_back(new CRigidBody());
	}

	for(rIter=myWorld.m_vRigidBodies.begin();rIter!=myWorld.m_vRigidBodies.end();rIter++)
	{
		CRigidBody *body = *rIter;
		body->m_pShape = new COBB3r(center, vUVW, extends);
    body->m_iShape = CRigidBody::BOX;
	}
  return myWorld;
}

CWorld CParticleFactory::ProduceCylinders(int iCount, Real extends[3])
{
  CWorld myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
  std::vector<CRigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    myWorld.m_vRigidBodies.push_back(new CRigidBody());
  }

  for(rIter=myWorld.m_vRigidBodies.begin();rIter!=myWorld.m_vRigidBodies.end();rIter++)
  {
    CRigidBody *body = *rIter;
    body->m_pShape = new CCylinderr(center, vUVW[2], extends[0], extends[2]);
    body->m_iShape = CRigidBody::CYLINDER;
  }
  return myWorld;
}

CWorld CParticleFactory::ProduceTube()
{
	CGenericLoader Loader;
	CWorld myDomain;
	std::vector<C3DModel> &Models = myDomain.m_vParticles;
	std::cout<<"Loading object file tube1.3DS"<<std::endl;
	C3DModel Model;
	Loader.ReadModelFromFile(&Model,(std::string("tube1.3DS")).c_str());
	Model.GenerateBoundingBox();
  Model.m_vMeshes[0].SetTransformation(MATRIX3X3::GenIdentity());
	Model.m_dRadius = 0.125;
	Models.push_back(Model);
	return myDomain;
}

CWorld CParticleFactory::ProduceTubes(const char* strFileName)
{
	CTubeLoader Loader;
	CWorld myDomain;
	CRigidBody *body = new CRigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,strFileName);
	pMesh->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->m_pShape = pMesh;
	body->m_iShape = CRigidBody::BVH;
	myDomain.m_vRigidBodies.push_back(body);
	return myDomain;
}

CWorld CParticleFactory::ProduceMesh(const char* strFileName)
{
	CGenericLoader Loader;
	CWorld myDomain;
	CRigidBody *body = new CRigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,strFileName);
	pMesh->m_Model.GenerateBoundingBox();
  pMesh->SetFileName(strFileName);
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->m_pShape = pMesh;
	body->m_iShape = CRigidBody::MESH;
	myDomain.m_vRigidBodies.push_back(body);
	return myDomain;
}

CWorld CParticleFactory::ProduceMixer()
{
	CGenericLoader Loader;
	CGenericLoader Loader1;
	CGenericLoader Loader2;
	CWorld myDomain;
	CRigidBody *body = new CRigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,"meshes/piece_scaledyz0.obj");
	pMesh->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->m_pShape = pMesh;
	body->m_iShape = CRigidBody::BVH;
	myDomain.m_vRigidBodies.push_back(body);

	CRigidBody *body1 = new CRigidBody();
	CMeshObject<Real> *pMesh1= new CMeshObject<Real>();
	Loader1.ReadModelFromFile(&pMesh1->m_Model,"meshes/piece_scaledyz1.obj");
	pMesh1->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh1->m_Model.m_vMeshes.size();i++)
	{
		pMesh1->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body1->m_pShape = pMesh1;
	body1->m_iShape = CRigidBody::BVH;
	myDomain.m_vRigidBodies.push_back(body1);

	CRigidBody *body2 = new CRigidBody();
	CMeshObject<Real> *pMesh2= new CMeshObject<Real>();
	Loader2.ReadModelFromFile(&pMesh2->m_Model,"meshes/piece_scaledyz2.obj");
	pMesh2->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh2->m_Model.m_vMeshes.size();i++)
	{
		pMesh2->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body2->m_pShape = pMesh2;
	body2->m_iShape = CRigidBody::BVH;
	myDomain.m_vRigidBodies.push_back(body2);

	return myDomain;

}

CWorld CParticleFactory::Produce2RigidBodies(void)
{
	CWorld myDomain;
	//VECTOR3 center(0,0,0);
	//VECTOR3 axis[3];
	//Real extend[3];
	//axis[0]=VECTOR3(1,0,0);
	//axis[1]=VECTOR3(0,1,0);
	//axis[2]=VECTOR3(0,0,1);
	//extend[0]=1.f;
	//extend[1]=1.f;
	//extend[2]=1.f;
	//myDomain.m_vRigidBodies.push_back(new CRigidBody(VECTOR3(0,0,0),1.f,1.f,1.f,VECTOR3(0,0,0),2));
	//myDomain.m_vRigidBodies[0]->m_bBox=COBB3r(center,axis,extend);
	//myDomain.m_vRigidBodies.push_back(new CRigidBody(VECTOR3(0,0.0,0),1.f,1.f,1.f,VECTOR3(0,0,0),2));
	//center=VECTOR3(0,-2.25,0);
	//myDomain.m_vRigidBodies[1]->m_bBox=COBB3r(center,axis,extend);
	return myDomain;	
}

CWorld  CParticleFactory::ProduceSphericalWithObstacles(int iCount)
{
	CGenericLoader Loader;
	CWorld myDomain;

	VECTOR3 center(0,0,0);
	Real rad = (Real)0.075;
	for(int i=0;i<iCount;i++)
		myDomain.m_vRigidBodies.push_back(new CRigidBody());

	std::vector<CRigidBody*>::iterator rIter;
	for(rIter=myDomain.m_vRigidBodies.begin();rIter!=myDomain.m_vRigidBodies.end();rIter++)
	{
		CRigidBody *body = *rIter;
		body->m_pShape = new CSpherer(VECTOR3(0,0,0),rad);
		body->m_iShape = CRigidBody::SPHERE;
	}

	CGenericLoader Loader1;
	CGenericLoader Loader2;
	CGenericLoader Loader3;
	C3DModel ModelBridge1;
	C3DModel ModelBridge2;
	C3DModel ModelBridge3;

	//std::cout<<"Loading object file obstacles1.obj"<<std::endl;	
	Loader1.ReadModelFromFile(&ModelBridge1,"meshes/obstacles1.obj");
	Loader2.ReadModelFromFile(&ModelBridge2,"meshes/obstacles2.obj");
	Loader3.ReadModelFromFile(&ModelBridge3,"meshes/obstacles3.obj");

	ModelBridge1.GenerateBoundingBox();
	ModelBridge2.GenerateBoundingBox();
	ModelBridge3.GenerateBoundingBox();
	ModelBridge1.m_vMeshes[0].GenerateBoundingBox();
	ModelBridge2.m_vMeshes[0].GenerateBoundingBox();
	ModelBridge3.m_vMeshes[0].GenerateBoundingBox();

	myDomain.m_vSolids.push_back(ModelBridge1);
	myDomain.m_vSolids.push_back(ModelBridge2);
	myDomain.m_vSolids.push_back(ModelBridge3);

	//std::cout<<"Obstacles loaded successfully..."<<std::endl;	
	
	return myDomain;
}

CWorld CParticleFactory::Produce2RigidBodies3(void)
{
	CWorld myWorld;
	VECTOR3 center(0,0,0);
	VECTOR3 centerWorld(1,1,1);
	VECTOR3 axis[3];
	Real extend[3];
	axis[0]=VECTOR3(1,0,0);
	axis[1]=VECTOR3(0,1,0);
	axis[2]=VECTOR3(0,0,1);
	extend[0]=0.25f;
	extend[1]=0.25f;
	extend[2]=0.25f;

	CRigidBody *body = new CRigidBody();
	body->m_pShape = new COBB3r(center,axis,extend);
  body->m_iShape = CRigidBody::BOX;
  myWorld.m_vRigidBodies.push_back(body);

	return myWorld;
}

CWorld CParticleFactory::ProduceFromFile(const char* strFileName, CTimeControl &timeControl)
{
	CWorld myWorld;
	
	//link the timeControl to the newly created world
  myWorld.m_pTimeControl = &timeControl;
	
	CRigidBodyIO rbIO;
	rbIO.Read(myWorld,strFileName);

	return myWorld;
}

CWorld CParticleFactory::ProduceFromParameters(CWorldParameters &param)  
{
  CWorld myWorld;

  for(int i=0;i<param.m_iBodies;i++)
  {
    sRigidBody *sBody = &param.m_vRigidBodies[i];
    CRigidBody *pBody = new CRigidBody(sBody);
    myWorld.m_vRigidBodies.push_back(pBody);
  }  
  
  //std::cout<<myWorld<<std::endl;

  return myWorld;
}

CWorld CParticleFactory::ProduceFromDeformParameters(CDeformParameters& param)
{
  CWorld myWorld;

  for(int i=0;i<param.m_vRigidBodies.size();i++)
  {
    sRigidBody *sBody = &param.m_vRigidBodies[i];
    CRigidBody *pBody = new CRigidBody(sBody);
    myWorld.m_vRigidBodies.push_back(pBody);
  }  
  
  std::cout<<myWorld<<std::endl;

  return myWorld;
}


}
