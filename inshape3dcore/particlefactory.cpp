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

World ParticleFactory::ProduceSpheres(int iCount, Real rad)
{
  World myWorld;

  std::vector<RigidBody*>::iterator rIter;
  for(int i=0;i<iCount;i++)
  {
    myWorld.rigidBodies_.push_back(new RigidBody());
  }

  for(rIter=myWorld.rigidBodies_.begin();rIter!=myWorld.rigidBodies_.end();rIter++)
  {
    RigidBody *body = *rIter;
    body->shape_ = new CSpherer(VECTOR3(0,0,0),rad);
    body->shapeId_ = RigidBody::SPHERE;
  }
  return myWorld;
}

void ParticleFactory::addSpheres(std::vector<RigidBody*> &vRigidBodies, int iCount, Real rad)
{
  std::vector<RigidBody*>::iterator rIter;
  for(int i=0;i<iCount;i++)
  {
    Real randRadius;
    if(i%2 == 0)
      randRadius = rad;
    else
      randRadius = rad;//0.75 * rad;
    RigidBody *body = new RigidBody();
    body->shape_ = new CSpherer(VECTOR3(0,0,0),randRadius);
    body->shapeId_ = RigidBody::SPHERE;
    vRigidBodies.push_back(body);
  }
}

void ParticleFactory::addMeshObjects(std::vector< RigidBody* >& vRigidBodies, int iCount, const char* strFileName)
{
  std::vector<RigidBody*>::iterator rIter;
  for(int i=0;i<iCount;i++)
  {        
    RigidBody *body = new RigidBody();    
    body->shape_ = new CMeshObject<Real>();
    CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
    pMeshObject->SetFileName(strFileName);
    body->volume_   = body->shape_->Volume();
    body->invMass_  = 0.0;

    CGenericLoader Loader;
    Loader.ReadModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

    pMeshObject->m_Model.GenerateBoundingBox();
    for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
    {
      pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
    }
        
    body->shapeId_ = RigidBody::MESH;
    vRigidBodies.push_back(body);                
  }
  
}

void ParticleFactory::addCylinders(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3])
{

  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
  std::vector<RigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    RigidBody *body = new RigidBody();
    //const CVector3<T> &center, const CVector3<T> u, T radius, T h2
    body->shape_ = new Cylinderr(center, vUVW[2], extends[0], extends[2]);
    body->shapeId_ = RigidBody::CYLINDER;
    vRigidBodies.push_back(body);
  }

}

void ParticleFactory::buildSpheres(std::vector<RigidBody*> &vBodies, Real dRad)
{
	std::vector<RigidBody*>::iterator rIter;
	
	int id=0;
	for(rIter=vBodies.begin();rIter!=vBodies.end();rIter++)
	{
		RigidBody *body = *rIter;
		body->shape_ = new CSpherer(VECTOR3(0,0,0),dRad);
		body->shapeId_ = RigidBody::SPHERE;
	}
}

void ParticleFactory::addBoxes(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3])
{
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
	std::vector<RigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new COBB3r(center, vUVW, extends);
    body->shapeId_ = RigidBody::BOX;
    vRigidBodies.push_back(body);
  }

}

World ParticleFactory::produceBoxes(int iCount, Real extends[3])
{
  World myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
	std::vector<RigidBody*>::iterator rIter;

	for(int i=0;i<iCount;i++)
	{
		myWorld.rigidBodies_.push_back(new RigidBody());
	}

	for(rIter=myWorld.rigidBodies_.begin();rIter!=myWorld.rigidBodies_.end();rIter++)
	{
		RigidBody *body = *rIter;
		body->shape_ = new COBB3r(center, vUVW, extends);
    body->shapeId_ = RigidBody::BOX;
	}
  return myWorld;
}

World ParticleFactory::produceCylinders(int iCount, Real extends[3])
{
  World myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
  std::vector<RigidBody*>::iterator rIter;

  for(int i=0;i<iCount;i++)
  {
    myWorld.rigidBodies_.push_back(new RigidBody());
  }

  for(rIter=myWorld.rigidBodies_.begin();rIter!=myWorld.rigidBodies_.end();rIter++)
  {
    RigidBody *body = *rIter;
    body->shape_ = new Cylinderr(center, vUVW[2], extends[0], extends[2]);
    body->shapeId_ = RigidBody::CYLINDER;
  }
  return myWorld;
}

World ParticleFactory::produceTubes(const char* strFileName)
{
	CTubeLoader Loader;
	World myDomain;
	RigidBody *body = new RigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,strFileName);
	pMesh->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->shape_ = pMesh;
	body->shapeId_ = RigidBody::BVH;
	myDomain.rigidBodies_.push_back(body);
	return myDomain;
}

World ParticleFactory::produceMesh(const char* strFileName)
{
	CGenericLoader Loader;
	World myDomain;
	RigidBody *body = new RigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,strFileName);
	pMesh->m_Model.GenerateBoundingBox();
  pMesh->SetFileName(strFileName);
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->shape_ = pMesh;
	body->shapeId_ = RigidBody::MESH;
	myDomain.rigidBodies_.push_back(body);
	return myDomain;
}

World ParticleFactory::produceMixer()
{
	CGenericLoader Loader;
	CGenericLoader Loader1;
	CGenericLoader Loader2;
	World myDomain;
	RigidBody *body = new RigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,"meshes/piece_scaledyz0.obj");
	pMesh->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh->m_Model.m_vMeshes.size();i++)
	{
		pMesh->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body->shape_ = pMesh;
	body->shapeId_ = RigidBody::BVH;
	myDomain.rigidBodies_.push_back(body);

	RigidBody *body1 = new RigidBody();
	CMeshObject<Real> *pMesh1= new CMeshObject<Real>();
	Loader1.ReadModelFromFile(&pMesh1->m_Model,"meshes/piece_scaledyz1.obj");
	pMesh1->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh1->m_Model.m_vMeshes.size();i++)
	{
		pMesh1->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body1->shape_ = pMesh1;
	body1->shapeId_ = RigidBody::BVH;
	myDomain.rigidBodies_.push_back(body1);

	RigidBody *body2 = new RigidBody();
	CMeshObject<Real> *pMesh2= new CMeshObject<Real>();
	Loader2.ReadModelFromFile(&pMesh2->m_Model,"meshes/piece_scaledyz2.obj");
	pMesh2->m_Model.GenerateBoundingBox();
	for(int i=0;i< pMesh2->m_Model.m_vMeshes.size();i++)
	{
		pMesh2->m_Model.m_vMeshes[i].GenerateBoundingBox();
	}
	body2->shape_ = pMesh2;
	body2->shapeId_ = RigidBody::BVH;
	myDomain.rigidBodies_.push_back(body2);

	return myDomain;

}

World ParticleFactory::produce2RigidBodies(void)
{
	World myDomain;
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

World ParticleFactory::produce2RigidBodies3(void)
{
	World myWorld;
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

	RigidBody *body = new RigidBody();
	body->shape_ = new COBB3r(center,axis,extend);
  body->shapeId_ = RigidBody::BOX;
  myWorld.rigidBodies_.push_back(body);

	return myWorld;
}

World ParticleFactory::produceFromFile(const char* strFileName, TimeControl &timeControl)
{
	World myWorld;
	
	//link the timeControl to the newly created world
        myWorld.timeControl_ = &timeControl;
	
	CRigidBodyIO rbIO;
	rbIO.Read(myWorld,strFileName);

	return myWorld;
}

World ParticleFactory::produceFromParameters(WorldParameters &param)  
{
  World myWorld;

  for(int i=0;i<param.m_iBodies;i++)
  {
    sRigidBody *sBody = &param.m_vRigidBodies[i];
    RigidBody *pBody = new RigidBody(sBody);
    myWorld.rigidBodies_.push_back(pBody);
  }  
  
  //std::cout<<myWorld<<std::endl;

  return myWorld;
}

void ParticleFactory::addFromDataFile(WorldParameters &param, World *pWorld)  
{

  for(int i=0;i<param.m_iBodies;i++)
  {
    sRigidBody *sBody = &param.m_vRigidBodies[i];
    RigidBody *pBody = new RigidBody(sBody);
    pWorld->rigidBodies_.push_back(pBody);
  }  
  
}

World ParticleFactory::produceFromDeformParameters(CDeformParameters& param)
{
  World myWorld;

  for(int i=0;i<param.m_vRigidBodies.size();i++)
  {
    sRigidBody *sBody = &param.m_vRigidBodies[i];
    RigidBody *pBody = new RigidBody(sBody);
    myWorld.rigidBodies_.push_back(pBody);
  }  
  
  std::cout<<myWorld<<std::endl;

  return myWorld;
}


}
