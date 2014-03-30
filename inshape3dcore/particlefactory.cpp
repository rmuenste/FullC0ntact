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

ParticleFactory::ParticleFactory(World &world, WorldParameters &params)
{

  world_  = &world;

  params_ = &params;

  switch (params.bodyInit_) {

  case 0:
    world = produceFromParameters(params);
    buildSphereOfSpheres();
    break;
  default:
    break;
  }

}

void ParticleFactory::initRigidBodyParameters()
{

  for (auto &i : world_->rigidBodies_)
  {
    RigidBody *body = i;
    if (!body->affectedByGravity_)
      continue;
    body->density_ = params_->defaultDensity_;
    body->volume_ = body->shape_->getVolume();
    Real dmass = body->density_ * body->volume_;
    body->invMass_ = 1.0 / (body->density_ * body->volume_);
    body->angle_ = VECTOR3(0, 0, 0);
    body->setAngVel(VECTOR3(0, 0, 0));
    body->velocity_ = VECTOR3(0, 0, 0);
    body->com_ = VECTOR3(0, 0, 0);
    body->force_ = VECTOR3(0, 0, 0);
    body->torque_ = VECTOR3(0, 0, 0);
    body->restitution_ = 0.0;
    body->setOrientation(body->angle_);
    body->setTransformationMatrix(body->getQuaternion().GetMatrix());
    //calculate the inertia tensor
    //Get the inertia tensor
    body->generateInvInertiaTensor();
  }

}

void ParticleFactory::buildSphereOfSpheres()
{

  Real extends[3] = { params_->defaultRadius_, params_->defaultRadius_, 2.0*params_->defaultRadius_ };

  //add the desired number of particles
  addSpheres(world_->rigidBodies_, 515, params_->defaultRadius_); //515
  initRigidBodyParameters();

  int r = 5, ballr = 5;
  // inject a sphere of particles
  float pr = params_->defaultRadius_;
  float tr = pr + (pr*2.0f)*ballr;
  float pos[4], vel[4];
  pos[0] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[1] = 1.0f - tr;
  pos[2] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[3] = 0.0f;
  //  vel[0] = vel[1] = vel[2] = vel[3] = 0.0f;

  float spacing = pr*2.0f;
  unsigned int index = 1;
  for (int z = -r; z <= r; z++)
  {
    for (int y = -r; y <= r; y++)
    {
      for (int x = -r; x <= r; x++)
      {
        float dx = x*spacing;
        float dy = y*spacing;
        float dz = z*spacing;
        float l = sqrtf(dx*dx + dy*dy + dz*dz);
        float jitter = params_->defaultRadius_*0.01f;
        if ((l <= params_->defaultRadius_*2.0f*r) && (index < world_->rigidBodies_.size()))
        {
          VECTOR3 position(pos[0] + dx + (frand()*2.0f - 1.0f)*jitter,
            pos[1] + dy + (frand()*2.0f - 1.0f)*jitter,
            pos[2] + dz + (frand()*2.0f - 1.0f)*jitter);
          world_->rigidBodies_[index]->translateTo(position);
          world_->rigidBodies_[index]->color_ = position.x;
          index++;
        }
      }
    }
  }
}

World ParticleFactory::produceSpheres(int nspheres, Real rad)
{
  World myWorld;

  std::vector<RigidBody*>::iterator rIter;
  for(int i=0;i<nspheres;i++)
  {
    myWorld.rigidBodies_.push_back(new RigidBody());
  }

  //for(rIter=myWorld.rigidBodies_.begin();rIter!=myWorld.rigidBodies_.end();rIter++)
  for(auto &i : myWorld.rigidBodies_)
  {
    RigidBody *body = i;
    body->shape_ = new Spherer(VECTOR3(0,0,0),rad);
    body->shapeId_ = RigidBody::SPHERE;
  }
  return myWorld;
}

void ParticleFactory::addSpheres(std::vector<RigidBody*> &rigidBodies, int nSpheres, Real rad)
{
  for(int i=0;i<nSpheres;i++)
  {
    Real randRadius;
    if(i%2 == 0)
      randRadius = rad;
    else
      randRadius = rad;//0.75 * rad;
    RigidBody *body = new RigidBody();
    body->shape_ = new Spherer(VECTOR3(0,0,0),randRadius);
    body->shapeId_ = RigidBody::SPHERE;
    rigidBodies.push_back(body);
  }
}

void ParticleFactory::addMeshObjects(std::vector< RigidBody* >& rigidBodies, int nObjects, const char* fileName)
{

  for(int i=0;i<nObjects;i++)
  {        
    RigidBody *body = new RigidBody();    
    body->shape_ = new CMeshObject<Real>();
    CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
    pMeshObject->SetFileName(fileName);
    body->volume_   = body->shape_->getVolume();
    body->invMass_  = 0.0;

    GenericLoader Loader;
    Loader.readModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

    pMeshObject->m_Model.GenerateBoundingBox();
    for(auto &i : pMeshObject->m_Model.m_vMeshes)
    {
      i.GenerateBoundingBox();
    }

    body->shapeId_ = RigidBody::MESH;
    rigidBodies.push_back(body);
  }

}

void ParticleFactory::addCylinders(std::vector<RigidBody*> &rigidBodies, int nCylinders, Real extends[3])
{

  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};

  for(int i=0;i<nCylinders;i++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new Cylinderr(center, vUVW[2], extends[0], extends[2]);
    body->shapeId_ = RigidBody::CYLINDER;
    rigidBodies.push_back(body);
  }

}

void ParticleFactory::buildSpheres(std::vector<RigidBody*> &bodies, Real rad)
{

int id=0;
  for(auto &i : bodies)
  {
    RigidBody *body = i;
    body->shape_ = new Spherer(VECTOR3(0,0,0),rad);
    body->shapeId_ = RigidBody::SPHERE;
  }
}

void ParticleFactory::addBoxes(std::vector<RigidBody*> &rigidBodies, int nBoxes, Real extends[3])
{

  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};
  std::vector<RigidBody*>::iterator rIter;

  for(int i=0;i<nBoxes;i++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new OBB3r(center, vUVW, extends);
    body->shapeId_ = RigidBody::BOX;
    rigidBodies.push_back(body);
  }

}

World ParticleFactory::produceBoxes(int iCount, Real extends[3])
{

  World myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};

  for(int i=0;i<iCount;i++)
  {
    myWorld.rigidBodies_.push_back(new RigidBody());
  }

  for(auto &i : myWorld.rigidBodies_)
  {
    RigidBody *body = i;
    body->shape_ = new OBB3r(center, vUVW, extends);
    body->shapeId_ = RigidBody::BOX;
  }

  return myWorld;

}

World ParticleFactory::produceCylinders(int nCylinders, Real extends[3])
{

  World myWorld;
  VECTOR3 center(0,0,0);
  VECTOR3 vUVW[3] = {VECTOR3(1,0,0),VECTOR3(0,1,0),VECTOR3(0,0,1)};

  for(int i=0;i<nCylinders;i++)
  {
    myWorld.rigidBodies_.push_back(new RigidBody());
  }

  for(auto &i : myWorld.rigidBodies_)
  {
    RigidBody *body = i;
    body->shape_ = new Cylinderr(center, vUVW[2], extends[0], extends[2]);
    body->shapeId_ = RigidBody::CYLINDER;
  }

  return myWorld;

}

World ParticleFactory::produceTubes(const char* fileName)
{
	CTubeLoader Loader;
	World myDomain;
	RigidBody *body = new RigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.ReadModelFromFile(&pMesh->m_Model,fileName);
	pMesh->m_Model.GenerateBoundingBox();
	for(auto &i : pMesh->m_Model.m_vMeshes)
	{
		i.GenerateBoundingBox();
	}
	body->shape_ = pMesh;
	body->shapeId_ = RigidBody::BVH;
	myDomain.rigidBodies_.push_back(body);
	return myDomain;
}

World ParticleFactory::produceMesh(const char* fileName)
{
	GenericLoader Loader;
	World myDomain;
	RigidBody *body = new RigidBody();
	CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	Loader.readModelFromFile(&pMesh->m_Model,fileName);
	pMesh->m_Model.GenerateBoundingBox();
	pMesh->SetFileName(fileName);
	for(auto &i : pMesh->m_Model.m_vMeshes)
	{
		i.GenerateBoundingBox();
	}
	body->shape_ = pMesh;
	body->shapeId_ = RigidBody::MESH;
	myDomain.rigidBodies_.push_back(body);
	return myDomain;
}

World ParticleFactory::produceFromFile(const char* fileName, TimeControl &timeControl)
{
	World myWorld;
	
	//link the timeControl to the newly created world
	myWorld.timeControl_ = &timeControl;
	
	RigidBodyIO io;
	io.read(myWorld,fileName);

	return myWorld;
}

World ParticleFactory::produceFromParameters(WorldParameters &param)  
{

  World myWorld;

  for(int i=0;i<param.bodies_;i++)
  {
    BodyStorage *sBody = &param.rigidBodies_[i];
    RigidBody *pBody = new RigidBody(sBody);
    myWorld.rigidBodies_.push_back(pBody);
  }  
  
  return myWorld;

}

void ParticleFactory::addFromDataFile(WorldParameters &param, World *world)
{

  for(int i=0;i<param.bodies_;i++)
  {
    BodyStorage *bluePrint = &param.rigidBodies_[i];
    RigidBody *body = new RigidBody(bluePrint);
    world->rigidBodies_.push_back(body);
  }
  
}

World ParticleFactory::produceFromDeformParameters(DeformParameters& param)
{

  World myWorld;

  for(int i=0;i<param.rigidBodies_.size();i++)
  {
    BodyStorage *sBody = &param.rigidBodies_[i];
    RigidBody *pBody = new RigidBody(sBody);
    myWorld.rigidBodies_.push_back(pBody);
  }

  std::cout<<myWorld<<std::endl;

  return myWorld;

}


}
