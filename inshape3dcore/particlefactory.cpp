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
#include <world.h>
#include <genericloader.h>
#include <3dmodel.h>
#include <string>
#include <sphere.h>
#include <rigidbodyio.h>

#include <geom_config.hpp>

#include <meshobject.h>
#include <cylinder.h>
#include <vtkwriter.h>
#include <compoundbody.h>

#include <json.hpp>
#include <ode_config.hpp>

namespace i3d {

ParticleFactory::ParticleFactory(World &world, WorldParameters &params)
{

  world_  = &world;

  params_ = &params;

  switch (params.bodyInit_) {

  case 0:
    world = produceFromParameters(params);
    break;
  case 1:
    {
      world = produceFromParameters(params);
      buildSphereOfSpheres();
    }
    break;
  case 2:
    {
      world = produceFromParameters(params);
      initFromParticleFile();
    }
    break;
  case 3:
    {
      world = produceFromParameters(params);
      initSplitBottom();
    }
    break;
  case 4:
  {
    world = produceFromParameters(params);
    initPyramidTest();
    break;
  }
  case 5:
  {
    world = produceFromParameters(params);
    initBoxStack();
    break;
  }
  case 6:
  {
    world = produceFromParameters(params);
    initCompoundBodies();
    break;
  }
  case 7:
  {
    world = produceFromParameters(params);
    buildSphereOfCompounds();
    break;
  }
  case 8:
  {
    world = produceFromParameters(params);
    initDemSphereTest();
    break;
  }
  case 9:
  {
    world = produceFromParameters(params);
    buildTorqueTest();
    break;
  }
  case 10:
  {
    world = produceFromParameters(params);
    buildHopperTest();
    break;
  }
  case 11:
  {
    world = produceFromParameters(params);
    readBinaryFile();
    break;
  }
  case 12:
  {
    world = produceFromParameters(params);
    buildBoxGrainTest();
    break;
  }
  case 13:
  {
    world = produceFromParameters(params);
    initDemSpherePlaneTest();
    break;
  }
  case 14:
  {
    world = produceFromParameters(params);
    grainFields();
    break;
  }
  case 15:
  {
    world = produceFromParameters(params);
    stictionTest();
    break;
  }
  case 16:
  {
    world = produceFromParameters(params);
    initCompoundBodies2();
    break;
  }
  case 17:
  {
    world = produceFromParameters(params);
    meshCowStack();
    break;
  }
  case 18:
  {
    world = produceFromParameters(params);
    initPyramidSticks();
    break;
  }
  case 19:
  {
    world = produceFromParameters(params);
    initTowers();
    break;
  }
  case 20:
  {
    world = produceFromParameters(params);
    meshDogStack();
    break;
  }
  case 21:
  {
    world = produceFromParameters(params);
    complexParticles();
    break;
  }
  case 22:
  {
    world = produceFromParameters(params);
    softBodyParticles();
    break;
  }
  default:
    break;
  }

}

void ParticleFactory::readBinaryFile()
{
  RigidBodyIO io;
  World &world = *world_;
  unsigned offset = world.rigidBodies_.size();
  io.read(world, params_->solutionFile_.c_str());
  offset = world.rigidBodies_.size();

  for (auto &rb : world_->rigidBodies_)
  {

    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

}

void ParticleFactory::grainFields()
{
  RigidBodyIO io;
  World &world = *world_;
  unsigned offset = world.rigidBodies_.size();
  io.read(world, params_->solutionFile_.c_str());
  offset = world.rigidBodies_.size();

  for (auto &rb : world_->rigidBodies_)
  {

    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    //body->com_.y -= 1.2;
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  io.read(world, params_->solutionFile_.c_str());

  for (unsigned i = offset; i < world_->rigidBodies_.size(); i++)
  {
    RigidBody* rb = world_->rigidBodies_[i];
    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    body->com_.y -= 2.0;
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  offset = world.rigidBodies_.size();
  io.read(world, params_->solutionFile_.c_str());

  for (unsigned i = offset; i < world_->rigidBodies_.size(); i++)
  {
    RigidBody* rb = world_->rigidBodies_[i];
    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    body->com_.y += 2.0;
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  offset = world.rigidBodies_.size();
  io.read(world, params_->solutionFile_.c_str());

  for (unsigned i = offset; i < world_->rigidBodies_.size(); i++)
  {
    RigidBody* rb = world_->rigidBodies_[i];
    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    body->com_.y += 4.0;
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  offset = world.rigidBodies_.size();
  io.read(world, params_->solutionFile_.c_str());

  for (unsigned i = offset; i < world_->rigidBodies_.size(); i++)
  {
    RigidBody* rb = world_->rigidBodies_[i];
    if (rb->shapeId_ != RigidBody::COMPOUND)continue;
    rb->affectedByGravity_ = true;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->generateInvInertiaTensor();
    body->com_.y -= 4.0;
    for (auto &comp : body->rigidBodies_)
    {
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }


}

void ParticleFactory::initFromParticleFile()
{

  std::cout << "Function initFromParticleFile is deprecated: " << __FILE__ << " line: " << __LINE__<< std::endl; 
  std::exit(EXIT_FAILURE);

//  CVtkWriter writer;
//  std::vector<VECTOR3> points;
//  std::vector<Real> rho;
//  std::vector<Real> radii;
//  writer.readVTKParticles("meshes/particle_in.vtk",points,rho,radii);
//
//  for(unsigned i=0;i<points.size();i++)
//  {
//    RigidBody *body = new RigidBody();
//    body->shape_ = new Spherer(VECTOR3(0,0,0),radii[i]);
//    body->shape_ = new Spherer(VECTOR3(0, 0, 0), 0.001);
//    body->com_ = points[i];
//    body->density_ = rho[i];
//    body->shapeId_ = RigidBody::SPHERE;
//    world_->rigidBodies_.push_back(body);
//  }
//
//  for (auto &i : world_->rigidBodies_)
//  {
//    RigidBody *body = i;
//    if (!body->affectedByGravity_)
//      continue;
//    body->volume_ = body->shape_->getVolume();
//    Real dmass = body->density_ * body->volume_;
//    body->invMass_ = 1.0 / (dmass);
//    body->angle_ = VECTOR3(0, 0, 0);
//    body->setAngVel(VECTOR3(0, 0, 0));
//    body->velocity_ = VECTOR3(0, 0, 0);
//    body->force_ = VECTOR3(0, 0, 0);
//    body->torque_ = VECTOR3(0, 0, 0);
//    body->restitution_ = 0.0;
//    body->setOrientation(body->angle_);
//    body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//    //calculate the inertia tensor
//    //Get the inertia tensor
//    body->generateInvInertiaTensor();
//  }
//
//  int perRow=9;
//  int columns=7;
//  for(int i=0;i<columns*perRow;i++)
//  {
//    CompoundBody *body = new CompoundBody();
//    body->density_ = 8522.0;
//
//    //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
//    //from previous timestep is stored in these
//
//    body->angle_=VECTOR3(0, 0.0, 0);
//    body->setOrientation(body->angle_);
//    body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
//    body->setTransformationMatrix(body->quat_.GetMatrix());
//    //addSpheres2(body->rigidBodies_, 3, 0.05);
//
//    addSpheres2(body->rigidBodies_, 4 , 0.05);
//    body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.2);
//    body->rigidBodies_[1]->com_=VECTOR3(0.025,0.0,0.2);
//    body->rigidBodies_[2]->com_=VECTOR3(0.025,0.0,0.175);
//    body->rigidBodies_[3]->com_=VECTOR3(0.00,0.0,0.175);
//
//    world_->rigidBodies_.push_back(body);
//
//    body->generateInvInertiaTensor();
//  }
//
//  for (auto &rb : world_->rigidBodies_)
//  {
//
//    if(rb->shapeId_!=RigidBody::COMPOUND)continue;
//
//    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
//    body->setVolume();
//    body->setInvMass();
//
//    for (auto &comp : body->rigidBodies_)
//    {
//      body->com_ += comp->com_;
//    }
//    body->com_ *= 1.0/body->rigidBodies_.size();
//  }
//
//  for (auto &rb : world_->rigidBodies_)
//  {
//
//    if(rb->shapeId_!=RigidBody::COMPOUND)continue;
//
//    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
//    for (auto &comp : body->rigidBodies_)
//    {
//      comp->com_ = comp->com_ - body->com_;
//      comp->transform_.setOrigin(body->com_);
//      comp->transform_.setMatrix(body->getTransformationMatrix());
//    }
//  }
//
//  CompoundBody *body = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
//  VECTOR3 position = VECTOR3(-1.0+body->getBoundingSphereRadius()+5.0*body->getBoundingSphereRadius(),-1.0+4.0*body->getBoundingSphereRadius(),0.75);
//
//  int index=1;
//  for(int col=0;col<columns;col++)
//  {
//    for(int row=0;row<perRow;row++)
//    {
//      //world_->rigidBodies_[index]->translateTo(position);
//      CompoundBody *c = dynamic_cast<CompoundBody*>(world_->rigidBodies_[index]);
//      c->com_ = position;
//
//      for (auto &comp : c->rigidBodies_)
//      {
//        comp->transform_.setOrigin(c->com_);
//        comp->transform_.setMatrix(c->getTransformationMatrix());
//      }
//
//      world_->rigidBodies_[index]->color_ = position.x;
//      position.x += 2.0*body->getBoundingSphereRadius();
//      index++;
//    }
//    position.y += 2.0*body->getBoundingSphereRadius();
//    position.x =-1.0+body->getBoundingSphereRadius()+4.0*body->getBoundingSphereRadius();
//  }

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
  //addSpheres(world_->rigidBodies_, 10000, params_->defaultRadius_); //515
  //addSpheres(world_->rigidBodies_, 1000, params_->defaultRadius_); //515
  addSpheres(world_->rigidBodies_, 515, params_->defaultRadius_); //515
  initRigidBodyParameters();
  //world_->rigidBodies_.back()->translateTo(VECTOR3(-0.1, 0.4, -0.94));

  int r = 15, ballr = 15;
  // inject a sphere of particles
  float pr = params_->defaultRadius_;
  float tr = pr + (pr*2.0f)*ballr;
  float pos[4];
  pos[0] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[1] = 1.0f - tr;
  pos[2] = 0.5f + tr + frand()*(2.0f - tr*2.0f);
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

void ParticleFactory::meshCowStack()
{

  int offset = world_->rigidBodies_.size();
  for (int j = 0; j < 363; j++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new MeshObject<Real>();
    MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);

//    if (((double)rand() / (double)RAND_MAX) > 0.5)
//    {
//      //pMeshObject->SetFileName("meshes/swimmer_export.obj");
//      pMeshObject->SetFileName("meshes/cow.obj");
//    }
//    else
//      pMeshObject->SetFileName("meshes/cow.obj");

    pMeshObject->setFileName("meshes/dog.obj");

    body->shape_ = pMeshObject;
    body->shapeId_ = RigidBody::MESH;
    body->density_ = 2.5;

    if (pMeshObject->getFileName() == "meshes/swimmer_export.obj")
    {
      body->volume_ = 8.22e-3;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }
    else if (pMeshObject->getFileName() == "meshes/cow.obj")
    {
      body->volume_ = 0.01303;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }
    else if (pMeshObject->getFileName() == "meshes/dog.obj")
    {
      body->volume_ = 0.01303;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }

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
    body->generateInvInertiaTensor();

    //load model from file
    GenericLoader Loader;
    Loader.readModelFromFile(&pMeshObject->getModel(), pMeshObject->getFileName().c_str());

    pMeshObject->generateBoundingBox();

    Model3D model_out_0(pMeshObject->getModel());
    model_out_0.meshes_[0].com_ = VECTOR3(0, 0, 0);
    model_out_0.generateBoundingBox();
    model_out_0.meshes_[0].generateBoundingBox();
    std::vector<Triangle3r> pTriangles = model_out_0.genTriangleVector();

    CSubDivRessources myRessources_dm(1, 5, 0, model_out_0.getBox(), &pTriangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
    pMeshObject->initTree(subdivider_dm);

    world_->rigidBodies_.push_back(body);
    
  }

  int count = offset;

  Real drad = world_->rigidBodies_[count]->shape_->getAABB().extents_[world_->rigidBodies_[count]->shape_->getAABB().longestAxis()];

  Real d = 2.0 * drad;
  Real dz = 4.0 * drad;
  Real distbetween = 0.2 * drad;
  Real distbetweeny = drad;
  Real distbetweenz = 0.5 * drad;

  int perrowx = 11;
  int perrowy = 11;

  int numPerLayer = perrowx * perrowy;
  int layers = 3;
  int nTotal = numPerLayer * layers;

  Real ynoise = 0.1*drad;

  //add the desired number of particles
  std::cout << "Number of meshes: " << numPerLayer*layers << std::endl;
  VECTOR3 pos(params_->extents_[0] + 4.0 * drad + distbetween, params_->extents_[2] + 4.0 * drad + distbetween + ynoise, params_->extents_[4] + 20.0 * drad);

  for (int z = 0; z<layers; z++)
  {
    for (int j = 0; j<perrowy; j++)
    {
      for (int i = 0; i<perrowx; i++, count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x, pos.y + ynoise, pos.z);
        world_->rigidBodies_[count]->translateTo(bodypos);
        if (z == 1)
        {
          double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, radian, 0);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        else if (z == 0)
        {
          double radian = 1.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, 0, radian);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        pos.x += d + distbetween;
      }
      pos.x = params_->extents_[0] + 4.0 * drad + distbetween;
      pos.y += d + distbetween;
    }
    ynoise = -ynoise;
    pos.z += 2.0*d;
    pos.y = params_->extents_[2] + 4.0 * drad + distbetween + ynoise;
  }

//  RigidBody *body0 = new RigidBody();
//  body0->shape_ = new CMeshObject<Real>();
//  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body0->shape_);
//
//  pMeshObject->SetFileName("meshes/hopper1.obj");
//
//  body0->shape_ = pMeshObject;
//  body0->shapeId_ = RigidBody::MESH;
//  body0->density_ = 2.5;
//
//  if (pMeshObject->GetFileName() == "meshes/swimmer_export.obj")
//  {
//    body0->volume_ = 8.22e-3;
//    body0->invMass_ = 1.0 / (body0->density_ * body0->volume_);
//  }
//  else if (pMeshObject->GetFileName() == "meshes/cow.obj")
//  {
//    body0->volume_ = 0.01303;
//    body0->invMass_ = 1.0 / (body0->density_ * body0->volume_);
//  }
//  else
//  {
//    body0->volume_ = 0.0;
//    body0->invMass_ = 0.0;
//  }
//
//  body0->invMass_ = 0.0;
//  body0->angle_ = VECTOR3(0, 0, 0);
//  body0->setAngVel(VECTOR3(0, 0, 0));
//  body0->velocity_ = VECTOR3(0, 0, 0);
//  body0->com_ = VECTOR3(0, 0, 0);
//
//  body0->force_ = VECTOR3(0, 0, 0);
//  body0->torque_ = VECTOR3(0, 0, 0);
//  body0->restitution_ = 0.0;
//  body0->affectedByGravity_ = false;
//  body0->setOrientation(body0->angle_);
//  body0->setTransformationMatrix(body0->getQuaternion().GetMatrix());
//  //calculate the inertia tensor
//  body0->generateInvInertiaTensor();
//
//  //load model from file
//  GenericLoader Loader;
//  Loader.readModelFromFile(&pMeshObject->m_Model, pMeshObject->GetFileName().c_str());
//
//  pMeshObject->m_Model.GenerateBoundingBox();
//  for (unsigned i = 0; i< pMeshObject->m_Model.meshes_.size(); i++)
//  {
//    pMeshObject->m_Model.meshes_[i].generateBoundingBox();
//  }
//
//  Model3D model_out_0(pMeshObject->m_Model);
//  model_out_0.meshes_[0].com_ = VECTOR3(0, 0, 0);
//  model_out_0.GenerateBoundingBox();
//  model_out_0.meshes_[0].generateBoundingBox();
//  std::vector<Triangle3r> pTriangles = model_out_0.GenTriangleVector();
//
//  CSubDivRessources myRessources_dm(1, 4, 0, model_out_0.GetBox(), &pTriangles);
//  CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
//  pMeshObject->m_BVH.InitTree(&subdivider_dm);
//
//
//  VECTOR3 p = VECTOR3(0.0, 0.0, -1.4);
//  body0->translateTo(p);
//
//  world_->rigidBodies_.push_back(body0);

}

void ParticleFactory::meshDogStack()
{

  int offset = world_->rigidBodies_.size();
  for (int j = 0; j < 363; j++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new MeshObject<Real>();
    MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);

//    if (((double)rand() / (double)RAND_MAX) > 0.5)
//    {
//      //pMeshObject->SetFileName("meshes/swimmer_export.obj");
//      pMeshObject->SetFileName("meshes/cow.obj");
//    }
//    else
//      pMeshObject->SetFileName("meshes/cow.obj");

    pMeshObject->setFileName("meshes/dog_small.obj");

    body->shape_ = pMeshObject;
    body->shapeId_ = RigidBody::MESH;
    body->density_ = 2.5;

    if (pMeshObject->getFileName() == "meshes/swimmer_export.obj")
    {
      body->volume_ = 8.22e-3;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }
    else if (pMeshObject->getFileName() == "meshes/cow.obj")
    {
      body->volume_ = 0.01303;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }
    else if (pMeshObject->getFileName() == "meshes/dog.obj")
    {
      body->volume_ = 0.01303;
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }
    else if (pMeshObject->getFileName() == "meshes/dog_small.obj")
    {
      body->volume_ = 1.5e-7; // 94.0; //94 micro meter^3
      body->invMass_ = 1.0 / (body->density_ * body->volume_);
    }

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
    body->generateInvInertiaTensor();

    //load model from file
    GenericLoader Loader;
    Loader.readModelFromFile(&pMeshObject->getModel(), pMeshObject->getFileName().c_str());

    pMeshObject->generateBoundingBox();

    Model3D model_out_0(pMeshObject->getModel());
    model_out_0.meshes_[0].com_ = VECTOR3(0, 0, 0);
    model_out_0.generateBoundingBox();
    model_out_0.meshes_[0].generateBoundingBox();
    std::vector<Triangle3r> pTriangles = model_out_0.genTriangleVector();

    CSubDivRessources myRessources_dm(1, 5, 0, model_out_0.getBox(), &pTriangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
    pMeshObject->initTree(subdivider_dm);
    world_->rigidBodies_.push_back(body);

  }

  int count = offset;

  Real drad = world_->rigidBodies_[count]->shape_->getAABB().extents_[world_->rigidBodies_[count]->shape_->getAABB().longestAxis()];

  Real d = 2.0 * drad;
  Real dz = 4.0 * drad;
  Real distbetween = 0.2 * drad;
  Real distbetweeny = drad;
  Real distbetweenz = 0.5 * drad;

//  int perrowx = 11;
//  int perrowy = 11;

  int perrowx = 11;
  int perrowy = 11;

  int numPerLayer = perrowx * perrowy;
  //int layers = 3;
  int layers = 3;
  int nTotal = numPerLayer * layers;

  Real ynoise = 0.1*drad;

  //add the desired number of particles
  std::cout << "Number of meshes: " << numPerLayer*layers << std::endl;
  VECTOR3 pos(params_->extents_[0] + 7.0 * drad + distbetween, params_->extents_[2] + 7.0 * drad + distbetween + ynoise, params_->extents_[4] + 20.0 * drad);

  for (int z = 0; z<layers; z++)
  {
    for (int j = 0; j<perrowy; j++)
    {
      for (int i = 0; i<perrowx; i++, count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x, pos.y + ynoise, pos.z);
        world_->rigidBodies_[count]->translateTo(bodypos);
        if (z == 1)
        {
          double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, radian, 0);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        else if (z == 0)
        {
          double radian = 1.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, 0, radian);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        world_->rigidBodies_[count]->matTransform_ = world_->rigidBodies_[count]->getQuaternion().GetMatrix();
        world_->rigidBodies_[count]->transform_.setMatrix(world_->rigidBodies_[count]->matTransform_);
        world_->rigidBodies_[count]->transform_.setOrigin(world_->rigidBodies_[count]->com_);
        pos.x += d + distbetween;
      }
      pos.x = params_->extents_[0] + 7.0 * drad + distbetween;
      pos.y += d + distbetween;
    }
    ynoise = -ynoise;
    pos.z += 2.0*d;
    pos.y = params_->extents_[2] + 7.0 * drad + distbetween + ynoise;
  }

}

void ParticleFactory::bloodCells()
{

  for (int j = 0; j<4; j++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new MeshObject<Real>();
    MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);

    pMeshObject->setFileName("meshes/blood_cell.obj");

    body->shape_ = pMeshObject;
    body->shapeId_ = RigidBody::MESH;
    body->density_ = 1.1e-6;

    body->volume_ = 94.0;
    body->invMass_ = 1.0 / (body->density_ * body->volume_);

    Real dmass = body->density_ * body->volume_;
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
    body->generateInvInertiaTensor();

    //load model from file
    GenericLoader Loader;
    Loader.readModelFromFile(&pMeshObject->getModel(), pMeshObject->getFileName().c_str());

    pMeshObject->generateBoundingBox();

    Model3D model_out_0(pMeshObject->getModel());
    model_out_0.meshes_[0].com_ = VECTOR3(0, 0, 0);
    model_out_0.generateBoundingBox();
    model_out_0.meshes_[0].generateBoundingBox();
    std::vector<Triangle3r> pTriangles = model_out_0.genTriangleVector();

    CSubDivRessources myRessources_dm(1, 5, 0, model_out_0.getBox(), &pTriangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
    pMeshObject->initTree(subdivider_dm);

    world_->rigidBodies_.push_back(body);
  }

  Real drad = world_->rigidBodies_[0]->shape_->getAABB().extents_[world_->rigidBodies_[0]->shape_->getAABB().longestAxis()];

  Real d = 2.0 * drad;
  Real dz = 4.0 * drad;
  Real distbetween = 0.2 * drad;
  Real distbetweeny = drad;
  Real distbetweenz = 0.5 * drad;

  int perrowx = 2;
  int perrowy = 1;

  int numPerLayer = perrowx * perrowy;
  int layers = 2;
  int nTotal = numPerLayer * layers;

  Real ynoise = 0.1*drad;

  //add the desired number of particles
  std::cout << "Number of meshes: " << numPerLayer*layers << std::endl;
  VECTOR3 pos(params_->extents_[0] + drad + distbetween, params_->extents_[2] + drad + distbetween + ynoise, params_->extents_[4] + drad);

  int count = 0;

  for (int z = 0; z<layers; z++)
  {
    for (int j = 0; j<perrowy; j++)
    {
      for (int i = 0; i<perrowx; i++, count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x, pos.y + ynoise, pos.z);
        world_->rigidBodies_[count]->translateTo(bodypos);

        if (z%2 != 0)
        {
          double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, radian, 0);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        else
        {
          double radian = 1.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, 0, radian);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }

        pos.x += d + distbetween;
      }
      pos.x = params_->extents_[0] + drad + distbetween;
      pos.y += d + distbetween;
    }
    ynoise = -ynoise;
    pos.z += 1.25*d;
    pos.y = params_->extents_[2] + drad + distbetween + ynoise;
  }

}

void ParticleFactory::initSplitBottom()
{

  Real rad = 0.05;
  Real z_bottom = params_->extents_[4]+rad;
  std::vector<RigidBody*>::iterator rIter;

  int offset=0;
  int count=0;
  for(int z(0);z<2;z++)
  {
    int nu = 18;
    for(int k(0);k<1;k++)
    {


      for(int j=0; j<nu; j++)
      {

        CompoundBody *body = new CompoundBody();
        body->density_ = 8522.0;

        //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
        //from previous timestep is stored in these

        body->angle_=VECTOR3(0, 0.0, 0);
        body->setOrientation(body->angle_);
        body->setAngVel(VECTOR3(0, 0.0 , 0));
        body->setTransformationMatrix(body->quat_.GetMatrix());
        //addSpheres2(body->rigidBodies_, 3, 0.05);

        addSpheres2(body->rigidBodies_, 1 , 0.05);
        body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.0);
        world_->rigidBodies_.push_back(body);
        body->generateInvInertiaTensor();

        VECTOR3 t0(1,0,0);
        VECTOR3 t1(0,1,0);

        VECTOR3 dhk = getPointOnCircle(t0,t1,params_->extents_[5] + (k+1)*2.1*rad ,j,nu);
        body->rigidBodies_[0]->com_ = VECTOR3(0,0,z_bottom + z * 2.0*rad);
        body->rigidBodies_[0]->com_ += dhk;

        count++;

      }


      offset += nu;
      nu+=10-k;
    }

  }

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

}

void ParticleFactory::initBoxStack()
{
  Real extends[3]={params_->defaultRadius_,params_->defaultRadius_,params_->defaultRadius_};
  Real drad = extends[0];
  Real d    = 2.0 * drad;
  Real distbetween = drad * 0.05;
  Real deltaz = d;

  int towerheight=9;

  addBoxes(world_->rigidBodies_,towerheight,extends);

  //assign the physical parameters of the rigid bodies
  initRigidBodyParameters();

  VECTOR3 pos(0.0, 0.0, extends[2]);
  for(int i=0;i<towerheight;i++)
  {
    world_->rigidBodies_[i]->translateTo(pos);
    pos.z+=deltaz;
  }

}

void ParticleFactory::buildHopperTest()
{

  int perRow=8;
  int columns=8;
  for(int i=0;i<columns*perRow;i++)
  {
    CompoundBody *body = new CompoundBody();
    body->density_ = 8522.0;

    //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
    //from previous timestep is stored in these

    body->angle_=VECTOR3(0, 0.0, 0);
    body->setOrientation(body->angle_);
    body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
    body->setTransformationMatrix(body->quat_.GetMatrix());
    //addSpheres2(body->rigidBodies_, 3, 0.05);

    addSpheres2(body->rigidBodies_, 4 , 0.05);
    body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.2);
    body->rigidBodies_[1]->com_=VECTOR3(0.025,0.0,0.2);
    body->rigidBodies_[2]->com_=VECTOR3(0.025,0.0,0.175);
    body->rigidBodies_[3]->com_=VECTOR3(0.00,0.0,0.175);

    world_->rigidBodies_.push_back(body);

    body->generateInvInertiaTensor();
  }

  for (auto &rb : world_->rigidBodies_)
  {

    if(rb->shapeId_!=RigidBody::COMPOUND)continue;

    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {

    if(rb->shapeId_!=RigidBody::COMPOUND)continue;

    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  CompoundBody *body = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  VECTOR3 position = VECTOR3(-1.0+body->getBoundingSphereRadius()+5.0*body->getBoundingSphereRadius(),-1.0+4.0*body->getBoundingSphereRadius(),0.75);

  int index=1;
  for(int col=0;col<columns;col++)
  {
    for(int row=0;row<perRow;row++)
    {
      //world_->rigidBodies_[index]->translateTo(position);
      CompoundBody *c = dynamic_cast<CompoundBody*>(world_->rigidBodies_[index]);
      c->com_ = position;

      for (auto &comp : c->rigidBodies_)
      {
        comp->transform_.setOrigin(c->com_);
        comp->transform_.setMatrix(c->getTransformationMatrix());
      }

      world_->rigidBodies_[index]->color_ = position.x;
      position.x += 2.0*body->getBoundingSphereRadius();
      index++;
    }
    position.y += 2.0*body->getBoundingSphereRadius();
    position.x =-1.0+body->getBoundingSphereRadius()+4.0*body->getBoundingSphereRadius();
  }

}

void ParticleFactory::buildTorqueTest()
{

  for(int i=0; i<2; i++)
  {
    CompoundBody *body = new CompoundBody();
    body->density_ = 8522.0;

    //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
    //from previous timestep is stored in these

    body->angle_=VECTOR3(0, 0.0, 0);
    body->setOrientation(body->angle_);
    body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
    body->setTransformationMatrix(body->quat_.GetMatrix());

    addSpheres2(body->rigidBodies_, 1 , 0.05);
    body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.0);

    world_->rigidBodies_.push_back(body);

    body->generateInvInertiaTensor();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  CompoundBody *a = dynamic_cast<CompoundBody*>(world_->rigidBodies_[0]);
  a->com_ = VECTOR3(0.0,0.0,0);

  CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  b->com_ = VECTOR3(0.099, 0.0, 0.0);

  b->setAngVel(VECTOR3(0, 3.14, 0));

  for (auto &comp : b->rigidBodies_)
  {
    comp->transform_.setOrigin(b->com_);
    comp->transform_.setMatrix(b->getTransformationMatrix());
  }

}

void ParticleFactory::buildBoxGrainTest()
{

  int columns = 1;
  int perRow = 1;

  for(int i=0; i<columns*perRow; i++)
  {
    CompoundBody *body = new CompoundBody();
    body->density_ = 8522.0;

    //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
    //from previous timestep is stored in these

    body->angle_=VECTOR3(0, 0.0, 0);
    body->setOrientation(body->angle_);
    body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
    body->setTransformationMatrix(body->quat_.GetMatrix());

    addSpheres2(body->rigidBodies_, 4 , 0.05);
    body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.2);
    body->rigidBodies_[1]->com_=VECTOR3(0.025,0.0,0.2);
    body->rigidBodies_[2]->com_=VECTOR3(0.025,0.0,0.175);
    body->rigidBodies_[3]->com_=VECTOR3(0.00,0.0,0.175);

    world_->rigidBodies_.push_back(body);

    body->generateInvInertiaTensor();
  }

  for (auto &rb : world_->rigidBodies_)
  {

    if(rb->shapeId_ != RigidBody::COMPOUND)
      continue;

    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {

    if(rb->shapeId_ != RigidBody::COMPOUND)
      continue;

    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }
  }

  CompoundBody *body = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  
  VECTOR3 position = VECTOR3(-0.76, -3.5 * body->getBoundingSphereRadius(), -1.0 + 0.066);

  int index = 1;
  for (int col = 0; col<columns; col++)
  {
    for (int row = 0; row<perRow; row++)
    {
      //world_->rigidBodies_[index]->translateTo(position);
      CompoundBody *c = dynamic_cast<CompoundBody*>(world_->rigidBodies_[index]);
      c->com_ = position;

      for (auto &comp : c->rigidBodies_)
      {
        comp->transform_.setOrigin(c->com_);
        comp->transform_.setMatrix(c->getTransformationMatrix());
      }

      world_->rigidBodies_[index]->color_ = position.x;
      position.x += 2.0*body->getBoundingSphereRadius();
      index++;
    }
    position.y += 2.0*body->getBoundingSphereRadius();
    position.x = -0.76;
  }

  CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  b->com_ = VECTOR3(-0.95,0.0,-1.0+0.066);

  for (auto &comp : b->rigidBodies_)
  {
    comp->transform_.setOrigin(b->com_);
    comp->transform_.setMatrix(b->getTransformationMatrix());
  }

}

void ParticleFactory::buildSphereOfCompounds()
{

  for(int i=0;i!=50;i++)
  {
    CompoundBody *body = new CompoundBody();
    body->density_ = 8522.0;

    //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
    //from previous timestep is stored in these

    body->angle_=VECTOR3(0, 0.0, 0);
    body->setOrientation(body->angle_);
    body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
    body->setTransformationMatrix(body->quat_.GetMatrix());
    //addSpheres2(body->rigidBodies_, 3, 0.05);

    addSpheres2(body->rigidBodies_, 3 , 0.05);
    body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.233);
    body->rigidBodies_[1]->com_=VECTOR3(0.1,0.0,0.233);
    body->rigidBodies_[2]->com_=VECTOR3(0.05,0.0,0.15);


    world_->rigidBodies_.push_back(body);

    body->generateInvInertiaTensor();
  }


  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();


    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

  CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[0]);

  //add the desired number of particles

  int r = 5, ballr = 5;
  // inject a sphere of particles
  float pr = b->getBoundingSphereRadius()/1.0;
  float tr = pr + (pr*2.0f)*ballr;
  float pos[4];
  pos[0] = -1.0f + tr + frand()*(2.0f - tr*2.0f);
  pos[1] = 1.0f - tr;
  pos[2] = 0.15f + tr + frand()*(2.0f - tr*2.0f);
  pos[3] = 0.0f;
  //  vel[0] = vel[1] = vel[2] = vel[3] = 0.0f;

  float spacing = pr*2.0f;
  unsigned int index = 0;
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
        float jitter = pr*0.1f;
        if ((l <= pr*2.0f*r) && (index < world_->rigidBodies_.size()))
        {
          VECTOR3 position(pos[0] + dx + (frand()*2.0f - 1.0f)*jitter,
          pos[1] + dy + (frand()*2.0f - 1.0f)*jitter,
          pos[2] + dz + (frand()*2.0f - 1.0f)*jitter);

          //world_->rigidBodies_[index]->translateTo(position);
          CompoundBody *c = dynamic_cast<CompoundBody*>(world_->rigidBodies_[index]);
          c->com_ = position;

          float ang_z = 2.0 * CMath<Real>::SYS_PI * frand();

          c->angle_=VECTOR3(0,0,ang_z);
          c->setOrientation(c->angle_);

          for (auto &comp : c->rigidBodies_)
          {
            comp->transform_.setOrigin(c->com_);
            comp->transform_.setMatrix(c->getTransformationMatrix());
          }

          world_->rigidBodies_[index]->color_ = position.x;
          index++;
        }
      }
    }
  }

  for(int i=0;i!=20;i++)
  {
    if(i==0 || i==14 || i==15)
    {
      std::cout << "Position: " << world_->rigidBodies_[i]->com_;
      std::cout << "Ang_z: " << world_->rigidBodies_[i]->angle_.z << std::endl;
    }
  }

}


void ParticleFactory::initCompoundBodies()
{

  CompoundBody *body = new CompoundBody();
  body->density_ = 8522.0;
  
  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration 
  //from previous timestep is stored in these
  
  body->angle_=VECTOR3(0,0.0, 0);
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->quat_.GetMatrix());
  //addSpheres2(body->rigidBodies_, 3, 0.05);
  
  addSpheres2(body->rigidBodies_, 3 , 0.05);
  body->rigidBodies_[1]->com_=VECTOR3(0.12,0.01,0.133);
  body->rigidBodies_[0]->com_=VECTOR3(0.02,0.01,0.133);
  body->rigidBodies_[2]->com_=VECTOR3(0.07,0.01,0.05);
  body->velocity_ = VECTOR3(0,0,1.0);
  world_->rigidBodies_.push_back(body);

  body->generateInvInertiaTensor();

  CompoundBody *body1 = new CompoundBody();
  body1->density_ = 8522.0;


  body1->angle_=VECTOR3(0, 0, 0.0);
  body1->setOrientation(body1->angle_);
  body1->setTransformationMatrix(body1->quat_.GetMatrix());


  addSpheres2(body1->rigidBodies_, 3 , 0.05);
  body1->rigidBodies_[1]->com_=VECTOR3(0.12,0.0,0.233 + 0.1);
  body1->rigidBodies_[0]->com_=VECTOR3(0.02,0.0,0.233 + 0.1);
  body1->rigidBodies_[2]->com_=VECTOR3(0.07,0.0,0.15 + 0.1);


  world_->rigidBodies_.push_back(body1);

  body1->generateInvInertiaTensor();

  for (auto &rb : world_->rigidBodies_)
  {
	  CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
	  CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

  CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[0]);
  b->com_ = VECTOR3(-0.17,0,0.325);
  b->velocity_ = VECTOR3(0.0,0,0.0);

  b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  b->com_ = VECTOR3(0,0,0.525);
  b->velocity_ = VECTOR3(0.0,0,-1.0);

  for(int i=0; i<2; i++)
  {
    b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[i]);
    for (auto &comp : b->rigidBodies_)
    {
      comp->transform_.setOrigin(b->com_);
      comp->transform_.setMatrix(b->getTransformationMatrix());
    }
  }

}

void ParticleFactory::initCompoundBodies2()
{

  CompoundBody *body = new CompoundBody();
  body->density_ = 8522.0;

  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
  //from previous timestep is stored in these
  body->setAngVel(VECTOR3(0.0,3.14,3.14));
  body->angle_=VECTOR3(0,0.0, 0);
  body->setOrientation(body->angle_);

  body->setTransformationMatrix(body->quat_.GetMatrix());
  //addSpheres2(body->rigidBodies_, 3, 0.05);

  addSpheres2(body->rigidBodies_, 7 , 0.05);
  body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,0.0);
  body->rigidBodies_[1]->com_=VECTOR3(0.0,0.0,0.05);
  body->rigidBodies_[2]->com_=VECTOR3(0.0,0.0,-0.05);
  body->rigidBodies_[3]->com_=VECTOR3(0.05,0.0,0.0);
  body->rigidBodies_[4]->com_=VECTOR3(-0.05,0.0,0.0);
  body->rigidBodies_[5]->com_=VECTOR3(0.0,0.05,0.0);
  body->rigidBodies_[6]->com_=VECTOR3(0.0,-0.05,0.0);

  world_->rigidBodies_.push_back(body);

  body->generateInvInertiaTensor();

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

  CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[0]);
  b->com_ = VECTOR3(0.5,0,0.0);
  b->velocity_ = VECTOR3(0.0,0,0.0);

  for(int i=0; i<1; i++)
  {
    b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[i]);
    for (auto &comp : b->rigidBodies_)
    {
      comp->transform_.setOrigin(b->com_);
      comp->transform_.setMatrix(b->getTransformationMatrix());
    }
  }

}


void  ParticleFactory::initDemSpherePlaneTest()
{
  CompoundBody *body = new CompoundBody();
  body->density_ = 8522.0;

  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
  //from previous timestep is stored in these

  body->angle_=VECTOR3(0,0.0, 0);
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->quat_.GetMatrix());
  //addSpheres2(body->rigidBodies_, 3, 0.05);

  addSpheres2(body->rigidBodies_, 1 , 0.05);
  body->rigidBodies_[0]->com_=VECTOR3(0.0,0.0,-0.94);
  body->velocity_ = VECTOR3(0,0,-0.2);

  body->setVolume();
  body->setInvMass();
  body->com_ = VECTOR3(0.05, 0, 0.0105);
  
  body->translateTo(VECTOR3(0.05, 0, 0.0605));

  world_->rigidBodies_.push_back(body);

  body->generateInvInertiaTensor();

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

}

void ParticleFactory::initDemSphereTest()
{

  CompoundBody *body = new CompoundBody();
  body->density_ = 8522.0;

  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
  //from previous timestep is stored in these

  body->angle_=VECTOR3(0,0.0, 0);
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->quat_.GetMatrix());
//  body->setAngVel(VECTOR3(0.0,6.28,0.0));
//  body->setAngVel(VECTOR3(0.0,3.14,3.14));
  //addSpheres2(body->rigidBodies_, 3, 0.05);

  addSpheres2(body->rigidBodies_, 1 , 0.05);
  //body->rigidBodies_[0]->com_=VECTOR3(-40.0,0.0,24.7);
  //body->rigidBodies_[0]->com_ = VECTOR3(0, -0.9, 0.26);
  //body->rigidBodies_[0]->com_ = VECTOR3(-0.25, 0.0, 0.25);
  //body->rigidBodies_[0]->com_ = VECTOR3(-0.75, 0.0, -0.95);
  body->rigidBodies_[0]->com_ = VECTOR3(-0.95, 0.0, 0.625);
  body->rigidBodies_[0]->com_ = VECTOR3(0., 0.0, 0.625);
  body->rigidBodies_[0]->com_ = VECTOR3(0.5, 0.0, -0.2);
  //body->rigidBodies_[0]->com_ = VECTOR3(0, 0.8, 0.56);
  //body->rigidBodies_[0]->com_ = VECTOR3(0, 0.95, 0.3);
  //body->rigidBodies_[0]->com_ = VECTOR3(-0.75, 0.0, -0.95);
  //body->velocity_ = VECTOR3(1.0,0.0,0.0);

  world_->rigidBodies_.push_back(body);

  body->generateInvInertiaTensor();

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }



  //int columns = 1;
  //int perRow = 1;

  //for (int i = 0; i<columns*perRow; i++)
  //{
  //  CompoundBody *body = new CompoundBody();
  //  body->density_ = 8522.0;

  //  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
  //  //from previous timestep is stored in these

  //  body->angle_ = VECTOR3(0, 0.0, 0);
  //  body->setOrientation(body->angle_);
  //  body->setAngVel(VECTOR3(0, 0.0 * CMath<Real>::SYS_PI, 0));
  //  body->setTransformationMatrix(body->quat_.GetMatrix());

  //  addSpheres2(body->rigidBodies_, 4, 1.0);
  //  body->rigidBodies_[0]->com_ = VECTOR3(0.0, 0.0, 4.0);
  //  body->rigidBodies_[1]->com_ = VECTOR3(0.5, 0.0, 4.0);
  //  body->rigidBodies_[2]->com_ = VECTOR3(0.5, 0.0, 3.5);
  //  body->rigidBodies_[3]->com_ = VECTOR3(0.00, 0.0, 3.5);

  //  world_->rigidBodies_.push_back(body);

  //  body->generateInvInertiaTensor();
  //}

  //for (auto &rb : world_->rigidBodies_)
  //{

  //  if (rb->shapeId_ != RigidBody::COMPOUND)
  //    continue;

  //  CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
  //  body->setVolume();
  //  body->setInvMass();

  //  for (auto &comp : body->rigidBodies_)
  //  {
  //    body->com_ += comp->com_;
  //  }
  //  body->com_ *= 1.0 / body->rigidBodies_.size();
  //}

  //for (auto &rb : world_->rigidBodies_)
  //{

  //  if (rb->shapeId_ != RigidBody::COMPOUND)
  //    continue;

  //  CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
  //  for (auto &comp : body->rigidBodies_)
  //  {
  //    comp->com_ = comp->com_ - body->com_;
  //    comp->transform_.setOrigin(body->com_);
  //    comp->transform_.setMatrix(body->getTransformationMatrix());
  //  }
  //}

  //CompoundBody *body = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);

  //VECTOR3 position = VECTOR3(-0.76, -3.5 * body->getBoundingSphereRadius(), -1.0 + 0.066);

  //int index = 1;
  //for (int col = 0; col<columns; col++)
  //{
  //  for (int row = 0; row<perRow; row++)
  //  {
  //    //world_->rigidBodies_[index]->translateTo(position);
  //    CompoundBody *c = dynamic_cast<CompoundBody*>(world_->rigidBodies_[index]);
  //    c->com_ = position;

  //    for (auto &comp : c->rigidBodies_)
  //    {
  //      comp->transform_.setOrigin(c->com_);
  //      comp->transform_.setMatrix(c->getTransformationMatrix());
  //    }

  //    world_->rigidBodies_[index]->color_ = position.x;
  //    position.x += 2.0*body->getBoundingSphereRadius();
  //    index++;
  //  }
  //  position.y += 2.0*body->getBoundingSphereRadius();
  //  position.x = -0.76;
  //}

  //CompoundBody *b = dynamic_cast<CompoundBody*>(world_->rigidBodies_[1]);
  ////body->rigidBodies_[0]->com_=VECTOR3(-40.0,0.0,24.7);
  //b->com_ = VECTOR3(-40.0, 0.0, 25.0);

  //for (auto &comp : b->rigidBodies_)
  //{
  //  comp->transform_.setOrigin(b->com_);
  //  comp->transform_.setMatrix(b->getTransformationMatrix());
  //}


}

void ParticleFactory::stictionTest()
{

  CompoundBody *body = new CompoundBody();
  body->density_ = 8522.0;

  //for motionintegratorDEM, set biasAngVel and biasVelocity to zero before Simulation starts since acceleration
  //from previous timestep is stored in these

  body->angle_=VECTOR3(0,0.0, 0);
  body->setOrientation(body->angle_);
  body->setTransformationMatrix(body->quat_.GetMatrix());
  //addSpheres2(body->rigidBodies_, 3, 0.05);

  addSpheres2(body->rigidBodies_, 1 , 0.05);

  body->rigidBodies_[0]->com_ = VECTOR3(-0.75, 0.0, -0.95);
  body->velocity_ = VECTOR3(1.0,0.0,0.0);

  world_->rigidBodies_.push_back(body);

  body->generateInvInertiaTensor();

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    body->setVolume();
    body->setInvMass();

    for (auto &comp : body->rigidBodies_)
    {
      body->com_ += comp->com_;
    }
    body->com_ *= 1.0/body->rigidBodies_.size();
  }

  for (auto &rb : world_->rigidBodies_)
  {
    if (rb->shapeId_ != RigidBody::COMPOUND)
      continue;
    CompoundBody *body = dynamic_cast<CompoundBody*>(rb);
    for (auto &comp : body->rigidBodies_)
    {
      comp->com_ = comp->com_ - body->com_;
      comp->transform_.setOrigin(body->com_);
      comp->transform_.setMatrix(body->getTransformationMatrix());
    }

  }

}

void ParticleFactory::initPyramidTest()
{

  Real extends[3]={params_->defaultRadius_,params_->defaultRadius_,params_->defaultRadius_};
  Real drad = extends[0];
  Real d    = 2.0 * drad;
  Real distbetween = drad * 0.1;
  Real delta = d+distbetween;
  Real deltaz = d;

  int towerheight=9;

  int layers=14;
  int iboxes = (layers*(layers+1))/2.0;
  addBoxes(world_->rigidBodies_,iboxes,extends);

  //assign the physical parameters of the rigid bodies
  initRigidBodyParameters();
  Real length = Real(layers-1) * delta + d;
  Real ystart = -(length/2.0);
  
  VECTOR3 pos(0.5*(params_->extents_[1] + params_->extents_[0]), ystart, params_->extents_[4] + world_->rigidBodies_.back()->getAABB().extents_[2]);
  int index = 0;
  for(int i=0;i<layers;i++)
  {
    pos.y=ystart+Real(i) * (drad+distbetween/2.0);
    for(int j=i;j<layers;j++)
    {
      world_->rigidBodies_[index]->translateTo(pos);
      pos.y+=delta;
      index++;
    }
    pos.z+=deltaz;
  }

//  addBoxes(myWorld.rigidBodies_,towerheight,extends);
//  pos = VECTOR3(params_->extents_[1]/2.0+5.0*d,
//                params_->extents_[3]/2.0, extends[2]);
//
//  for(int j=iboxes;j<myWorld.rigidBodies_.size();j++)
//  {
//    RigidBody *body    = myWorld.rigidBodies_[j];
//    if(!body->affectedByGravity_)
//     continue;
//    body->density_    = myParameters.defaultDensity_;
//    body->volume_     = body->shape_->getVolume();
//    Real dmass          = body->density_ * body->volume_;
//    body->invMass_    = 1.0/(body->density_ * body->volume_);
//    body->angle_      = VECTOR3(0,0,0);
//    body->setAngVel(VECTOR3(0,0,0));
//    body->velocity_   = VECTOR3(0,0,0);
//    body->com_        = VECTOR3(0,0,0);
//    body->force_      = VECTOR3(0,0,0);
//    body->torque_     = VECTOR3(0,0,0);
//    body->restitution_ = 0.0;
//    body->setOrientation(body->angle_);
//    body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//    //calculate the inertia tensor
//    //Get the inertia tensor
//    body->generateInvInertiaTensor();
//    body->translateTo(pos);
//    pos.z+=d;
//  }
//
//  iboxes = myWorld.rigidBodies_.size();
//  addBoxes(myWorld.rigidBodies_,towerheight,extends);
//  pos = VECTOR3(params_->extents_[1]/2.0+10.0*d, params_->extents_[3]/2.0, extends[2]);
//
//  for(int j=iboxes;j<myWorld.rigidBodies_.size();j++)
//  {
//    RigidBody *body    = myWorld.rigidBodies_[j];
//    if(!body->affectedByGravity_)
//     continue;
//    body->density_    = myParameters.defaultDensity_;
//    body->volume_     = body->shape_->getVolume();
//    Real dmass        = body->density_ * body->volume_;
//    body->invMass_    = 1.0/(body->density_ * body->volume_);
//    body->angle_      = VECTOR3(0,0,0);
//    body->setAngVel(VECTOR3(0,0,0));
//    body->velocity_   = VECTOR3(0,0,0);
//    body->com_        = VECTOR3(0,0,0);
//    body->force_      = VECTOR3(0,0,0);
//    body->torque_     = VECTOR3(0,0,0);
//    body->restitution_ = 0.0;
//    body->setOrientation(body->angle_);
//    body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//    //calculate the inertia tensor
//    //Get the inertia tensor
//    body->generateInvInertiaTensor();
//    body->translateTo(pos);
//    pos.z+=d;
//  }
//
//  iboxes = myWorld.rigidBodies_.size();
//  addBoxes(myWorld.rigidBodies_,1,extends);
//  pos = VECTOR3(params_->extents_[1]/2.0-4.0*d, params_->extents_[3]/2.0, 7.25 * extends[2]);
//
//  RigidBody *body    = myWorld.rigidBodies_.back();
//  body->density_    = myParameters.defaultDensity_;
//  body->volume_     = body->shape_->getVolume();
//  Real dmass          = body->density_ * body->volume_;
//  body->invMass_    = 1.0/(body->density_ * body->volume_);
//  body->angle_      = VECTOR3(0,0,0);
//  body->setAngVel(VECTOR3(0,0,0));
//  body->velocity_   = VECTOR3(5.0,0,0);
//  body->com_        = VECTOR3(0,0,0);
//  body->force_      = VECTOR3(0,0,0);
//  body->torque_     = VECTOR3(0,0,0);
//  body->restitution_ = 0.0;
//  body->setOrientation(body->angle_);
//  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
//  //calculate the inertia tensor
//  //Get the inertia tensor
//  body->generateInvInertiaTensor();
//  body->translateTo(pos);
//  pos.z+=d;

}

void ParticleFactory::initPyramidSticks()
{

  Real extends[3] = { params_->defaultRadius_, params_->defaultRadius_*5, params_->defaultRadius_ };
  Real drad = extends[0];
  Real d = 2.0 * drad;
  Real dy = 2.0 * extends[1];
  Real drady = extends[1];
  Real distbetween = drad * 0.1;
  Real delta = d + distbetween;
  Real deltay = dy + distbetween;
  Real deltaz = d;

  int towerheight = 9;

  int layers = 18;
  int iboxes = (layers*(layers + 1)) / 2.0;
  addBoxes(world_->rigidBodies_, iboxes, extends);

  //assign the physical parameters of the rigid bodies
  initRigidBodyParameters();
  Real length = Real(layers - 1) * delta + dy;
  Real ystart = -(length / 2.0) - 1.0;

  VECTOR3 pos(0.5*(params_->extents_[1] + params_->extents_[0]), ystart, params_->extents_[4] + world_->rigidBodies_.back()->getAABB().extents_[2]);
  int index = 0;
  for (int i = 0; i < layers; i++)
  {
    pos.y = ystart + Real(i) * (drady + distbetween / 2.0);
    for (int j = i; j < layers; j++)
    {
      world_->rigidBodies_[index]->translateTo(pos);
      pos.y += deltay;
      index++;
    }
    pos.z += deltaz;
  }

}

void ParticleFactory::initTowers()
{

  Real extends[3] = { params_->defaultRadius_, params_->defaultRadius_ * 5, params_->defaultRadius_ };
  Real drad = extends[0];
  Real d = 2.0 * drad;
  Real dy = 2.0 * extends[1];
  Real drady = extends[1];
  Real distbetween = drad * 0.1;
  Real delta = d + distbetween;
  Real deltay = dy + distbetween;
  Real deltaz = d;

  int towerheight = 9;

  int layers = 18;
  //int iboxes = (layers*(layers + 1)) / 2.0;
  addBoxes(world_->rigidBodies_, layers*layers, extends);

  //assign the physical parameters of the rigid bodies
  initRigidBodyParameters();
  Real length = Real(layers - 1) * delta + dy;
  Real ystart = -(length / 2.0) - 1.0;

  VECTOR3 pos(0.5*(params_->extents_[1] + params_->extents_[0]), ystart, params_->extents_[4] + world_->rigidBodies_.back()->getAABB().extents_[2]);
  int index = 0;


  for (int y = 0; y < layers; ++y)
  {
    for (int i = 0; i < layers; ++i)
    {
      world_->rigidBodies_[index]->translateTo(pos);
      pos.z += deltaz;
      index++;
    }
    pos.z = params_->extents_[4] + world_->rigidBodies_.back()->getAABB().extents_[2];
    pos.y += deltay;
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

void ParticleFactory::addSpheres2(std::vector<RigidBody*> &rigidBodies, int nSpheres, Real rad)
{
  for(int i=0;i<nSpheres;i++)
  {
    RigidBody *body = new RigidBody();
    body->shape_ = new Spherer(VECTOR3(0,0,0),rad);
    body->shapeId_ = RigidBody::SPHERE;
    body->density_ = params_->defaultDensity_;
    body->volume_ = body->shape_->getVolume();
    Real dmass = body->density_ * body->volume_;
    body->invMass_ = 1.0 / (body->density_ * body->volume_);
    body->com_= VECTOR3(0, 0, 0);
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

    rigidBodies.push_back(body);
  }
}

void ParticleFactory::addMeshObjects(std::vector< RigidBody* >& rigidBodies, int nObjects, const char* fileName)
{

  for(int i=0;i<nObjects;i++)
  {        
    RigidBody *body = new RigidBody();    
    body->shape_ = new MeshObject<Real>();
    MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);
    pMeshObject->setFileName(fileName);
    body->volume_   = body->shape_->getVolume();
    body->invMass_  = 0.0;

    GenericLoader Loader;
    Loader.readModelFromFile(&pMeshObject->getModel(), pMeshObject->getFileName().c_str());

    pMeshObject->generateBoundingBox();

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

void ParticleFactory::addStandardMeshes(std::vector<RigidBody*> &rigidBodies, int iCount)
{

  VECTOR3 center(0, 0, 0);
  VECTOR3 vUVW[3] = { VECTOR3(1, 0, 0), VECTOR3(0, 1, 0), VECTOR3(0, 0, 1) };
  std::vector<RigidBody*>::iterator rIter;
  int l = 0;
  for (int i = 0; i<iCount; i++)
  {
    BodyStorage bs;

    bs.velocity_ = VECTOR3(0,0,0);
    bs.density_ = 1.5;
    bs.restitution_ = 0.0;
    bs.angle_ = VECTOR3(0,0,0);
    bs.angVel_ = VECTOR3(0, 0, 0);
    bs.shapeId_ = RigidBody::MESH;
    bs.id_ = 0;
    bs.com_ = VECTOR3(0, 0, 0);
    bs.force_ = VECTOR3(0, 0, 0);
    bs.torque_ = VECTOR3(0, 0, 0);
    bs.quat_ = Quaternionr(0,0,0,1);
    bs.affectedByGravity_ = 1;

    memset(bs.tensor_, 0, 9 * sizeof(Real));

    bs.uvw_[0] = vUVW[0];
    bs.uvw_[1] = vUVW[1];
    bs.uvw_[2] = vUVW[2];

    std::string name;
    if (l==0)
      name = std::string("meshes/dog_small.obj");
    else if (l==1)
      name = std::string("meshes/cylinder.obj");
    else if (l == 2)
      name = std::string("meshes/cone.obj");
    else if (l == 3)
      name = std::string("meshes/ellipsoid.obj");


    l++;
    if (l > 3)
      l = 0;

    for (int j = 0; j < 256; ++j)
    {
      bs.fileName_[j]='\0';
    }

    name.copy(bs.fileName_, name.length() + 1);

    RigidBody *body = new RigidBody(&bs);
    rigidBodies.push_back(body);
  }

}

void ParticleFactory::softBodyParticles()
{

  // add 100 particles
  addSpheres(world_->rigidBodies_, 50, params_->defaultRadius_);

  initRigidBodyParameters();

}

void ParticleFactory::complexParticles()
{

  int offset = world_->rigidBodies_.size();
  //for (int j = 0; j < 4; j++)
  //{
  //  RigidBody *body = new RigidBody();
  //  body->shape_ = new CMeshObject<Real>();
  //  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);

  //  if (l == 1)
  //    pMeshObject->SetFileName("meshes/torus.obj");
  //  else if (l == 2)
  //    pMeshObject->SetFileName("meshes/ellipsoid.obj");
  //  else if (l == 3)
  //    pMeshObject->SetFileName("meshes/capsule.obj");
  //  else if (l == 0)
  //    pMeshObject->SetFileName("meshes/cone.obj");

  //  l++;
  //  if (l > 3) l = 0;

  //  body->shape_ = pMeshObject;
  //  body->shapeId_ = RigidBody::MESH;
  //  body->density_ = 2.5;

  //  if (pMeshObject->GetFileName() == "meshes/dog_small.obj")
  //  {
  //    body->volume_ = 1.5e-7; // 94.0; //94 micro meter^3
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else if (pMeshObject->GetFileName() == std::string("meshes/capsule.obj"))
  //  {
  //    body->volume_ = 1.3e-6; // 94.0; //94 micro meter^3
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else if (pMeshObject->GetFileName() == std::string("meshes/cone.obj"))
  //  {
  //    body->volume_ = 2.3e-7; // 94.0; //94 micro meter^3
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else if (pMeshObject->GetFileName() == std::string("meshes/cylinder.obj"))
  //  {
  //    body->volume_ = 2.3e-7; // 94.0; //94 micro meter^3
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else if (pMeshObject->GetFileName() == std::string("meshes/torus.obj"))
  //  {
  //    body->volume_ = 3.55e-7;
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else if (pMeshObject->GetFileName() == std::string("meshes/ellipsoid.obj"))
  //  {
  //    body->volume_ = 4.0e-6;
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }
  //  else
  //  {
  //    body->volume_ = 0.01303;
  //    body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  }

  //  Real dmass = body->density_ * body->volume_;
  //  body->invMass_ = 1.0 / (body->density_ * body->volume_);
  //  body->angle_ = VECTOR3(0, 0, 0);
  //  body->setAngVel(VECTOR3(0, 0, 0));
  //  body->velocity_ = VECTOR3(0, 0, 0);
  //  body->com_ = VECTOR3(0, 0, 0);
  //  body->force_ = VECTOR3(0, 0, 0);
  //  body->torque_ = VECTOR3(0, 0, 0);
  //  body->restitution_ = 0.0;
  //  body->setOrientation(body->angle_);
  //  body->setTransformationMatrix(body->getQuaternion().GetMatrix());
  //  //calculate the inertia tensor
  //  body->generateInvInertiaTensor();

  //  //load model from file
  //  GenericLoader Loader;
  //  Loader.readModelFromFile(&pMeshObject->m_Model, pMeshObject->GetFileName().c_str());

  //  pMeshObject->m_Model.GenerateBoundingBox();
  //  for (unsigned i = 0; i< pMeshObject->m_Model.meshes_.size(); i++)
  //  {
  //    pMeshObject->m_Model.meshes_[i].generateBoundingBox();
  //  }

  //  Model3D model_out_0(pMeshObject->m_Model);
  //  model_out_0.meshes_[0].com_ = VECTOR3(0, 0, 0);
  //  model_out_0.GenerateBoundingBox();
  //  model_out_0.meshes_[0].generateBoundingBox();
  //  std::vector<Triangle3r> pTriangles = model_out_0.GenTriangleVector();

  //  CSubDivRessources myRessources_dm(1, 5, 0, model_out_0.GetBox(), &pTriangles);
  //  CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
  //  pMeshObject->m_BVH.InitTree(&subdivider_dm);
  //  world_->rigidBodies_.push_back(body);

  //}

  addStandardMeshes(world_->rigidBodies_, 81);

  int count = offset;

  Real drad = world_->rigidBodies_[count]->shape_->getAABB().extents_[world_->rigidBodies_[count]->shape_->getAABB().longestAxis()];

  Real d = 2.0 * drad;
  Real dz = 4.0 * drad;
  Real distbetween = 0.2 * drad;
  Real distbetweeny = drad;
  Real distbetweenz = 0.5 * drad;

  //  int perrowx = 11;
  //  int perrowy = 11;

  int perrowx = 9;
  int perrowy = 9;

  int numPerLayer = perrowx * perrowy;
  //int layers = 3;
  int layers = 1;
  int nTotal = numPerLayer * layers;

  Real ynoise = 0.1*drad;

  //add the desired number of particles
  std::cout << "Number of meshes: " << numPerLayer*layers << std::endl;
  VECTOR3 pos(params_->extents_[0] + 7.0 * drad + distbetween, params_->extents_[2] + 7.0 * drad + distbetween + ynoise, params_->extents_[4] + 20.0 * drad);

  for (int z = 0; z<layers; z++)
  {
    for (int j = 0; j<perrowy; j++)
    {
      for (int i = 0; i<perrowx; i++, count++)
      {
        //one row in x
        VECTOR3 bodypos = VECTOR3(pos.x, pos.y + ynoise, pos.z);
        world_->rigidBodies_[count]->translateTo(bodypos);
        if (z == 1)
        {
          double radian = 2.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, radian, 0);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        else if (z == 0)
        {
          double radian = 1.0 * CMath<double>::SYS_PI * ((double)rand() / (double)RAND_MAX);
          world_->rigidBodies_[count]->angle_ = VECTOR3(0, 0, radian);
          world_->rigidBodies_[count]->setOrientation(world_->rigidBodies_[count]->angle_);
          world_->rigidBodies_[count]->setTransformationMatrix(world_->rigidBodies_[count]->getQuaternion().GetMatrix());
        }
        world_->rigidBodies_[count]->matTransform_ = world_->rigidBodies_[count]->getQuaternion().GetMatrix();
        world_->rigidBodies_[count]->transform_.setMatrix(world_->rigidBodies_[count]->matTransform_);
        world_->rigidBodies_[count]->transform_.setOrigin(world_->rigidBodies_[count]->com_);
        //pos.x += d + distbetween;
        pos.x += 0.015 + distbetween;
      }
      pos.x = params_->extents_[0] + 7.0 * drad + distbetween;
      //pos.y += d + distbetween;
      pos.y += 0.015 + distbetween;
    }
    ynoise = -ynoise;
    pos.z += 2.0*d;
    pos.y = params_->extents_[2] + 7.0 * drad + distbetween + ynoise;
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

World ParticleFactory::produceMesh(const char* fileName)
{

	GenericLoader Loader;
	World myDomain;
	RigidBody *body = new RigidBody();
	MeshObject<Real> *pMesh= new MeshObject<Real>();

	Loader.readModelFromFile(&pMesh->getModel(),fileName);
	pMesh->generateBoundingBox();

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

  if (param.solverType_ == 5)
  {
    return produceFromJSONParameters(param);
  }
  else
  {
    for(int i=0;i<param.bodies_;i++)
    {
      BodyStorage *sBody = &param.rigidBodies_[i];
      RigidBody *pBody = new RigidBody(sBody);
      myWorld.rigidBodies_.push_back(pBody);
    }
    if (geom_kernel == cgalKernel)
    {
#ifdef WITH_CGAL
      for (int i = 0; i<param.boundaryComponents_; i++)
      {
        if (param.boundaries_[i].type == BoundaryDescription<cgalKernel>::TRISURF)
        {
          myWorld.bndry_.addBoundaryShape(new BoundaryShapeTriSurf<cgalKernel>(param.boundaries_[i].name));
        }
        else if (param.boundaries_[i].type == BoundaryDescription<cgalKernel>::PLINE)
        {
          myWorld.bndry_.addBoundaryShape(new BoundaryShapePolyLine<cgalKernel>(param.boundaries_[i].name));
        }

      }
#endif
    }
    return myWorld;
  }

}

World ParticleFactory::produceFromJSONParameters(WorldParameters & param)
{

  World myWorld;

#ifdef WITH_ODE

  // create world
  dInitODE2(0);
  myWorld.world = dWorldCreate();


  myWorld.space = dHashSpaceCreate (0);

  myWorld.contactgroup = dJointGroupCreate (0);
  
  dWorldSetGravity (myWorld.world,param.gravity_.x,param.gravity_.y,param.gravity_.z);
  dWorldSetQuickStepNumIterations (myWorld.world, 32);

  int isDynamic = 1;

  dMass m;

  using json = nlohmann::json;

  std::ifstream i(param.odeConfigurationFile_);
  json j;
  i >> j;

  for (int i(0); i < j.size(); ++i)
  {

    Vec3 p(j[i]["Pos"][0], j[i]["Pos"][1], j[i]["Pos"][2]);
    Vec3 d(j[i]["Dim"][0], j[i]["Dim"][1], j[i]["Dim"][2]);
    Vec3 q(j[i]["Rot"][0], j[i]["Rot"][1], j[i]["Rot"][2]);
    Vec3 norm(j[i]["Norm"][0], j[i]["Norm"][1], j[i]["Norm"][2]);

    std::string sIsDyn = j[i]["IsDynamic"];

    isDynamic = std::atoi(sIsDyn.c_str());

    BodyODE b;

    dReal rho = dReal(param.defaultDensity_);
    dReal rho_f = dReal(param.densityMedium_);

    if (j[i]["Type"] == "Sphere")
    {

      // Create an ODE rigid body
      b._bodyId = dBodyCreate (myWorld.world);
      
//      // Get an ODE rotation matrix
//      dMatrix3 rMat;
//
//      // Create a rotation matrix from euler angles
//      dRFromEulerAngles(rMat, q.x, q.y, q.z); 
//
//      // Set the rotation of the ODE body 
//      dBodySetRotation(b._bodyId, rMat); 

      // Get the sphere mass
      dMassSetSphere (&m, rho, 0.5 * d.y);

      // Set the mass in the rb structure 
      dBodySetMass (b._bodyId,&m);

      dReal mrel;

      // Get the relative mass
      dMassSetRel(&m, &mrel, rho, rho_f);

      // Set the rmass in the dxBody structure
      dBodySetRelMass(b._bodyId, &mrel);

      // Create a dxBox
      b._geomId = dCreateSphere(0, 0.5 * d.y);

      // Set the geometry of the dxBody
      dGeomSetBody (b._geomId,b._bodyId);

      // Set the position of the dxBody
      dBodySetPosition (b._bodyId, p.x, p.y, p.z);

      // Add the geometry to the world space
      dSpaceAdd (myWorld.space, b._geomId);

      myWorld.bodies_.push_back(b);

      // Create a wrapper rigid body instance
      Real bodyMass(m.mass); 

      BodyStorage body(p, q, 0.5 * d, RigidBody::SPHERE,
                       rho, bodyMass);

      if (isDynamic == 0)
      {
        dBodySetKinematic(b._bodyId);
      }

//      body.toString();
//      std::cout << "ODEmass: " << m.mass << std::endl;
//      std::cout << "mass: " << 1./body.invMass_ << std::endl;

      b._type   = std::string("Sphere");
      b._index  = myWorld.rigidBodies_.size();

      RigidBody *pBody = new RigidBody(&body);

      pBody->odeIndex_ = myWorld.bodies_.size();

      myWorld.rigidBodies_.push_back(pBody);

    }
    else if (j[i]["Type"] == "Plane")
    {

      Real d = norm * p;

      dGeomID plane = dCreatePlane (myWorld.space,norm.x, norm.y, norm.z, d);

      b._geomId = plane;
      b._bodyId = dBodyID(-10);

      myWorld.boundaryGeometries_.push_back(b);

    }
    else if (j[i]["Type"] == "Cube")
    {

      // Create an ODE rigid body
      b._bodyId = dBodyCreate (myWorld.world);

      // Get an ODE rotation matrix
      dMatrix3 rMat;

      Mat3 mat;
      mat.MatrixFromAngles(q);

      // Create a rotation matrix from euler angles
      rMat[0] = mat.m_dEntries[0];
      rMat[1] = mat.m_dEntries[1];
      rMat[2] = mat.m_dEntries[2];
      rMat[3] = 0.0;

      rMat[4] = mat.m_dEntries[3];
      rMat[5] = mat.m_dEntries[4];
      rMat[6] = mat.m_dEntries[5];
      rMat[7] = 0.0;

      rMat[8]  = mat.m_dEntries[6];
      rMat[9] = mat.m_dEntries[7];
      rMat[10] = mat.m_dEntries[8];
      rMat[11] = 0.0;

      // Set the rotation of the ODE body 
      dBodySetRotation(b._bodyId, rMat); 


      // Create a rotation matrix from euler angles
      dRFromEulerAngles(rMat, q.x, q.y, q.z); 

      // Set the rotation of the ODE body 
      dBodySetRotation(b._bodyId, rMat); 

      // Get the box mass
      dMassSetBox(&m, rho, d.x, d.y, d.z);

      // Set the mass in the rb structure 
      dBodySetMass (b._bodyId,&m);

      dReal mrel;

      // Get the relative mass
      dMassSetRel(&m, &mrel, rho, rho_f);

      // Set the rmass in the dxBody structure
      dBodySetRelMass(b._bodyId, &mrel);

      // Create a dxBox
      b._geomId = dCreateBox(0, d.x, d.y, d.z);

      // Set the geometry of the dxBody
      dGeomSetBody (b._geomId,b._bodyId);

      // Set the position of the dxBody
      dBodySetPosition (b._bodyId, p.x , p.y, p.z);

      // Add the geometry to the world space
      dSpaceAdd (myWorld.space, b._geomId);

      myWorld.bodies_.push_back(b);

      // Create a wrapper rigid body instance
      Real bodyMass(m.mass); 

      const dReal *SPos = dBodyGetPosition(b._bodyId);
      const dReal *SRot = dBodyGetRotation(b._bodyId);
      float spos[3] = {SPos[0], SPos[1], SPos[2]};
      float srot[12] = { SRot[0], SRot[1], SRot[2], 
                         SRot[3], SRot[4], SRot[5], 
                         SRot[6], SRot[7], SRot[8], 
                         SRot[9], SRot[10], SRot[11] };


      double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ 
                            SRot[4], SRot[5], SRot[6], /* */ 
                            SRot[8], SRot[9], SRot[10] };

      BodyStorage body(p, q, 0.5 * d, RigidBody::BOX,
                       rho, bodyMass);

      if (isDynamic == 0)
      {
        dBodySetKinematic(b._bodyId);
      }

//      body.toString();
//      std::cout << "ODEmass: " << m.mass << std::endl;
//      std::cout << "mass: " << 1./body.invMass_ << std::endl;

      b._type   = std::string("Box");
      b._index  = myWorld.rigidBodies_.size();

      MATRIX3X3 transform(entries);

      RigidBody *pBody = new RigidBody(&body);

      pBody->setTransformationMatrix(transform);

      pBody->odeIndex_ = myWorld.bodies_.size();

      myWorld.rigidBodies_.push_back(pBody);

    }
    else if (j[i]["Type"] == "Cylinder")
    {

      // Create an ODE rigid body
      b._bodyId = dBodyCreate (myWorld.world);

      Mat3 mat;
      mat.MatrixFromAngles(q);

      // Get an ODE rotation matrix
      dMatrix3 rMat;

      // Create a rotation matrix from euler angles
      rMat[0] = mat.m_dEntries[0];
      rMat[1] = mat.m_dEntries[1];
      rMat[2] = mat.m_dEntries[2];
      rMat[3] = 0.0;

      rMat[4] = mat.m_dEntries[3];
      rMat[5] = mat.m_dEntries[4];
      rMat[6] = mat.m_dEntries[5];
      rMat[7] = 0.0;

      rMat[8]  = mat.m_dEntries[6];
      rMat[9] = mat.m_dEntries[7];
      rMat[10] = mat.m_dEntries[8];
      rMat[11] = 0.0;

      // Set the rotation of the ODE body 
      dBodySetRotation(b._bodyId, rMat); 

//      std::cout << "Quat ODE: " << quat;
//      std::cout << "Quat My: " << myQuat;
//      std::cout << "MyMat: " << myQuat.GetMatrix();
//      std::cout << "--------------------------" << std::endl;
//      for(int j(0); j < 3; ++j)
//      {
//        for(int i(0); i < 3; ++i)
//        {
//          std::cout << rMat[4*i+j] << " ";
//        }
//        std::cout << std::endl;
//      }
//      std::exit(EXIT_FAILURE);

      Real rad = 0.5 * d.x;
      Real &l  = d.z;

      // Get the cylinder mass
      dMassSetCylinder(&m, rho, 3, rad, l);

      // set the mass for the cylinder body
      dBodySetMass (b._bodyId,&m);

      dReal mrel;

      // Get the relative mass
      dMassSetRel(&m, &mrel, rho, rho_f);

      // Set the rmass in the dxBody structure
      dBodySetRelMass(b._bodyId, &mrel);

      // Create a dxBox
      b._geomId = dCreateCylinder(0, rad, l);

      // assign the geometry to the rigid body
      dGeomSetBody (b._geomId,b._bodyId);

      // Set the position of the dxBody
      dBodySetPosition (b._bodyId, p.x , p.y, p.z);

      // Add the geometry to the world space
      dSpaceAdd (myWorld.space, b._geomId);

      myWorld.bodies_.push_back(b);

      // Create a wrapper rigid body instance
      Real bodyMass(m.mass); 

      const dReal *SPos = dBodyGetPosition(b._bodyId);
      const dReal *SRot = dBodyGetRotation(b._bodyId);
      float spos[3] = {SPos[0], SPos[1], SPos[2]};
      float srot[12] = { SRot[0], SRot[1], SRot[2], 
                         SRot[3], SRot[4], SRot[5], 
                         SRot[6], SRot[7], SRot[8], 
                         SRot[9], SRot[10], SRot[11] };


      double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ 
                            SRot[4], SRot[5], SRot[6], /* */ 
                            SRot[8], SRot[9], SRot[10] };

      BodyStorage body(p, q, 0.5 * d, RigidBody::CYLINDER,
                       rho, bodyMass);

      if (isDynamic == 0)
      {
        dBodySetKinematic(b._bodyId);
      }

//      body.toString();
//      std::cout << "ODEmass: " << m.mass << std::endl;
//      std::cout << "mass: " << 1./body.invMass_ << std::endl;

      b._type   = std::string("Cylinder");
      b._index  = myWorld.rigidBodies_.size();

      MATRIX3X3 transform(entries);

      RigidBody *pBody = new RigidBody(&body);

      pBody->setTransformationMatrix(transform);

      pBody->odeIndex_ = myWorld.bodies_.size();

      myWorld.rigidBodies_.push_back(pBody);

    }

  }

#endif

  return myWorld;
}

void ParticleFactory::addFromDataFile(WorldParameters &param, World *world)
{

  for(int i=0;i<param.bodies_;i++)
  {
    BodyStorage *bluePrint = &param.rigidBodies_[i];
    //if the body to be constructed is a compound, initialize a compound body, else a rigid body

    RigidBody *body = new RigidBody(bluePrint);
    world->rigidBodies_.push_back(body);

  }
  
}

World ParticleFactory::produceFromDeformParameters(DeformParameters& param)
{

  World myWorld;

  for(unsigned i=0;i<param.rigidBodies_.size();i++)
  {
    BodyStorage *sBody = &param.rigidBodies_[i];
    RigidBody *pBody = new RigidBody(sBody);
    myWorld.rigidBodies_.push_back(pBody);
  }

  std::cout<<myWorld<<std::endl;

  return myWorld;

}


}
