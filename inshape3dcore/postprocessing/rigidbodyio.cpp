/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "rigidbodyio.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <rigidbody.h>
#include <world.h>
#include <meshobject.h>

namespace i3d {

RigidBodyIO::RigidBodyIO()
{

}

RigidBodyIO::~RigidBodyIO()
{

}

void RigidBodyIO::write(World &world, const char *strFileName, bool outputBoundary)
{
	
	FILE *outFile;
	
	outFile = fopen(strFileName,"wb");
	
	RigidBodyHeader header;
  if(outputBoundary)
	  header.nParticles_ = world.rigidBodies_.size();
  else
	  header.nParticles_ = world.rigidBodies_.size()-1;
	header.nOutput_      = world.output_;
	header.timeStep_     = world.timeControl_->GetTimeStep();
	header.simTime_      = world.timeControl_->GetTime();
	header.deltaT_       = world.timeControl_->GetDeltaT();
	fwrite(&header,sizeof(RigidBodyHeader),1,outFile);

	std::vector<RigidBody*>::iterator rIter;
	rIter = world.rigidBodies_.begin();
	for(;rIter!=world.rigidBodies_.end();rIter++)
	{
		RigidBody &body = *(*rIter);
    if((body.shapeId_ == RigidBody::BOUNDARYBOX && !outputBoundary)  || (body.shapeId_ == RigidBody::SUBDOMAIN))
    {
      continue;
    }
    else if (body.shapeId_ == RigidBody::COMPOUND)
    {
      BodyStorage outBody;
      CompoundBody *c = dynamic_cast<CompoundBody*>(&body);
      outBody.density_ = c->density_;
      outBody.invMass_ = c->invMass_;
      outBody.volume_ = c->volume_;
      outBody.restitution_ = c->restitution_;
      outBody.com_ = c->com_;
      outBody.velocity_ = c->velocity_;
      outBody.shapeId_ = c->shapeId_;
      outBody.id_ = c->iID_;
      outBody.angle_ = c->angle_;
      outBody.angVel_ = c->getAngVel();
      outBody.force_ = c->force_;
      outBody.torque_ = c->torque_;
      outBody.quat_ = c->getQuaternion();
      outBody.extents_[0] = c->getBoundingSphereRadius();
      outBody.extents_[1] = c->getBoundingSphereRadius();
      outBody.extents_[2] = c->getBoundingSphereRadius();
      if (c->affectedByGravity_)
        outBody.affectedByGravity_ = 1;
      else
        outBody.affectedByGravity_ = 0;

      outBody.affectedByGravity_ = 0;
      memcpy(outBody.tensor_, c->invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

      outBody.matrixAvailable_ = true;

      int spheres = c->getNumComponents();
      outBody.spheres = spheres;

      fwrite(&outBody, sizeof(BodyStorage), 1, outFile);

      for (int i = 0; i < spheres; i++)
      {
        RigidBody &_subBody = (*c->rigidBodies_[i]);
        BodyStorage subBody;        
        subBody.density_ = _subBody.density_;
        subBody.invMass_ = _subBody.invMass_;
        subBody.volume_ = _subBody.volume_;
        subBody.restitution_ = _subBody.restitution_;
        subBody.com_ = _subBody.com_;
        subBody.velocity_ = _subBody.velocity_;
        subBody.shapeId_ = _subBody.shapeId_;
        subBody.id_ = _subBody.iID_;
        subBody.angle_ = _subBody.angle_;
        subBody.angVel_ = _subBody.getAngVel();
        subBody.force_ = _subBody.force_;
        subBody.torque_ = _subBody.torque_;
        subBody.quat_ = _subBody.getQuaternion();
        AABB3r box = _subBody.shape_->getAABB();
        subBody.extents_[0] = box.extents_[0];
        subBody.extents_[1] = box.extents_[1];
        subBody.extents_[2] = box.extents_[2];
        if (_subBody.affectedByGravity_)
          subBody.affectedByGravity_ = 1;
        else
          subBody.affectedByGravity_ = 0;

        subBody.affectedByGravity_ = 0;
        memcpy(subBody.tensor_, body.invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

        subBody.matrixAvailable_ = true;

        fwrite(&subBody, sizeof(BodyStorage), 1, outFile);
      }

    }
    else
    {
		  BodyStorage outBody;
		  outBody.density_  = body.density_;
		  outBody.invMass_  = body.invMass_;
		  outBody.volume_   = body.volume_;
		  outBody.restitution_ = body.restitution_;
		  outBody.com_      = body.com_;
		  outBody.velocity_ = body.velocity_;
		  outBody.shapeId_  = body.shapeId_;
      outBody.id_       = body.iID_;
		  outBody.angle_    = body.angle_;
		  outBody.angVel_   = body.getAngVel();
		  outBody.force_    = body.force_;
		  outBody.torque_   = body.torque_;
      outBody.quat_     = body.getQuaternion();
		  AABB3r box         = body.shape_->getAABB();
		  outBody.extents_[0]= box.extents_[0];
		  outBody.extents_[1]= box.extents_[1];
		  outBody.extents_[2]= box.extents_[2];
      if(body.affectedByGravity_)
        outBody.affectedByGravity_=1;
      else
        outBody.affectedByGravity_=0;
    
      memcpy(outBody.tensor_,body.invInertiaTensor_.m_dEntries,9*sizeof(Real));
    
      outBody.matrixAvailable_ = true;
    
		  if(body.shapeId_ == RigidBody::BOX)
		  {
			  OBB3r *pBox = dynamic_cast<OBB3r*>(body.shape_);
			  outBody.uvw_[0]   = pBox->uvw_[0];
			  outBody.uvw_[1]   = pBox->uvw_[1];
			  outBody.uvw_[2]   = pBox->uvw_[2];
		  }

      if(body.shapeId_ == RigidBody::MESH)
		  {
        CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.shape_);
        std::string name = pMesh->GetFileName();
        outBody.fileName_[name.size()]=0;
        memcpy(outBody.fileName_,name.c_str(),name.size());
		  }

		  fwrite(&outBody,sizeof(BodyStorage),1,outFile);
	  }
  }
	
	fclose(outFile);
}

void RigidBodyIO::write(World &world, std::vector<int> &vIndices, const char *strFileName)
{
	
	FILE *outFile;
	int index,bodyid;

	outFile = fopen(strFileName,"wb");

	RigidBodyHeader header;
  header.nParticles_ = vIndices.size();
	header.nOutput_       = world.output_;
	header.timeStep_     = world.timeControl_->GetTimeStep();
	header.simTime_      = world.timeControl_->GetTime();
	header.deltaT_       = world.timeControl_->GetDeltaT();
	fwrite(&header,sizeof(RigidBodyHeader),1,outFile);

	std::vector<RigidBody*>::iterator rIter;
	rIter = world.rigidBodies_.begin();
  index=0;
  bodyid=0;
	for(;rIter!=world.rigidBodies_.end();rIter++,bodyid++)
	{

    if(index >= vIndices.size())
      break;

    if(vIndices[index]!=bodyid)
    {
      continue;
    }
    else
    {
		  RigidBody &body = *(*rIter);                                 
		  BodyStorage outBody;
		  outBody.density_  = body.density_;
		  outBody.invMass_  = body.invMass_;
		  outBody.volume_   = body.volume_;
		  outBody.restitution_ = body.restitution_;
		  outBody.com_      = body.com_;
		  outBody.velocity_ = body.velocity_;
		  outBody.shapeId_  = body.shapeId_;
      outBody.id_       = body.iID_;
		  outBody.angle_    = body.angle_;
		  outBody.angVel_   = body.getAngVel();
		  outBody.force_    = body.force_;
		  outBody.torque_   = body.torque_;
      outBody.quat_     = body.getQuaternion();      
		  AABB3r box        = body.shape_->getAABB();
		  outBody.extents_[0]= box.extents_[0];
		  outBody.extents_[1]= box.extents_[1];
		  outBody.extents_[2]= box.extents_[2];
      if(body.affectedByGravity_)
        outBody.affectedByGravity_=1;
      else
        outBody.affectedByGravity_=0;

      memcpy(outBody.tensor_,body.invInertiaTensor_.m_dEntries,9*sizeof(Real));

      outBody.matrixAvailable_ = true;
      
		  if(body.shapeId_ == RigidBody::BOX)
		  {
			  OBB3r *pBox = dynamic_cast<OBB3r*>(body.shape_);
			  outBody.uvw_[0]   = pBox->uvw_[0];
			  outBody.uvw_[1]   = pBox->uvw_[1];
			  outBody.uvw_[2]   = pBox->uvw_[2];
		  }

      if(body.shapeId_ == RigidBody::MESH)
		  {
        CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.shape_);
        std::string name = pMesh->GetFileName();
        outBody.fileName_[name.size()]=0;
        memcpy(outBody.fileName_,name.c_str(),name.size());
		  }
		  fwrite(&outBody,sizeof(BodyStorage),1,outFile);
      index++;
    }
  }
	fclose(outFile);
}

void RigidBodyIO::read(World &world, const char *strFileName)
{
	
	FILE *inFile;
	
	inFile = fopen(strFileName,"rb");
	
	//allocate memory for the header
	RigidBodyHeader *header = new RigidBodyHeader();
	fread((RigidBodyHeader*)header,sizeof(RigidBodyHeader),1,inFile);
	//allocate the inbody structure
	BodyStorage *inBody = new BodyStorage();
	
	//set the information from the header
	world.output_      = header->nOutput_;
	//world.timeControl_->SetDeltaT(header->deltaT_);
	//world.timeControl_->SetTime(header->simTime_);
	//world.timeControl_->SetTimeStep(header->timeStep_);
	
	for(int i=0;i<header->nParticles_;i++)
	{
		//read rigid bodies from the file
		fread((BodyStorage*)inBody,sizeof(BodyStorage),1,inFile);
    if (inBody->shapeId_ != RigidBody::COMPOUND)
    {
      RigidBody *pBody = new RigidBody(inBody);
      std::cout << pBody->com_;
      world.rigidBodies_.push_back(pBody);
    }
    else
    {
      inBody->volume_  = 0.0020929212634522424;
      inBody->invMass_ = 0.056066775507211314;
      CompoundBody *c = new CompoundBody(inBody);
      int spheres = inBody->spheres;
      for (int i = 0; i < spheres; i++)
      {
        BodyStorage *subBody = new BodyStorage();
        fread((BodyStorage*)subBody, sizeof(BodyStorage), 1, inFile);
        RigidBody *pBody = new RigidBody(subBody, true);
        c->rigidBodies_.push_back(pBody);
        delete subBody;
      }
      world.rigidBodies_.push_back(c);
    }
	}

	//clean the in body
	delete inBody;
	delete header;
	fclose(inFile);
}

}