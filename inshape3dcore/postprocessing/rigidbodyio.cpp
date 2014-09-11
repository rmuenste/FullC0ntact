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
#include <compoundbody.h>

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

	outFile = fopen(strFileName, "wb");

	RigidBodyHeader header;
	if (outputBoundary)
		header.nParticles_ = world.rigidBodies_.size();
	else
		header.nParticles_ = world.rigidBodies_.size() - 1;
	header.nOutput_ = world.output_;
	header.timeStep_ = world.timeControl_->GetTimeStep();
	header.simTime_ = world.timeControl_->GetTime();
	header.deltaT_ = world.timeControl_->GetDeltaT();
	fwrite(&header, sizeof(RigidBodyHeader), 1, outFile);

	std::vector<RigidBody*>::iterator rIter;
	rIter = world.rigidBodies_.begin();
	for (; rIter != world.rigidBodies_.end(); rIter++)
	{
		/**needs to be changed to work with compounds


		use the same format specified for the read methods, i.e.
		header
		body 1
		body 2 (compound, n components)
		body 3 (component)
		body 4 (component)
		...
		body n+2 (last component)
		body n+3  (not a component)
		**/

		if ((*rIter)->shapeId_ == RigidBody::COMPOUND){
			CompoundBody &body = *(dynamic_cast<CompoundBody*>(*rIter)); 

			if ((body.shapeId_ == RigidBody::BOUNDARYBOX && !outputBoundary)
				|| (body.shapeId_ == RigidBody::COMPOUND)
				|| (body.shapeId_ == RigidBody::SUBDOMAIN))
				continue;

			BodyStorage outBody;
			outBody.density_ = body.density_;
			outBody.invMass_ = body.invMass_;
			outBody.volume_ = body.volume_;
			outBody.restitution_ = body.restitution_;
			outBody.com_ = body.com_;
			outBody.velocity_ = body.velocity_;
			outBody.shapeId_ = body.shapeId_;
			outBody.id_ = body.iID_;
			outBody.angle_ = body.angle_;
			outBody.angVel_ = body.getAngVel();
			outBody.force_ = body.force_;
			outBody.torque_ = body.torque_;
			outBody.quat_ = body.getQuaternion();
			AABB3r box = body.shape_->getAABB();
			outBody.extents_[0] = box.extents_[0];
			outBody.extents_[1] = box.extents_[1];
			outBody.extents_[2] = box.extents_[2];
			if (body.affectedByGravity_)
				outBody.affectedByGravity_ = 1;
			else
				outBody.affectedByGravity_ = 0;

			memcpy(outBody.tensor_, body.invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

			outBody.matrixAvailable_ = true;
			//set number of components for the compound
			outBody.numofComps_ = body.numofComps_;


			/**since the components are only stored in rigidBodies_ of the compound, write the compound and then all the components **/
			//write the compound
			fwrite(&outBody, sizeof(BodyStorage), 1, outFile);

			//now loop over all the components

			std::vector<RigidBody*>::iterator compIter;
			compIter = outBody.rigidBodies_.begin();
			for (; compIter != outBody.rigidBodies_.end(); compIter++)
			{
				RigidBody &cbody = *(*compIter);
				//same cases as above for rigid bodies,  
				//  collision detection is currently only intended for spheres
				if ((cbody.shapeId_ == RigidBody::BOUNDARYBOX && !outputBoundary)
					|| (cbody.shapeId_ == RigidBody::COMPOUND)
					|| (cbody.shapeId_ == RigidBody::SUBDOMAIN))
					continue;
				BodyStorage CoutBody;
				CoutBody.density_ = cbody.density_;
				CoutBody.invMass_ = cbody.invMass_;
				CoutBody.volume_ = cbody.volume_;
				CoutBody.restitution_ = cbody.restitution_;
				CoutBody.com_ = cbody.com_;
				CoutBody.velocity_ = cbody.velocity_;
				CoutBody.shapeId_ = cbody.shapeId_;
				CoutBody.id_ = cbody.iID_;
				CoutBody.angle_ = cbody.angle_;
				CoutBody.angVel_ = cbody.getAngVel();
				CoutBody.force_ = cbody.force_;
				CoutBody.torque_ = cbody.torque_;
				CoutBody.quat_ = cbody.getQuaternion();
				AABB3r cbox = cbody.shape_->getAABB();
				CoutBody.extents_[0] = cbox.extents_[0];
				CoutBody.extents_[1] = cbox.extents_[1];
				CoutBody.extents_[2] = cbox.extents_[2];
				if (cbody.affectedByGravity_)
					CoutBody.affectedByGravity_ = 1;
				else
					CoutBody.affectedByGravity_ = 0;

				memcpy(CoutBody.tensor_, cbody.invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

				CoutBody.matrixAvailable_ = true;

				if (cbody.shapeId_ == RigidBody::BOX)
				{
					OBB3r *cpBox = dynamic_cast<OBB3r*>(cbody.shape_);
					CoutBody.uvw_[0] = cpBox->uvw_[0];
					CoutBody.uvw_[1] = cpBox->uvw_[1];
					CoutBody.uvw_[2] = cpBox->uvw_[2];
				}
				//this case should not be needed
				if (cbody.shapeId_ == RigidBody::MESH)
				{
					CMeshObject<Real> *cpMesh = dynamic_cast<CMeshObject<Real>*>(body.shape_);
					std::string name = cpMesh->GetFileName();
					CoutBody.fileName_[name.size()] = 0;
					memcpy(CoutBody.fileName_, name.c_str(), name.size());
				}
				//write the component body
				fwrite(&CoutBody, sizeof(BodyStorage), 1, outFile);

			}
			//when finished looping over all components, regular writing process can be resumed
			//end of compound loop
		}



		//else the body is a rigid body 
		else{
			RigidBody &body = *(*rIter);


			if ((body.shapeId_ == RigidBody::BOUNDARYBOX && !outputBoundary)
				|| (body.shapeId_ == RigidBody::COMPOUND)
				|| (body.shapeId_ == RigidBody::SUBDOMAIN))
				continue;
			BodyStorage outBody;
			outBody.density_ = body.density_;
			outBody.invMass_ = body.invMass_;
			outBody.volume_ = body.volume_;
			outBody.restitution_ = body.restitution_;
			outBody.com_ = body.com_;
			outBody.velocity_ = body.velocity_;
			outBody.shapeId_ = body.shapeId_;
			outBody.id_ = body.iID_;
			outBody.angle_ = body.angle_;
			outBody.angVel_ = body.getAngVel();
			outBody.force_ = body.force_;
			outBody.torque_ = body.torque_;
			outBody.quat_ = body.getQuaternion();
			AABB3r box = body.shape_->getAABB();
			outBody.extents_[0] = box.extents_[0];
			outBody.extents_[1] = box.extents_[1];
			outBody.extents_[2] = box.extents_[2];
			if (body.affectedByGravity_)
				outBody.affectedByGravity_ = 1;
			else
				outBody.affectedByGravity_ = 0;

			memcpy(outBody.tensor_, body.invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

			outBody.matrixAvailable_ = true;

			if (body.shapeId_ == RigidBody::BOX)
			{
				OBB3r *pBox = dynamic_cast<OBB3r*>(body.shape_);
				outBody.uvw_[0] = pBox->uvw_[0];
				outBody.uvw_[1] = pBox->uvw_[1];
				outBody.uvw_[2] = pBox->uvw_[2];
			}

			if (body.shapeId_ == RigidBody::MESH)
			{
				CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>*>(body.shape_);
				std::string name = pMesh->GetFileName();
				outBody.fileName_[name.size()] = 0;
				memcpy(outBody.fileName_, name.c_str(), name.size());
			}


			fwrite(&outBody, sizeof(BodyStorage), 1, outFile);
		}

		fclose(outFile);

	}
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
		//compound support  missing here
		//should work in the same way the read method is extended to support compounds, by dynamic_cast pointers
		
		if ((*rIter)->shapeId_ == RigidBody::COMPOUND){
			CompoundBody &body = *(dynamic_cast<CompoundBody*>(*rIter)); //??
			BodyStorage outBody;
			outBody.density_ = body.density_;
			outBody.invMass_ = body.invMass_;
			outBody.volume_ = body.volume_;
			outBody.restitution_ = body.restitution_;
			outBody.com_ = body.com_;
			outBody.velocity_ = body.velocity_;
			outBody.shapeId_ = body.shapeId_;
			outBody.id_ = body.iID_;
			outBody.angle_ = body.angle_;
			outBody.angVel_ = body.getAngVel();
			outBody.force_ = body.force_;
			outBody.torque_ = body.torque_;
			outBody.quat_ = body.getQuaternion();
			AABB3r box = body.shape_->getAABB();
			outBody.extents_[0] = box.extents_[0];
			outBody.extents_[1] = box.extents_[1];
			outBody.extents_[2] = box.extents_[2];
			if (body.affectedByGravity_)
				outBody.affectedByGravity_ = 1;
			else
				outBody.affectedByGravity_ = 0;

			memcpy(outBody.tensor_, body.invInertiaTensor_.m_dEntries, 9 * sizeof(Real));

			outBody.matrixAvailable_ = true;


			fwrite(&outBody, sizeof(BodyStorage), 1, outFile);
			index++;
		}
		
		else{ 


		  RigidBody &body = *(*rIter);                                 
		  BodyStorage outBody;
		  outBody.density_  = body.density_;
		  outBody.invMass_  = body.invMass_;
		  outBody.volume_   = body.volume_;
		  outBody.restitution_ = body.restitution_;
		  outBody.com_      = body.com_;
		  outBody.velocity_ = body.velocity_;
		  outBody.shapeId_    = body.shapeId_;
      outBody.id_       = body.iID_;
		  outBody.angle_    = body.angle_;
		  outBody.angVel_   = body.getAngVel();
		  outBody.force_    = body.force_;
		  outBody.torque_   = body.torque_;
      outBody.quat_        = body.getQuaternion();      
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
      index++;
	  }
    }
  }
	fclose(outFile);
}

void RigidBodyIO::read(World &world, const char *strFileName)
{

	FILE *inFile;

	inFile = fopen(strFileName, "rb");

	//allocate memory for the header
	RigidBodyHeader *header = new RigidBodyHeader();
	fread((RigidBodyHeader*)header, sizeof(RigidBodyHeader), 1, inFile);
	//allocate the inbody structure
	BodyStorage *inBody = new BodyStorage();

	//set the information from the header
	world.output_ = header->nOutput_;
	world.timeControl_->SetDeltaT(header->deltaT_);
	world.timeControl_->SetTime(header->simTime_);
	world.timeControl_->SetTimeStep(header->timeStep_);

	for (int i = 0; i < header->nParticles_; i++)
	{
		//read rigid bodies from the file
		fread((BodyStorage*)inBody, sizeof(BodyStorage), 1, inFile);

		//different approach for compounds
		/** idea: use two pointers; since the input file is structured as
		header
		rigidbody
		rigidbody
		...
		rigidbody

		use the following format

		rigidbody (some body that is not part of a compound)
		compoundbody (+number of components)
		rigidbody (component)
		rigidbody (component)

		then read and construct compound body with empty rigidBodies_ vector. Use a second pointer during iteration for the components, incrementing the second pointer
		for each body read while initializing the component bodies. use a counter for number of components, and then set the first pointer to the object the second points to,
		and finally delete the second pointer.
		Don't save the component bodies in the world's rigidBodies_ vector, since the collision detection algorithm will not know that they belong to the compound otherwise.
		**/

		if (inBody->shapeId_ == RigidBody::COMPOUND){
			CompoundBody *pBody = new CompoundBody(inBody);
			std::cout << pBody->com_;
			//the compound has to be stored in the vector containing all rigid bodies in the simulation
			world.rigidBodies_.push_back(pBody);
			//number of components of the compound is stored in integer variable numofComps_
			// loop for initializing the components; after the loop, i has the value i + numofComps
			for (int j = 0; j < pBody->numofComps_; j++){

				BodyStorage *inBodyComp = new BodyStorage();
				//read the component rigid bodies
				fread((BodyStorage*)inBodyComp, sizeof(BodyStorage), 1, inFile);


				RigidBody *pBodyComp = new RigidBody(inBodyComp);
				std::cout << pBodyComp->com_;

				//add pointer to the component to rigidBodies_ vector
				pBody->rigidBodies_.push_back(pBodyComp);
				//add the body to the overall list of rigid bodies
				//world.rigidBodies_.push_back(pBodyComp);

				//clean up inBodyComp
				delete inBodyComp;

			}
			i += pBody->numofComps_;



		}
		//else the body is a not part of a compound
		else{

			RigidBody *pBody = new RigidBody(inBody);
			std::cout << pBody->com_;
			world.rigidBodies_.push_back(pBody);
		}
	}

	//clean the in body
	delete inBody;
	delete header;
	fclose(inFile);
 }

}