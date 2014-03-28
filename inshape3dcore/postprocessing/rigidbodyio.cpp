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

CRigidBodyIO::CRigidBodyIO()
{

}

CRigidBodyIO::~CRigidBodyIO()
{

}

void CRigidBodyIO::Write(World &world, const char *strFileName, bool outputBoundary)
{
	
	FILE *outFile;
	
	outFile = fopen(strFileName,"wb");
	
	sRigidBodyHeader header;
  if(outputBoundary)
	  header.iNumParticles = world.rigidBodies_.size();
  else
	  header.iNumParticles = world.rigidBodies_.size()-1;
	header.iOutput       = world.output_;
	header.iTimeStep     = world.timeControl_->GetTimeStep();
	header.dSimTime      = world.timeControl_->GetTime();
	header.dDeltaT       = world.timeControl_->GetDeltaT();
	fwrite(&header,sizeof(sRigidBodyHeader),1,outFile);

	std::vector<RigidBody*>::iterator rIter;
	rIter = world.rigidBodies_.begin();
	for(;rIter!=world.rigidBodies_.end();rIter++)
	{
		RigidBody &body = *(*rIter);
    if((body.shapeId_ == RigidBody::BOUNDARYBOX && !outputBoundary)
    || (body.shapeId_ == RigidBody::COMPOUND)
    || (body.shapeId_ == RigidBody::SUBDOMAIN))      
      continue;
		sRigidBody outBody;
		outBody.m_dDensity  = body.density_;
		outBody.m_dInvMass  = body.invMass_;
		outBody.m_dVolume   = body.volume_;
		outBody.m_Restitution = body.restitution_;
		outBody.m_vCOM      = body.com_;
		outBody.m_vVelocity = body.velocity_;
		outBody.m_iShape    = body.shapeId_;
    outBody.m_iID       = body.iID_;
		outBody.m_vAngle    = body.angle_;
		outBody.m_vAngVel   = body.getAngVel();
		outBody.m_vForce    = body.force_;
		outBody.m_vTorque   = body.torque_;
    outBody.m_vQ        = body.getQuaternion();
		CAABB3r box         = body.shape_->getAABB();
		outBody.m_Extends[0]= box.m_Extends[0];
		outBody.m_Extends[1]= box.m_Extends[1];
		outBody.m_Extends[2]= box.m_Extends[2];
    if(body.affectedByGravity_)
      outBody.m_iAffectedByGravity=1;
    else
      outBody.m_iAffectedByGravity=0;
    
    memcpy(outBody.m_dTensor,body.invInertiaTensor_.m_dEntries,9*sizeof(Real));
    
    outBody.m_bMatrixAvailable = true;
    
		if(body.shapeId_ == RigidBody::BOX)
		{
			OBB3r *pBox = dynamic_cast<OBB3r*>(body.shape_);
			outBody.m_vUVW[0]   = pBox->uvw_[0];
			outBody.m_vUVW[1]   = pBox->uvw_[1];
			outBody.m_vUVW[2]   = pBox->uvw_[2];
		}

    if(body.shapeId_ == RigidBody::MESH)
		{
      CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.shape_);
      std::string name = pMesh->GetFileName();
      outBody.m_strFileName[name.size()]=0;
      memcpy(outBody.m_strFileName,name.c_str(),name.size());
		}

		fwrite(&outBody,sizeof(sRigidBody),1,outFile);
	}
	
	fclose(outFile);
}

void CRigidBodyIO::Write(World &world, std::vector<int> &vIndices, const char *strFileName)
{
	
	FILE *outFile;
	int index,bodyid;

	outFile = fopen(strFileName,"wb");

	sRigidBodyHeader header;
  header.iNumParticles = vIndices.size();
	header.iOutput       = world.output_;
	header.iTimeStep     = world.timeControl_->GetTimeStep();
	header.dSimTime      = world.timeControl_->GetTime();
	header.dDeltaT       = world.timeControl_->GetDeltaT();
	fwrite(&header,sizeof(sRigidBodyHeader),1,outFile);

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
		  sRigidBody outBody;
		  outBody.m_dDensity  = body.density_;
		  outBody.m_dInvMass  = body.invMass_;
		  outBody.m_dVolume   = body.volume_;
		  outBody.m_Restitution = body.restitution_;
		  outBody.m_vCOM      = body.com_;
		  outBody.m_vVelocity = body.velocity_;
		  outBody.m_iShape    = body.shapeId_;
      outBody.m_iID       = body.iID_;
		  outBody.m_vAngle    = body.angle_;
		  outBody.m_vAngVel   = body.getAngVel();
		  outBody.m_vForce    = body.force_;
		  outBody.m_vTorque   = body.torque_;
      outBody.m_vQ        = body.getQuaternion();      
		  CAABB3r box         = body.shape_->getAABB();
		  outBody.m_Extends[0]= box.m_Extends[0];
		  outBody.m_Extends[1]= box.m_Extends[1];
		  outBody.m_Extends[2]= box.m_Extends[2];
      if(body.affectedByGravity_)
        outBody.m_iAffectedByGravity=1;
      else
        outBody.m_iAffectedByGravity=0;

      memcpy(outBody.m_dTensor,body.invInertiaTensor_.m_dEntries,9*sizeof(Real));

      outBody.m_bMatrixAvailable = true;
      
		  if(body.shapeId_ == RigidBody::BOX)
		  {
			  OBB3r *pBox = dynamic_cast<OBB3r*>(body.shape_);
			  outBody.m_vUVW[0]   = pBox->uvw_[0];
			  outBody.m_vUVW[1]   = pBox->uvw_[1];
			  outBody.m_vUVW[2]   = pBox->uvw_[2];
		  }

      if(body.shapeId_ == RigidBody::MESH)
		  {
        CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.shape_);
        std::string name = pMesh->GetFileName();
        outBody.m_strFileName[name.size()]=0;
        memcpy(outBody.m_strFileName,name.c_str(),name.size());
		  }
		  fwrite(&outBody,sizeof(sRigidBody),1,outFile);
      index++;
    }
  }
	fclose(outFile);
}

void CRigidBodyIO::Read(World &world, const char *strFileName)
{
	
	FILE *inFile;
	
	inFile = fopen(strFileName,"rb");
	
	//allocate memory for the header
	sRigidBodyHeader *header = new sRigidBodyHeader();
	fread((sRigidBodyHeader*)header,sizeof(sRigidBodyHeader),1,inFile);
	//allocate the inbody structure
	sRigidBody *inBody = new sRigidBody();
	
	//set the information from the header
	world.output_      = header->iOutput;
	world.timeControl_->SetDeltaT(header->dDeltaT);
	world.timeControl_->SetTime(header->dSimTime);
	world.timeControl_->SetTimeStep(header->iTimeStep);
	
	for(int i=0;i<header->iNumParticles;i++)
	{
		//read rigid bodies from the file
		fread((sRigidBody*)inBody,sizeof(sRigidBody),1,inFile);
		RigidBody *pBody = new RigidBody(inBody);
    std::cout<<pBody->com_;
		world.rigidBodies_.push_back(pBody);
	}

	//clean the in body
	delete inBody;
	delete header;
	fclose(inFile);
}

}