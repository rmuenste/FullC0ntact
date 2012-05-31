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

void CRigidBodyIO::Write(CWorld &world, const char *strFileName, bool outputBoundary)
{
	
	FILE *outFile;
	
	outFile = fopen(strFileName,"wb");
	
	sRigidBodyHeader header;
  if(outputBoundary)
	  header.iNumParticles = world.m_vRigidBodies.size();
  else
	  header.iNumParticles = world.m_vRigidBodies.size()-1;
	header.iOutput       = world.m_iOutput;
	header.iTimeStep     = world.m_pTimeControl->GetTimeStep();
	header.dSimTime      = world.m_pTimeControl->GetTime();
	header.dDeltaT       = world.m_pTimeControl->GetDeltaT();
	fwrite(&header,sizeof(sRigidBodyHeader),1,outFile);

	std::vector<CRigidBody*>::iterator rIter;
	rIter = world.m_vRigidBodies.begin();
	for(;rIter!=world.m_vRigidBodies.end();rIter++)
	{
		CRigidBody &body = *(*rIter);
    if((body.m_iShape == CRigidBody::BOUNDARYBOX && !outputBoundary)
    || (body.m_iShape == CRigidBody::COMPOUND)
    || (body.m_iShape == CRigidBody::SUBDOMAIN))      
      continue;
		sRigidBody outBody;
		outBody.m_dDensity  = body.m_dDensity;
		outBody.m_dInvMass  = body.m_dInvMass;
		outBody.m_dVolume   = body.m_dVolume;
		outBody.m_Restitution = body.m_Restitution;
		outBody.m_vCOM      = body.m_vCOM;
		outBody.m_vVelocity = body.m_vVelocity;
		outBody.m_iShape    = body.m_iShape;
    outBody.m_iID       = body.m_iID;
		outBody.m_vAngle    = body.m_vAngle;
		outBody.m_vAngVel   = body.GetAngVel();
		outBody.m_vForce    = body.m_vForce;
		outBody.m_vTorque   = body.m_vTorque;
    outBody.m_vQ        = body.GetQuaternion();
		CAABB3r box         = body.m_pShape->GetAABB();
		outBody.m_Extends[0]= box.m_Extends[0];
		outBody.m_Extends[1]= box.m_Extends[1];
		outBody.m_Extends[2]= box.m_Extends[2];
    if(body.m_bAffectedByGravity)
      outBody.m_iAffectedByGravity=1;
    else
      outBody.m_iAffectedByGravity=0;
    
    memcpy(outBody.m_dTensor,body.m_InvInertiaTensor.m_dEntries,9*sizeof(Real));

    outBody.m_bMatrixAvailable = true;
    
		if(body.m_iShape == CRigidBody::BOX)
		{
			COBB3r *pBox = dynamic_cast<COBB3r*>(body.m_pShape);
			outBody.m_vUVW[0]   = pBox->m_vUVW[0];
			outBody.m_vUVW[1]   = pBox->m_vUVW[1];
			outBody.m_vUVW[2]   = pBox->m_vUVW[2];
		}

    if(body.m_iShape == CRigidBody::MESH)
		{
      CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.m_pShape);
      std::string name = pMesh->GetFileName();
      outBody.m_strFileName[name.size()]=0;
      memcpy(outBody.m_strFileName,name.c_str(),name.size());
		}

		fwrite(&outBody,sizeof(sRigidBody),1,outFile);
	}
	
	fclose(outFile);
}

void CRigidBodyIO::Write(CWorld &world, std::vector<int> &vIndices, const char *strFileName)
{
	
	FILE *outFile;
	int index,bodyid;

	outFile = fopen(strFileName,"wb");

	sRigidBodyHeader header;
  header.iNumParticles = vIndices.size();
	header.iOutput       = world.m_iOutput;
	header.iTimeStep     = world.m_pTimeControl->GetTimeStep();
	header.dSimTime      = world.m_pTimeControl->GetTime();
	header.dDeltaT       = world.m_pTimeControl->GetDeltaT();
	fwrite(&header,sizeof(sRigidBodyHeader),1,outFile);

	std::vector<CRigidBody*>::iterator rIter;
	rIter = world.m_vRigidBodies.begin();
  index=0;
  bodyid=0;
	for(;rIter!=world.m_vRigidBodies.end();rIter++,bodyid++)
	{

    if(index >= vIndices.size())
      break;

    if(vIndices[index]!=bodyid)
    {
      continue;
    }
    else
    {
		  CRigidBody &body = *(*rIter);                                 
		  sRigidBody outBody;
		  outBody.m_dDensity  = body.m_dDensity;
		  outBody.m_dInvMass  = body.m_dInvMass;
		  outBody.m_dVolume   = body.m_dVolume;
		  outBody.m_Restitution = body.m_Restitution;
		  outBody.m_vCOM      = body.m_vCOM;
		  outBody.m_vVelocity = body.m_vVelocity;
		  outBody.m_iShape    = body.m_iShape;
      outBody.m_iID       = body.m_iID;
		  outBody.m_vAngle    = body.m_vAngle;
		  outBody.m_vAngVel   = body.GetAngVel();
		  outBody.m_vForce    = body.m_vForce;
		  outBody.m_vTorque   = body.m_vTorque;
      outBody.m_vQ        = body.GetQuaternion();      
		  CAABB3r box         = body.m_pShape->GetAABB();
		  outBody.m_Extends[0]= box.m_Extends[0];
		  outBody.m_Extends[1]= box.m_Extends[1];
		  outBody.m_Extends[2]= box.m_Extends[2];
      if(body.m_bAffectedByGravity)
        outBody.m_iAffectedByGravity=1;
      else
        outBody.m_iAffectedByGravity=0;

      memcpy(outBody.m_dTensor,body.m_InvInertiaTensor.m_dEntries,9*sizeof(Real));

      outBody.m_bMatrixAvailable = true;
      
		  if(body.m_iShape == CRigidBody::BOX)
		  {
			  COBB3r *pBox = dynamic_cast<COBB3r*>(body.m_pShape);
			  outBody.m_vUVW[0]   = pBox->m_vUVW[0];
			  outBody.m_vUVW[1]   = pBox->m_vUVW[1];
			  outBody.m_vUVW[2]   = pBox->m_vUVW[2];
		  }

      if(body.m_iShape == CRigidBody::MESH)
		  {
        CMeshObject<Real> *pMesh = dynamic_cast<CMeshObject<Real>* >(body.m_pShape);
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

void CRigidBodyIO::Read(CWorld &world, const char *strFileName)
{
	
	FILE *inFile;
	
	inFile = fopen(strFileName,"rb");
	
	//allocate memory for the header
	sRigidBodyHeader *header = new sRigidBodyHeader();
	fread((sRigidBodyHeader*)header,sizeof(sRigidBodyHeader),1,inFile);
	//allocate the inbody structure
	sRigidBody *inBody = new sRigidBody();
	
	//set the information from the header
	world.m_iOutput      = header->iOutput;
	world.m_pTimeControl->SetDeltaT(header->dDeltaT);
	world.m_pTimeControl->SetTime(header->dSimTime);
	world.m_pTimeControl->SetTimeStep(header->iTimeStep);
	
	for(int i=0;i<header->iNumParticles;i++)
	{
		//read rigid bodies from the file
		fread((sRigidBody*)inBody,sizeof(sRigidBody),1,inFile);
		CRigidBody *pBody = new CRigidBody(inBody);
    std::cout<<pBody->m_vCOM;
		world.m_vRigidBodies.push_back(pBody);
	}

	//clean the in body
	delete inBody;
	delete header;
	fclose(inFile);
}

}