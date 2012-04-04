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

#include "collresponsesi.h"
#include <3dmodel.h>
#include <3dmesh.h>
#include <rigidbody.h>
#include <world.h>

namespace i3d {

CCollResponseSI::CCollResponseSI(void)
{
}

CCollResponseSI::~CCollResponseSI(void)
{
}

CCollResponseSI::CCollResponseSI(std::list<CCollisionInfo> *CollInfo, CWorld *pWorld) : CCollResponse(CollInfo,pWorld)
{

}

void CCollResponseSI::Solve()
{

  //return status of our solver
  int ireturnStatus;

  //number of iterations
  int iterations;

  if(this->m_pGraph->m_pEdges->IsEmpty())
    return;

  int i,j;
  Real deltaT = m_pWorld->m_pTimeControl->GetDeltaT();

  //number of different contacts
  int nContacts=0;

  //in the SI framework we apply the external forces before
  //we call the constraint force solver
	std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
	std::vector<CRigidBody*>::iterator rIter;

	int count = 0;
	for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
	{

		CRigidBody *body = *rIter;

		if(body->m_iShape == CRigidBody::BOUNDARYBOX || !body->IsAffectedByGravity())
			continue;

		VECTOR3 &pos    = body->m_vCOM;
		VECTOR3 &vel    = body->m_vVelocity;
    body->SetAngVel(VECTOR3(0,0,0));        
    
    //velocity update
    if(body->IsAffectedByGravity())
    { 
      vel += m_pWorld->GetGravityEffect(body) * m_pWorld->m_pTimeControl->GetDeltaT();
    }

  }//end for

  //call the sequential impulses solver with a fixed
  //number of iterations
  for(iterations=0;iterations<10;iterations++)
  {
    CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
    for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
    {
      CCollisionInfo &info = *hiter;
      if(!info.m_vContacts.empty())
      {
        ApplyImpulse(info);
      }
    }
  }

}//end Solve

void CCollResponseSI::ApplyImpulse(CCollisionInfo &ContactInfo)
{

	double eps1=0.4;
	Real e = 0.4;


  std::vector<CContact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
		CContact &contact = *iter;

    if(contact.m_iState != CCollisionInfo::TOUCHING)
      continue;

		//calculate the collision normal and normalize it
		//compute collision normal
		VECTOR3 vNormal = contact.m_vNormal;

		//the massinverse is needed
		Real massinv = contact.m_pBody0->m_dInvMass + contact.m_pBody1->m_dInvMass;
      
    //VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
    //VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;
    //VECTOR3 impulse  = contact.m_vNormal * impulsemagnitude;

    //VECTOR3 impulse0 =  contact.m_vNormal * (impulsemagnitude * contact.m_pBody0->m_dInvMass);
    //VECTOR3 impulse1 = -contact.m_vNormal * (impulsemagnitude * contact.m_pBody1->m_dInvMass);

    //apply the impulse
    //contact.m_pBody0->ApplyImpulse(vR0, impulse,impulse0);
    //contact.m_pBody1->ApplyImpulse(vR1,-impulse,impulse1);

    //VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
    //VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;
    //VECTOR3 relativeVelocity = (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
    // - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));
    //Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

		//compute relative velocity
		VECTOR3 vAB = contact.m_pBody0->m_vVelocity - contact.m_pBody1->m_vVelocity;

		//velocity along the normal
		Real rNab = vAB * vNormal;

		//calculate the impulse
		Real n2 = vNormal * vNormal;
		Real normalImpulse = rNab * (-1.0/(massinv*n2));

		//create a vector that applies the impulse to the object
		//if the vector is added onto the velocity
		VECTOR3 vImpulse0 = vNormal * normalImpulse  * contact.m_pBody0->m_dInvMass;
		VECTOR3 vImpulse1 = vNormal * -normalImpulse * contact.m_pBody1->m_dInvMass;

	}

}

}