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
#include <vectorn.h>

namespace i3d {

/**
 * You need indeed two friction directions then. It might help that you align the first friction axis 
 * with the relative velocity in the tangent plane. It is also important to project the last friction 
 * impulse onto the new friction directions for warmstarting. 
 * Say you have your two accumulated friction impulses lambda1 and lambda2 and 
 * the associated direction vectors tangent1 and tangent2 from the last and current frame. 
 * Then you need to do the following to project the impulse:
 * Vec3 OldImpulse = Lambda1 * OldTangent1 + Lambda2 * OldTangent2;
 * Lambda1 = dot( Lambda1, NewTangent1 );
 * Lambda2 = dot( Lambda2, NewTangent2 );
 * Don't skip friction if the relative velocity in the tangent plane is zero. 
 * You create an arbitrary frame then. You can look at dPlaneSpace() in the ODE if you need an example.
*/
  
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

  m_iTotalContacts = 0;
  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {
    CCollisionInfo &info = *hiter;
    if(!info.m_vContacts.empty())
    {
      PreComputeConstants(info);
    }
  }

  //initialize the defect vector
  m_vDef = CVectorNr(m_iTotalContacts);

  //call the sequential impulses solver with a fixed
  //number of iterations
  for(iterations=0;iterations<40;iterations++)
  {
    hiter = m_pGraph->m_pEdges->begin();
    for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
    {
      CCollisionInfo &info = *hiter;
      if(!info.m_vContacts.empty())
      {
        ApplyImpulse(info);
      }
      
      //we now have to synchronize the remote bodies
      
      //push velocity difference to other domain 
      
      //pull velocity difference from other domain
      
      //apply velocity difference
      
    }
    //std::cout<<iterations<<" "<<ComputeDefect()<<std::endl;

  }

}//end Solve

void CCollResponseSI::PreComputeConstants(CCollisionInfo &ContactInfo)
{

  std::vector<CContact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
		CContact &contact = *iter;

    if(contact.m_iState != CCollisionInfo::TOUCHING)
      continue;

    m_iTotalContacts++;

    ComputeTangentSpace(contact.m_vNormal,contact.m_vTangentU,contact.m_vTangentV);
    
		//the massinverse is needed
		Real massinv = contact.m_pBody0->m_dInvMass + contact.m_pBody1->m_dInvMass;

    VECTOR3 vR0  = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
    VECTOR3 vR1  = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

    VECTOR3 vCR0 = VECTOR3::Cross(vR0,contact.m_vNormal);
    VECTOR3 vCR1 = VECTOR3::Cross(vR1,contact.m_vNormal);

    VECTOR3 vDA0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vCR0;
    VECTOR3 vDA1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vCR1;

    contact.m_dMassNormal  = 1.0/(massinv + vCR0 * vDA0 + vCR1 * vDA1);

    contact.m_dRestitution = 0.0;

    VECTOR3 impulse0 =  contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody0->m_dInvMass);
    VECTOR3 impulse1 = -contact.m_vNormal * (contact.m_dAccumulatedNormalImpulse * contact.m_pBody1->m_dInvMass);

    VECTOR3 impulse  = contact.m_vNormal * contact.m_dAccumulatedNormalImpulse;

    //apply the impulse
    contact.m_pBody0->ApplyImpulse(vR0, impulse,impulse0);
    contact.m_pBody1->ApplyImpulse(vR1,-impulse,impulse1);
    
    //contact.m_dAccumulatedNormalImpulse = 0.0;
    
    //precompute for the u-friction component
    VECTOR3 vTUR0 = VECTOR3::Cross(vR0,contact.m_vTangentU);
    VECTOR3 vTUR1 = VECTOR3::Cross(vR1,contact.m_vTangentU);    
    
    VECTOR3 vDTU0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vTUR0;
    VECTOR3 vDTU1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vTUR1;
    
    contact.m_dMassTangentU = 1.0/(massinv + vTUR0 * vDTU0 + vTUR1 * vDTU1);
    
    contact.m_dAccumulatedTangentImpulseU = 0.0;
    
    //precompute for the v-friction component
    VECTOR3 vTVR0 = VECTOR3::Cross(vR0,contact.m_vTangentV);
    VECTOR3 vTVR1 = VECTOR3::Cross(vR1,contact.m_vTangentV);    
    
    VECTOR3 vDTV0 = contact.m_pBody0->GetWorldTransformedInvTensor() * vTVR0;
    VECTOR3 vDTV1 = contact.m_pBody1->GetWorldTransformedInvTensor() * vTVR1;
    
    contact.m_dMassTangentV = 1.0/(massinv + vTVR0 * vDTV0 + vTVR1 * vDTV1);
    
    contact.m_dAccumulatedTangentImpulseV = 0.0;

	}

}

void CCollResponseSI::ApplyImpulse(CCollisionInfo &ContactInfo)
{

	double eps=0.0;

  std::vector<CContact>::iterator iter;
	for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	{
      
		CContact &contact = *iter;

    if(contact.m_iState != CCollisionInfo::TOUCHING)
      continue;

    VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
    VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

    VECTOR3 relativeVelocity = (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
                              - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

    Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

    //-(1+e) * rNV = -rNV - e * rNV
    Real normalImpulse    = contact.m_dMassNormal * (contact.m_dRestitution - relativeNormalVelocity);

    Real oldNormalImpulse = contact.m_dAccumulatedNormalImpulse;

    //clamp the accumulated impulse to 0
    contact.m_dAccumulatedNormalImpulse = std::max(oldNormalImpulse+normalImpulse,0.0);

    //set the impulse magnitude to the difference between 
    //the accumulated impulse and the old impulse
    normalImpulse    = contact.m_dAccumulatedNormalImpulse - oldNormalImpulse;

    VECTOR3 impulse  = contact.m_vNormal * normalImpulse;

    VECTOR3 impulse0 =  contact.m_vNormal * (normalImpulse * contact.m_pBody0->m_dInvMass);
    VECTOR3 impulse1 = -contact.m_vNormal * (normalImpulse * contact.m_pBody1->m_dInvMass);

    //apply the impulse
    contact.m_pBody0->ApplyImpulse(vR0, impulse,impulse0);
    contact.m_pBody1->ApplyImpulse(vR1,-impulse,impulse1);
    
    //compute the friction impulse
    Real maxTangentImpulse = (contact.m_pBody0->m_dFriction * contact.m_pBody1->m_dFriction) * contact.m_dAccumulatedNormalImpulse;
    
    //start with the u-tangent vector
    Real relativeTangentVelocity = relativeVelocity * contact.m_vTangentU;

    Real tangentImpulseU = contact.m_dMassTangentU * (-relativeTangentVelocity);
    
    //save the old accumulated impulse
    Real oldTangentImpulse = contact.m_dAccumulatedTangentImpulseU;
    
    //clamp the tangent impulse 
    contact.m_dAccumulatedTangentImpulseU = std::max(std::min(oldTangentImpulse+tangentImpulseU,maxTangentImpulse),
                                                    -maxTangentImpulse);

    //get the delta impulse
    tangentImpulseU = contact.m_dAccumulatedTangentImpulseU - oldTangentImpulse;
    
    VECTOR3 tangentImpulse = contact.m_vTangentU * tangentImpulseU;
    
    VECTOR3 tangentImpulseU0 =  contact.m_vTangentU * (tangentImpulseU * contact.m_pBody0->m_dInvMass);
    VECTOR3 tangentImpulseU1 = -contact.m_vTangentU * (tangentImpulseU * contact.m_pBody1->m_dInvMass); 
    
    //apply the tangent impulse
    contact.m_pBody0->ApplyImpulse(vR0, tangentImpulse,tangentImpulseU0);
    contact.m_pBody1->ApplyImpulse(vR1,-tangentImpulse,tangentImpulseU1);    

    //same procedure for the v-tangent vector
    relativeTangentVelocity = relativeVelocity * contact.m_vTangentV;

    Real tangentImpulseV = contact.m_dMassTangentV * (-relativeTangentVelocity);
    
    //save the old accumulated impulse
    oldTangentImpulse = contact.m_dAccumulatedTangentImpulseV;
    
    //clamp the tangent impulse 
    contact.m_dAccumulatedTangentImpulseV = std::max(std::min(oldTangentImpulse+tangentImpulseV,maxTangentImpulse),
                                                    -maxTangentImpulse);

    //get the delta impulse
    tangentImpulseV = contact.m_dAccumulatedTangentImpulseV - oldTangentImpulse;
    
    tangentImpulse = contact.m_vTangentV * tangentImpulseV;
    
    VECTOR3 tangentImpulseV0 =  contact.m_vTangentV * (tangentImpulseV * contact.m_pBody0->m_dInvMass);
    VECTOR3 tangentImpulseV1 = -contact.m_vTangentV * (tangentImpulseV * contact.m_pBody1->m_dInvMass); 
    
    //apply the tangent impulse
    contact.m_pBody0->ApplyImpulse(vR0, tangentImpulse,tangentImpulseV0);
    contact.m_pBody1->ApplyImpulse(vR1,-tangentImpulse,tangentImpulseV1);    

	}

}

void CCollResponseSI::ComputeTangentSpace(const VECTOR3& normal, VECTOR3& t1, VECTOR3& t2)
{
  
  //based on the value of the z-component
  //we approximate a first tangent vector
  if(fabs(normal.z) > 0.7071067)
  {
    Real a = normal.y * normal.y + normal.z * normal.z;
    Real k = 1.0/(sqrt(a));    
    t1.x   = 0.0;
    t1.y   = -normal.z*k;
    t1.z   = normal.y *k;
    
    //compute the 2nd tangent vector by:
    //t2 = n x t1
    t2.x   = a*k;
    t2.y   = -normal.x*t1.z;
    t2.z   = normal.x *t1.y;
  }
  else
  {
    Real a = normal.x * normal.x + normal.y * normal.y;
    Real k = 1.0/(sqrt(a));    
    t1.x   = -normal.y*k;
    t1.y   = normal.x*k;
    t1.z   = 0.0;
    
    //compute the 2nd tangent vector by:
    //t2 = n x t1
    t2.x   = -normal.z*t1.y;
    t2.y   = normal.z *t1.x;
    t2.z   = a*k;
  }

}

Real CCollResponseSI::ComputeDefect()
{

  CCollisionHash::iterator hiter = m_pGraph->m_pEdges->begin();
  hiter = m_pGraph->m_pEdges->begin();
  int count = 0;
  for(;hiter!=m_pGraph->m_pEdges->end();hiter++)
  {
    CCollisionInfo &ContactInfo = *hiter;
    std::vector<CContact>::iterator iter;
	  for(iter=ContactInfo.m_vContacts.begin();iter!=ContactInfo.m_vContacts.end();iter++)
	  {
      
		  CContact &contact = *iter;

      if(contact.m_iState != CCollisionInfo::TOUCHING)
        continue;

      VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
      VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;

      VECTOR3 relativeVelocity = (contact.m_pBody0->m_vVelocity + (VECTOR3::Cross(contact.m_pBody0->GetAngVel(),vR0))
                                - contact.m_pBody1->m_vVelocity - (VECTOR3::Cross(contact.m_pBody1->GetAngVel(),vR1)));

      Real relativeNormalVelocity = (relativeVelocity*contact.m_vNormal);

      m_vDef(count++)=relativeNormalVelocity;
    }
  }
  return m_vDef.norm(); 
}

}