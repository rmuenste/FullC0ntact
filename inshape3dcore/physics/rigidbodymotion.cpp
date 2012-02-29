#include "rigidbodymotion.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

CRigidBodyMotion::CRigidBodyMotion(void)
{
	m_pWorld = NULL;
	m_pTimeControl = NULL;
}

CRigidBodyMotion::~CRigidBodyMotion(void)
{
}

void CRigidBodyMotion::UpdateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque)
{
  std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
  std::vector<CRigidBody*>::iterator rIter;
  double densityLiquid = 1.0;
  int count;

  for(rIter=vRigidBodies.begin(),count=0;rIter!=vRigidBodies.end();rIter++,count++)
  {

    CRigidBody *body = *rIter;

    if(body->m_iShape == CRigidBody::BOUNDARYBOX)
      continue;
    
    VECTOR3 meanForce =  0.5 * (body->m_vForce + force[count]);
    Real massDiff = body->m_dVolume * (body->m_dDensity - densityLiquid);
    VECTOR3 velUpdate = m_pWorld->m_pTimeControl->GetDeltaT() * body->m_dInvMass*(meanForce);
                                   
    VECTOR3 angUpdate =  body->GetWorldTransformedInvTensor() * (0.5 * m_pWorld->m_pTimeControl->GetDeltaT() * 
                                   (body->m_vTorque + torque[count]));
    
    body->m_vVelocity += velUpdate;

    body->SetAngVel(body->GetAngVel() + angUpdate);

    // std::cout<<"mean force: 0.5 *  "<<body->m_vForce<<" + "<<force[count]<<std::endl;
    // std::cout<<"mean force Val: "<<meanForce<<std::endl;

    // std::cout<<"mass diff: "<<body->m_dVolume<<" * "<<"("<<body->m_dDensity<<"-"<<densityLiquid<<")"<<std::endl;
    // std::cout<<"massDiffVal: "<<massDiff<<std::endl;

    // std::cout<<"velupdate: "<<m_pWorld->m_pTimeControl->GetDeltaT()<<" * "<<body->m_dInvMass<<" * "<<meanForce<<std::endl;
    // std::cout<<"VelupdateVal: "<<velUpdate<<std::endl;

    //std::cout<<"timestep: "<<m_pWorld->m_pTimeControl->GetDeltaT()<<std::endl;
    
  }
}

void CRigidBodyMotion::UpdatePosition()
{

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
		VECTOR3 angvel  = body->GetAngVel();
		VECTOR3 &angle  = body->m_vAngle;
    CQuaternionr q0 = body->GetQuaternion();
    CQuaternionr q1(angvel.x,angvel.y,angvel.z,0);
    CQuaternionr q0q1;
    VECTOR3 vq0(q0.x,q0.y,q0.z);
    q0q1.w = -(angvel*vq0);
    VECTOR3 v = VECTOR3::Cross(angvel,vq0) + q0.w*angvel;
    q0q1.x = v.x;
    q0q1.y = v.y;
    q0q1.z = v.z;
    
    CQuaternionr q_next = q0 + (m_pTimeControl->GetDeltaT() * 0.5 * (q0q1));
    
    q_next.Normalize();
    
    //update orientation    
    //body->SetQuaternion(q_next);
    //body->SetTransformationMatrix(q_next.GetMatrix());
    
    
    //std::cout<<"Position before: "<<pos<<std::endl;    
    //update velocity
    if(body->IsAffectedByGravity())
    { // + massDiff * m_pWorld->GetGravity());
      //std::cout<<"velocity_before: "<<vel<<std::endl;
      vel += ((body->m_vForceResting * body->m_dInvMass) + m_pWorld->GetGravityEffect(body)) * m_pTimeControl->GetDeltaT();
      //std::cout<<"Gravity part"<<m_pWorld->GetGravityEffect(body) * m_pTimeControl->GetDeltaT()<<std::endl;
      //std::cout<<"velocity_after: "<<vel<<std::endl;
    }

    if(body->m_bTouchesGround)
    {
      //vel.x = vel.x * 0.98;
      //vel.y = vel.y * 0.98;
    }

    //Prevent floating point errors
    if(vel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      vel=VECTOR3(0,0,0);
    }

    //update the position
    pos += vel * m_pTimeControl->GetDeltaT();

    //update ang velocity
    angvel = angvel * body->m_dDampening;

    if(body->m_bTouchesGround)
      body->SetAngVel(angvel * 1.0);//0.98;
    
    if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      body->SetAngVel(VECTOR3(0,0,0));
    }

    //std::cout<<"Velocity: "<<vel<<std::endl;
    //std::cout<<"Position: "<<pos<<std::endl;
    //std::cout<<"angvel "<<body->GetAngVel()<<std::endl;

    //reset the resting force to 0
    body->m_vForceResting=VECTOR3(0,0,0);
    body->m_vForce=VECTOR3(0,0,0);
    body->m_vTorque=VECTOR3(0,0,0);    
    body->m_bTouchesGround = false;
    count++;
  }//end for
}

CRigidBodyMotion::CRigidBodyMotion(CWorld* pDomain)
{
	m_pWorld = pDomain;
	m_pTimeControl = pDomain->m_pTimeControl;
}

//void CRigidBodyMotion::ApplyImpuse(CContact &contact, Real dForce)
//{
//
//  CRigidBody *pBody0 = contact.m_pBody0;
//  CRigidBody *pBody1 = contact.m_pBody1;
//
//  VECTOR3 vR0 = contact.m_vPosition0 - contact.m_pBody0->m_vCOM;
//  VECTOR3 vR1 = contact.m_vPosition1 - contact.m_pBody1->m_vCOM;
//
//  MATRIX3X3 mInvInertiaTensor0 = contact.m_pBody0->GetWorldTransformedInvTensor();
//  MATRIX3X3 mInvInertiaTensor1 = contact.m_pBody1->GetWorldTransformedInvTensor();
//
//	contact.m_pBody0->m_vVelocity += contact.m_vNormal * (dForce * contact.m_pBody0->m_dInvMass);
//  contact.m_pBody0->m_vAngVel   += mInvInertiaTensor0 * (VECTOR3::Cross(vR0,dForce * contact.m_vNormal));
//
//	contact.m_pBody1->m_vVelocity -= contact.m_vNormal * (dForce * contact.m_pBody1->m_dInvMass);
//  contact.m_pBody1->m_vAngVel   -= mInvInertiaTensor1 * (VECTOR3::Cross(vR1,dForce * contact.m_vNormal));
//
//}

}
