#include "rigidbodymotion.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

RigidBodyMotion::RigidBodyMotion(void)
{
	m_pWorld = NULL;
	m_pTimeControl = NULL;
}

RigidBodyMotion::~RigidBodyMotion(void)
{
}

void RigidBodyMotion::UpdateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque)
{
  std::vector<RigidBody*> &vRigidBodies = m_pWorld->rigidBodies_;
  std::vector<RigidBody*>::iterator rIter;
  double densityLiquid = m_pWorld->densityMedium_;
  int count;

  for(rIter=vRigidBodies.begin(),count=0;rIter!=vRigidBodies.end();rIter++,count++)
  {

    RigidBody *body = *rIter;

    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;
    
    VECTOR3 meanForce =  0.5 * (body->force_ + force[count]);

    // compute the mass difference of fluid and solid
    Real massDiff = body->volume_ * (body->density_ - densityLiquid);

    // integrate the force to get an acceleration
    VECTOR3 velUpdate = m_pWorld->timeControl_->GetDeltaT() * body->invMass_*(meanForce);

    // integrate the torque to get angular acceleration
    VECTOR3 angUpdate =  body->getWorldTransformedInvTensor() * (0.5 * m_pWorld->timeControl_->GetDeltaT() * 
                                   (body->torque_ + torque[count]));
    
    body->velocity_ += velUpdate;

    body->setAngVel(body->getAngVel() + angUpdate);

    // std::cout<<"mean force: 0.5 *  "<<body->m_vForce<<" + "<<force[count]<<std::endl;
    // std::cout<<"mean force Val: "<<meanForce<<std::endl;

    // std::cout<<"mass diff: "<<body->m_dVolume<<" * "<<"("<<body->m_dDensity<<"-"<<densityLiquid<<")"<<std::endl;
    // std::cout<<"massDiffVal: "<<massDiff<<std::endl;

    // std::cout<<"velupdate: "<<m_pWorld->m_pTimeControl->GetDeltaT()<<" * "<<body->m_dInvMass<<" * "<<meanForce<<std::endl;
    // std::cout<<"VelupdateVal: "<<velUpdate<<std::endl;

    //std::cout<<"timestep: "<<m_pWorld->m_pTimeControl->GetDeltaT()<<std::endl;
    
  }
}

void RigidBodyMotion::UpdatePosition()
{

	std::vector<RigidBody*> &vRigidBodies = m_pWorld->rigidBodies_;
	std::vector<RigidBody*>::iterator rIter;

	int count = 0;
	for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
	{

		RigidBody *body = *rIter;

		if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
			continue;

		VECTOR3 &pos    = body->com_;
		VECTOR3 &vel    = body->velocity_;
    //body->SetAngVel(VECTOR3(0,0,0));        
		VECTOR3 angvel  = body->getAngVel();
		VECTOR3 &angle  = body->angle_;
    CQuaternionr q0 = body->getQuaternion();
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
    body->setQuaternion(q_next);
    body->setTransformationMatrix(q_next.GetMatrix());
        
    //std::cout<<"Position before: "<<pos<<std::endl;    
    //update velocity
    if(body->isAffectedByGravity())
    { // + massDiff * m_pWorld->GetGravity());
      //std::cout<<"velocity_before: "<<vel<<std::endl;
      vel += ((body->forceResting_ * body->invMass_) + m_pWorld->getGravityEffect(body)) * m_pTimeControl->GetDeltaT();
      //std::cout<<"Gravity part"<<m_pWorld->GetGravityEffect(body) * m_pTimeControl->GetDeltaT()<<std::endl;
      //std::cout<<"velocity_after: "<<vel<<std::endl;
    }

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
    angvel = angvel * body->dampening_;

      body->setAngVel(angvel * 1.0);//0.98;
    
    if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      body->setAngVel(VECTOR3(0,0,0));
    }

    //std::cout<<"Velocity: "<<vel<<std::endl;
    //std::cout<<"Position: "<<pos<<std::endl;
    //std::cout<<"angvel "<<body->GetAngVel()<<std::endl;

    //reset the resting force to 0
    body->forceResting_=VECTOR3(0,0,0);
    body->force_=VECTOR3(0,0,0);
    body->torque_=VECTOR3(0,0,0);    
    count++;
  }//end for
}

RigidBodyMotion::RigidBodyMotion(World* pDomain)
{
	m_pWorld = pDomain;
	m_pTimeControl = pDomain->timeControl_;
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
