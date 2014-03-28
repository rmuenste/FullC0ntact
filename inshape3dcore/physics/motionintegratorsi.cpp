#include "motionintegratorsi.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

CMotionIntegratorSI::CMotionIntegratorSI(void)
{
	m_pWorld = NULL;
	m_pTimeControl = NULL;
}

CMotionIntegratorSI::CMotionIntegratorSI(World* pDomain)
{
	m_pWorld = pDomain;
	m_pTimeControl = pDomain->timeControl_;
}

CMotionIntegratorSI::~CMotionIntegratorSI(void)
{
}

void CMotionIntegratorSI::UpdatePosition()
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
    angvel         += body->getBiasAngVel();

    CQuaternionr q0 = body->getQuaternion();
    CQuaternionr q1(angvel.x,angvel.y,angvel.z,0);
    
    //get the bias velocity

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
    
//     std::cout<<"position: "<<pos;
//     std::cout<<"velocity: "<<vel;    
    
    //Prevent floating point errors
    if(vel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      vel=VECTOR3(0,0,0);
    }

//     if(body->IsAffectedByGravity())
//     { 
//       vel += (m_pWorld->GetGravityEffect(body)) * m_pTimeControl->GetDeltaT();
//     }
        
    //add bias velocity
    pos += body->getBiasVelocity() * m_pTimeControl->GetDeltaT();

    //update the position
    pos += vel * m_pTimeControl->GetDeltaT();

    //update ang velocity
    angvel = angvel * body->dampening_;

    if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      body->setAngVel(VECTOR3(0,0,0));
    }

    body->force_=VECTOR3(0,0,0);
    body->torque_=VECTOR3(0,0,0);
    body->setBiasAngVel(VECTOR3(0,0,0));
    body->setBiasVelocity(VECTOR3(0,0,0));
    count++;
  }//end for
}

}
