#include "motionintegratorsi.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

MotionIntegratorSI::MotionIntegratorSI(void)
{
	world_ = NULL;
	timeControl_ = NULL;
}

MotionIntegratorSI::MotionIntegratorSI(World* pDomain)
{
	world_ = pDomain;
	timeControl_ = pDomain->timeControl_;
}

MotionIntegratorSI::~MotionIntegratorSI(void)
{
}

void MotionIntegratorSI::updatePosition()
{

	std::vector<RigidBody*> &vRigidBodies = world_->rigidBodies_;
	std::vector<RigidBody*>::iterator rIter;

	int count = 0;
	for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
	{

		RigidBody *body = *rIter;

		//if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
		//	continue;

		VECTOR3 &pos    = body->com_;
		VECTOR3 &vel    = body->velocity_;
    //body->SetAngVel(VECTOR3(0,0,0));        
		VECTOR3 angvel  = body->getAngVel();
    //angvel         += body->getBiasAngVel();

    Quaternionr q0 = body->getQuaternion();
    Quaternionr q1(angvel.x,angvel.y,angvel.z,0);
    
    //get the bias velocity

    Quaternionr q0q1;
    VECTOR3 vq0(q0.x,q0.y,q0.z);
    q0q1.w = -(angvel*vq0);
    VECTOR3 v = VECTOR3::Cross(angvel,vq0) + q0.w*angvel;
    q0q1.x = v.x;
    q0q1.y = v.y;
    q0q1.z = v.z;
 
    Quaternionr q_next = q0 + (timeControl_->GetDeltaT() * 0.5 * (q0q1));
    
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
    pos += body->getBiasVelocity() * timeControl_->GetDeltaT();

    //update the position
    pos += vel * timeControl_->GetDeltaT();

    //update ang velocity
    //angvel = angvel * body->dampening_;

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
