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

CMotionIntegratorSI::CMotionIntegratorSI(CWorld* pDomain)
{
	m_pWorld = pDomain;
	m_pTimeControl = pDomain->m_pTimeControl;
}

CMotionIntegratorSI::~CMotionIntegratorSI(void)
{
}

void CMotionIntegratorSI::UpdatePosition()
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
    //body->SetAngVel(VECTOR3(0,0,0));        
		VECTOR3 angvel  = body->GetAngVel();
    angvel         += body->GetBiasAngVel();

    CQuaternionr q0 = body->GetQuaternion();
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
    body->SetQuaternion(q_next);
    body->SetTransformationMatrix(q_next.GetMatrix());
    
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
    pos += body->GetBiasVelocity() * m_pTimeControl->GetDeltaT();

    //update the position
    pos += vel * m_pTimeControl->GetDeltaT();

    //update ang velocity
    angvel = angvel * body->m_dDampening;

    if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      body->SetAngVel(VECTOR3(0,0,0));
    }

    body->m_vForce=VECTOR3(0,0,0);
    body->m_vTorque=VECTOR3(0,0,0);
    body->SetBiasAngVel(VECTOR3(0,0,0));
    body->SetBiasVelocity(VECTOR3(0,0,0));
    count++;
  }//end for
}

}
