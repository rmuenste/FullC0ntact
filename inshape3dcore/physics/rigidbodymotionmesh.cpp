#include "rigidbodymotionmesh.h"
#include <vector3.h>
#include <vector>
#include <rigidbody.h>
#include <contact.h>


CEulerMotion::CEulerMotion(void)
{

}

CEulerMotion::CEulerMotion(CWorld* pDomain) : CRigidBodyMotion(pDomain)
{

}

CEulerMotion::~CEulerMotion(void)
{

}

void CEulerMotion::UpdateForces(VECTOR3 *force, VECTOR3 *torque)
{
  std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
  std::vector<CRigidBody*>::iterator rIter;
  double densityLiquid = 1.0;
  int count;

  for(rIter=vRigidBodies.begin(),count = 0;rIter!=vRigidBodies.end();rIter++,count++)
  {

    CRigidBody *body = *rIter;

    if(body->m_iShape == CRigidBody::BOUNDARYBOX)
      continue;
    
    VECTOR3 meanForce =  0.5 * (body->m_vForce + force[count]);
    Real massDiff = body->m_dVolume * (body->m_dDensity - densityLiquid);
    VECTOR3 velUpdate = m_pWorld->m_pTimeControl->GetDeltaT() * 
                                   body->m_dInvMass*(meanForce);
                                   
    VECTOR3 angUpdate =  body->GetWorldTransformedInvTensor() * (0.5 * m_pWorld->m_pTimeControl->GetDeltaT() * 
                                   (body->m_vTorque + torque[count]));
    
   body->m_vVelocity += velUpdate;
   
   body->SetAngVel(body->GetAngVel() + angUpdate);
                                   
  }
}


void CEulerMotion::UpdatePosition()
{
  std::vector<CRigidBody*> &vRigidBodies = m_pWorld->m_vRigidBodies;
  std::vector<CRigidBody*>::iterator rIter;

  int count = 0;

  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {

    CRigidBody *body = *rIter;

    if(body->m_iShape == CRigidBody::BOUNDARYBOX)
      continue;

    VECTOR3 &pos    = body->m_vCOM;
    VECTOR3 &vel    = body->m_vVelocity;
    VECTOR3 angvel  = body->GetAngVel();
    VECTOR3 &angle  = body->m_vAngle;
    MATRIX3X3 &matTransform = body->m_matTransform;
    MATRIX3X3 matAngUpdate = MATRIX3X3::GetSkewMatrix(angvel);

    //update the position
    pos += vel * m_pTimeControl->GetDeltaT();
    //std::cout<<pos<<std::endl;

    //update orientation
    matTransform += (matAngUpdate *matTransform) * m_pTimeControl->GetDeltaT();
    //std::cout<<matTransform<<std::endl;

    //update velocity
    if(body->IsAffectedByGravity())
      vel += ((body->m_vForceResting*body->m_dInvMass) + m_pWorld->m_vGrav) * m_pTimeControl->GetDeltaT();

    {
      //vel.x = vel.x * 0.98;
      //vel.y = vel.y * 0.98;
    }

    //Prevent floating point errors
    if(vel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      vel=VECTOR3(0,0,0);
    }
    //std::cout<<body->m_vForceResting * body->m_dInvMass<<" compared to gravity : "<<m_pWorld->m_vGrav<<" "<<pos.z<<" "<<vel.z<<std::endl;

    //update ang velocity
    angvel = angvel * body->m_dDampening;

      body->SetAngVel(angvel * 1.0);//0.98;
    
    if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
    {
      body->SetAngVel(VECTOR3(0,0,0));
    }
    //std::cout<<"angvel "<<angvel<<std::endl;
    //std::cout<<"world angvel "<<body->GetWorldAngVel()<<std::endl;

    //reset the resting force to 0
    body->m_vForceResting=VECTOR3(0,0,0);

    count++;
  }//end for
}
