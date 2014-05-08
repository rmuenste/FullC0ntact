#include "rigidbodymotion.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

RigidBodyMotion::RigidBodyMotion(void)
{
	world_ = NULL;
	timeControl_ = NULL;
}

RigidBodyMotion::~RigidBodyMotion(void)
{
}

void RigidBodyMotion::updateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque)
{
  double densityLiquid = world_->densityMedium_;
  int count;

  for (auto &body : world_->rigidBodies_)
  {

    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;
    
    VECTOR3 meanForce =  0.5 * (body->force_ + force[count]);

    // compute the mass difference of fluid and solid
    Real massDiff = body->volume_ * (body->density_ - densityLiquid);

    // integrate the force to get an acceleration
    VECTOR3 velUpdate = world_->timeControl_->GetDeltaT() * body->invMass_*(meanForce);

    // integrate the torque to get angular acceleration
    VECTOR3 angUpdate =  body->getWorldTransformedInvTensor() * (0.5 * world_->timeControl_->GetDeltaT() * 
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

void RigidBodyMotion::updatePosition()
{

  int count = 0;
  for (auto &body : world_->rigidBodies_)
  {

    if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
      continue;

    VECTOR3 &pos    = body->com_;
    VECTOR3 &vel    = body->velocity_;
    //body->SetAngVel(VECTOR3(0,0,0));        
    VECTOR3 angvel  = body->getAngVel();
    VECTOR3 &angle  = body->angle_;
    Quaternionr q0 = body->getQuaternion();
    Quaternionr q1(angvel.x,angvel.y,angvel.z,0);
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
        
    //std::cout<<"Position before: "<<pos<<std::endl;    
    //update velocity
    if(body->isAffectedByGravity())
    { // + massDiff * m_pWorld->GetGravity());
      //std::cout<<"velocity_before: "<<vel<<std::endl;
      vel += ((body->forceResting_ * body->invMass_) + world_->getGravityEffect(body)) * timeControl_->GetDeltaT();
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
    pos += vel * timeControl_->GetDeltaT();

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

RigidBodyMotion::RigidBodyMotion(World* domain)
{
  world_ = domain;
  timeControl_ = domain->timeControl_;
}

}
