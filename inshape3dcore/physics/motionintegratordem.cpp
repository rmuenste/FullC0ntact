#include "motionintegratordem.h"
#include <rigidbody.h>
#include <compoundbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>


namespace i3d {

MotionIntegratorDEM::MotionIntegratorDEM(void)
{
  world_ = NULL;
  timeControl_ = NULL;
}

MotionIntegratorDEM::MotionIntegratorDEM(World* pDomain)
{
  world_ = pDomain;
  timeControl_ = pDomain->timeControl_;
}

MotionIntegratorDEM::~MotionIntegratorDEM(void)
{

}

void MotionIntegratorDEM::updatePosition()
{

  std::vector<RigidBody*> &vRigidBodies = world_->rigidBodies_;
  std::vector<RigidBody*>::iterator rIter;

  Real dt = timeControl_->GetDeltaT();
  for(rIter=vRigidBodies.begin();rIter!=vRigidBodies.end();rIter++)
  {
    if ((*rIter)->shapeId_ == RigidBody::COMPOUND){
      CompoundBody *body = dynamic_cast<CompoundBody*>((*rIter));
      //update compound parameters

      if (!body->isAffectedByGravity())
        continue;

      VECTOR3 &pos = body->com_;
      VECTOR3 &vel = body->velocity_;
      VECTOR3 &force = body->force_;//ComponentForce_;
      VECTOR3 &torque = body->torque_;//ComponentTorque_;
      VECTOR3 angvel = body->getAngVel();

      MATRIX3X3 w2l = body->getQuaternion().GetMatrix();
      MATRIX3X3 l2w = body->getQuaternion().GetMatrix();
      w2l.TransposeMatrix();


#ifdef DEBUG
      std::cout << "body ID: " << body->iID_;
      std::cout << " sim_time: " << world_->timeControl_->GetTime() << " t_l: " << body->torque_ << " omega: " << angvel;

      std::cout << "angular velocity: " << body->getAngVel();
      std::cout << "matrix l2w: " << l2w << std::endl;

      std::cout << "matrix: " << w2l << std::endl;
#endif

      //calculate linear acceleration
      VECTOR3 LinAcc = force;
      if (body->isAffectedByGravity())
      {
        VECTOR3 grav = world_->getGravityEffect(body);
        LinAcc += (world_->getGravityEffect(body));
      }

      //calculate angular acceleration
      MATRIX3X3 mInvInertiaTensor = body->getWorldTransformedInvTensor();

      //transform torque to local space and multiply
      VECTOR3 AngAcc = body->invInertiaTensor_ * (w2l * torque);

      //calculate the first derivative of the linear acceleration
      VECTOR3 LinDer = (1.0 / dt) * (LinAcc - body->oldVel_);

      //calculate the first derivative of the angular acceleration
      VECTOR3 AngDer = (1.0 /dt) * (AngAcc - body->oldAngAcc_);

#ifdef DEBUG
      std::cout<< "sim_time: " << world_->timeControl_->GetTime() << " kN: " <<body->force_.z<<std::endl;
#endif

      //Prevent floating point errors
      if (vel.mag() < CMath<Real>::TOLERANCEZERO)
      {
        vel = VECTOR3(0, 0, 0);
      }

      //update position
      pos += vel * dt + 0.5 * LinAcc * dt * dt
        + (1.0 / 6.0) * LinDer * dt * dt * dt;

      //update orientation
      Quaternionr q0 = body->getQuaternion();

      VECTOR3 rotChange = angvel * dt + 0.5 * AngAcc * dt*dt + AngDer * (1.0 / 6.0) * dt*dt*dt;

      Quaternionr update(rotChange.x, rotChange.y, rotChange.z, 0.0);
      update = update.mult(q0);
      Quaternionr q_next2 = q0 + (0.5 * (update));
      q_next2.Normalize();

      body->setQuaternion(q_next2);
      body->setTransformationMatrix(q_next2.GetMatrix());
      //update ang velocity
      angvel += AngAcc *dt + 0.5 * AngDer *dt * dt;
      angvel *= world_->airFriction_;
#ifdef DEBUG
      std::cout<<"AngAcc: "<<AngAcc<<" AngDer: "<<AngDer<<std::endl;
#endif

      VECTOR3 angvel_w = l2w * angvel;
      /*dampening the angular velocity, so that particles may come to rest in ~100 steps in simulaton */
      body->setAngVel(angvel);
      //std::cout << "t: " << world_->timeControl_->GetTime() << " angvel: " << body->getAngVel().y << std::endl;
//      std::cout << "time: " << world_->timeControl_->GetTime() << " angvel: " << body->getAngVel();
//      std::cout << "time: " << world_->timeControl_->GetTime() << " position: " << body->com_.x << std::endl;
//      std::cout << "t: " << world_->timeControl_->GetTime() << " velocity: " << body->velocity_.x << std::endl;
      //std::cout << "t: " << world_->timeControl_->GetTime() << " position: " << body->com_.z << std::endl;

      //update Velocity
      vel += LinAcc * dt + LinDer * dt* dt;

#ifdef DEBUG

      std::cout << "angular velocity: " << body->getAngVel();
      std::cout << "orientation: " << q_next.convertToEuler();
      std::cout << "orientation2: " << q_next2.convertToEuler();
      std::cout << "Matrix: " << std::endl; std::cout << q_next.GetMatrix() << std::endl;


      std::cout << "velocity after: " << vel << std::endl;
      std::cout << "position: " << pos << std::endl;
      std::cout << "angular velocity: " << body->getAngVel();
      std::cout << "angular velocity_world: " << angvel_world;
      std::cout << "angular velocity_local: " << w2l * angvel_world;
      std::cout<<"orientation: "<<eulerAngles;
#endif

      //store linear and angular acceleration of current time step in oldVel_ and oldAngVel_
      body->oldVel_ = LinAcc;
      body->oldAngAcc_ = AngAcc;

      //
      if (angvel.mag() < CMath<Real>::TOLERANCEZERO)
      {
        body->setAngVel(VECTOR3(0, 0, 0));
      }

      body->force_ = VECTOR3(0, 0, 0);
      body->torque_ = VECTOR3(0, 0, 0);

      body->force_local_ = VECTOR3(0, 0, 0);
      body->torque_local_ = VECTOR3(0, 0, 0);

      //now update component parameters

      std::vector<RigidBody*> &bRigidBodies = body->rigidBodies_;
      std::vector<RigidBody*>::iterator rIter2;

#ifdef DEBUG

      //std::cout << "angvel: " << body->getAngVel().y << std::endl;
      std::cout << "vel_x: " << vel.x << std::endl;
      //std::cout << "pos_z: " << pos.z << std::endl;
#endif

      //update translational velocity and orientation of the components
      for (rIter2 = bRigidBodies.begin(); rIter2 != bRigidBodies.end(); rIter2++)
      {
        RigidBody *comp = *rIter2;
        VECTOR3 &vel_i = comp->velocity_;

        VECTOR3 pos_i = comp->getTransformedPosition();

        //update the position
        pos_i += vel * timeControl_->GetDeltaT();

        //angular velocity for all components is the same as that of the compound, the individual translational
        comp->setQuaternion(q_next2);
        comp->setTransformationMatrix(q_next2.GetMatrix());
        comp->transform_.setOrigin(body->com_);
        comp->transform_.setMatrix(body->getTransformationMatrix());

        //velocity of component i is obtained by  the formula
        // velocity_(i) = velocity_ + (com_(i)-com_) x angVel_
        //with velocity_ and angVel denoting trans. vel and ang. Vel of the compound

        vel_i = body->velocity_ + VECTOR3::Cross(pos_i - body->com_, angvel_w);


      }//end for

    }//end if
    else{
      RigidBody *body = *rIter;

      //if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
      //  continue;

      VECTOR3 &pos = body->com_;
      VECTOR3 &vel = body->velocity_;

      VECTOR3 &force = body->force_;
      VECTOR3 &torque = body->torque_;
      VECTOR3 angvel = body->getAngVel();

      //calculate linear acceleration
      VECTOR3 LinAcc = body->invMass_* force;;

      if (body->isAffectedByGravity())
      {
        VECTOR3 grav = world_->getGravityEffect(body);
        LinAcc += (world_->getGravityEffect(body));
      }

      //calculate angular acceleration
      MATRIX3X3 mInvInertiaTensor = body->getWorldTransformedInvTensor();
      VECTOR3 AngAcc = mInvInertiaTensor * torque;

      //calculate the first derivative of the linear acceleration
      VECTOR3 LinDer = (1.0 / timeControl_->GetDeltaT()) * (LinAcc - body->oldVel_);

      //calculate the first derivative of the angular acceleration
      VECTOR3 AngDer = (1.0 / timeControl_->GetDeltaT()) * (AngAcc - body->oldAngVel_);

      Quaternionr q0 = body->getQuaternion();
      Quaternionr q0q1;
      VECTOR3 vq0(q0.x, q0.y, q0.z);
      q0q1.w = -(angvel*vq0);
      VECTOR3 v = VECTOR3::Cross(angvel, vq0) + q0.w*angvel;
      q0q1.x = v.x;
      q0q1.y = v.y;
      q0q1.z = v.z;

      Quaternionr q_next = q0 + (timeControl_->GetDeltaT() * 0.5 * (q0q1));

      q_next.Normalize();

      //update orientation
      body->setQuaternion(q_next);
      body->setTransformationMatrix(q_next.GetMatrix());


      //update position
      pos += vel * dt;

      body->force_ = VECTOR3(0, 0, 0);
      body->torque_ = VECTOR3(0, 0, 0);

    }//end else
  }//end for
}

}
