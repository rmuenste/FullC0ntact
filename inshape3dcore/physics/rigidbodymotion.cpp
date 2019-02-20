#include "rigidbodymotion.h"
#include <rigidbody.h>
#include <world.h>
#include <iostream>
#include <timecontrol.h>
#include <termcolor.hpp>

namespace i3d {

  RigidBodyMotion::RigidBodyMotion(void)
  {
    world_ = NULL;
    timeControl_ = NULL;
  }

  RigidBodyMotion::~RigidBodyMotion(void)
  {

  }

  void RigidBodyMotion::applyExternalForce(RigidBody *body, const Vec3 &force, const Vec3 &torque)
  {

    // integrate the force to get an acceleration
    Vec3 velUpdate = world_->timeControl_->GetDeltaT() * body->invMass_* force;

    // integrate the torque to get angular acceleration
    Vec3 angUpdate =  body->getWorldTransformedInvTensor() * world_->timeControl_->GetDeltaT() *
                      torque;

    body->velocity_ += velUpdate;

    body->setAngVel(body->getAngVel() + angUpdate);

  }

  void RigidBodyMotion::updateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque)
  {

    int count=0;

    for (auto &body : world_->rigidBodies_)
    {

      if(body->shapeId_ == RigidBody::BOUNDARYBOX)
        continue;

      //Vec3 meanForce =  Real(0.5448) * (body->force_ + force[count]);
      Vec3 meanForce =  force[count];

      // integrate the force to get an acceleration
      Vec3 velUpdate = world_->timeControl_->GetDeltaT() * body->invMass_*(meanForce);

      // integrate the torque to get angular acceleration
      Vec3 angUpdate =  body->getWorldTransformedInvTensor() * (Real(0.5) * 
          world_->timeControl_->GetDeltaT() * 
          (body->torque_ + torque[count]));

      body->velocity_ += velUpdate;

      body->setAngVel(body->getAngVel() + angUpdate);

      if(world_->parInfo_.getId()==1)
      {
        std::cout << "====================" << std::endl;
        std::cout << "      Up-Calc       " << std::endl;
        std::cout << "====================" << std::endl;

        std::cout << termcolor::bold << termcolor::green << world_->parInfo_.getId() <<  
                    " > body force: " << body->force_ << termcolor::reset;

        std::cout << termcolor::bold << termcolor::blue << world_->parInfo_.getId() <<  
                    " > force[count]: " <<  force[count] << termcolor::reset;

        std::cout << termcolor::bold << termcolor::red << world_->parInfo_.getId() <<  
                    " > meanForce: " << meanForce << termcolor::reset;
        
        std::cout << termcolor::bold << termcolor::green << world_->parInfo_.getId() <<  
                    " > angUpdate: " << angUpdate << termcolor::reset;

        std::cout << termcolor::reset << std::endl;
      }

      body->force_ = force[count];
      body->torque_ = torque[count];

    }
  }

  void RigidBodyMotion::updateForces(std::vector<VECTOR3>& force, std::vector<VECTOR3>& torque, Real scale)
  {

    double densityLiquid = world_->densityMedium_;
    std::exit(EXIT_FAILURE);

    int count=0;

    for (auto &body : world_->rigidBodies_)
    {

      if(body->shapeId_ == RigidBody::BOUNDARYBOX)
        continue;

      VECTOR3 meanForce =  Real(0.5) * (body->force_ + force[count]);

      // integrate the force to get an acceleration
      VECTOR3 velUpdate = world_->timeControl_->GetDeltaT() * body->invMass_*(meanForce);

      body->velocity_ += velUpdate;

    }
  }

  void RigidBodyMotion::updatePosition()
  {

    int count = 0;
    for (auto &body : world_->rigidBodies_)
    {

      if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
        continue;

      Vec3 &pos = body->com_;
      Vec3 &vel = body->velocity_;
      Vec3 &angle = body->angle_;
      Vec3 angvel = body->getAngVel();

      // Save the values from the last time step
      Vec3 oldAngVel = body->getAngVel();
      Vec3 oldVelocity = body->velocity_;

      Quaternionr q0 = body->getQuaternion();
      Quaternionr q1(angvel.x,angvel.y,angvel.z,0);
      Quaternionr q0q1;
      Vec3 vq0(q0.x,q0.y,q0.z);
      q0q1.w = -(angvel*vq0);
      Vec3 v = VECTOR3::Cross(angvel,vq0) + q0.w*angvel;
      q0q1.x = v.x;
      q0q1.y = v.y;
      q0q1.z = v.z;

      Quaternionr q_next = q0 + (timeControl_->GetDeltaT() * Real(0.5) * (q0q1));

      q_next.Normalize();

      //update orientation    
      body->setQuaternion(q_next);
      body->setTransformationMatrix(q_next.GetMatrix());

      //std::cout<<"Position before: "<<pos<<std::endl;    
      //update velocity
      if(body->isAffectedByGravity())
      { // + massDiff * m_pWorld->GetGravity());
        //std::cout<<"velocity_before: "<<vel<<std::endl;
        if(world_->parInfo_.getId()==1)
        {

          //std::cout<<"mass: "<<body->invMass_<<std::endl;
          std::cout << "deltaT: " << timeControl_->GetDeltaT() <<std::endl;
          std::cout << "q0q1: " << q0q1 <<std::endl;
          std::cout << "q_next: " << q_next <<std::endl;
          //std::cout<<"Gravity part"<<world_->getGravityEffect(body) * timeControl_->GetDeltaT()<<std::endl;
        }
        vel += ((body->forceResting_ * body->invMass_) + world_->getGravityEffect(body)) * timeControl_->GetDeltaT();
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
      angvel = angvel;

      body->setAngVel(angvel * 1.0);//0.98;

#ifdef OPTIC_FORCES
      // calculate the laser contribution
      if(world_->parInfo_.getId()==1)
      {
        std::cout << "====================" << std::endl;
        std::cout << "    Pos-Vel-up      " << std::endl;
        std::cout << "====================" << std::endl;

        std::cout << termcolor::bold << termcolor::green << world_->parInfo_.getId() <<  
                    " > object pos[mm]: " << pos << termcolor::reset;

        std::cout << termcolor::bold << termcolor::blue << world_->parInfo_.getId() <<  
                    " > velocity[mm/s]: " <<  vel << termcolor::reset;

        std::cout << termcolor::bold << termcolor::red << world_->parInfo_.getId() <<  
                    " > Angular Vel[radians/s]: " << angvel << termcolor::reset;

        std::cout << termcolor::bold << termcolor::green << world_->parInfo_.getId() <<  
                    " > Gravity: " << ((body->forceResting_ * body->invMass_) + world_->getGravityEffect(body)) * timeControl_->GetDeltaT() << termcolor::reset;

        std::cout << termcolor::reset << std::endl;
      }

      if(world_->parInfo_.getId()==1)
      {

        std::ofstream out("observables.log", std::ios::out | std::ios::app);
        out << world_->timeControl_->getTime() << " " << 
        body->force_.x << " "  <<
        body->force_.y << " "  <<
        body->force_.z << " "  <<
        body->laserForce_.x << " "  <<
        body->laserForce_.y << " "  <<
        body->laserForce_.z << " "  <<
        body->torque_.x << " "  <<
        body->torque_.y << " "  <<
        body->torque_.z << " "  <<
        body->laserTorque_.x << " "  <<
        body->laserTorque_.y << " "  <<
        body->laserTorque_.z << " "  <<
        vel.x << " "  <<
        vel.y << " "  <<
        vel.z << " "  <<
        angvel.x << " "  <<
        angvel.y << " "  <<
        angvel.z << " "  <<
        "\n"; 
        out.close();

      }
#endif

//      if(angvel.mag() < CMath<Real>::TOLERANCEZERO)
//      {
//        body->setAngVel(VECTOR3(0,0,0));
//      }

      body->oldForce_ = body->force_;
      body->oldTorque_ = body->torque_;

      // reset the resting force to 0
      body->forceResting_=VECTOR3(0,0,0);
      body->force_=VECTOR3(0,0,0);
      body->torque_=VECTOR3(0,0,0);    
      count++;
    }//end for

    world_->timeControl_->step(world_->timeControl_->GetDeltaT());

  }

  RigidBodyMotion::RigidBodyMotion(World* domain)
  {
    world_ = domain;
    timeControl_ = domain->timeControl_;
  }

}
