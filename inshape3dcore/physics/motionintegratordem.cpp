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

	int count = 0;
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
      w2l.TransposeMatrix();
      VECTOR3 angvel_l = w2l * body->getAngVel();

#ifdef DEBUG						
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
			VECTOR3 AngAcc = mInvInertiaTensor * torque;

      MATRIX3X3 InvInertiaTensor = body->invInertiaTensor_;
      VECTOR3 AngAcc_world = mInvInertiaTensor * body->torque_local_;
			
			//calculate the first derivative of the linear acceleration
			VECTOR3 LinDer = (1.0 / dt) * (LinAcc - body->oldVel_);
			
			//calculate the first derivative of the angular acceleration
			VECTOR3 AngDer = (1.0 /dt) * (AngAcc - body->oldAngVel_);

      //calculate the first derivative of the angular acceleration
      VECTOR3 oldAngVel_world = body->getQuaternion().GetMatrix() * body->oldAngVel_;
      VECTOR3 angvel_world = body->getQuaternion().GetMatrix() * body->getAngVel();
      VECTOR3 AngDer_world = (1.0 / dt) * (AngAcc_world - oldAngVel_world);

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
           
			VECTOR3 eulerAngles = q0.convertToEuler();
      VECTOR3 eulerAngles2 = q0.convertToEuler();
			eulerAngles = eulerAngles + angvel * dt + 0.5 * AngAcc * dt*dt + AngDer * (1.0/6.0) * dt*dt*dt;
      eulerAngles2 = eulerAngles2 + angvel_world * dt + 0.5 * AngAcc_world * dt*dt + AngDer_world * (1.0 / 6.0) * dt*dt*dt;

			Quaternionr q_next;
      q_next.CreateFromEulerAngles(eulerAngles2.y, eulerAngles2.z, eulerAngles2.x);
			q_next.Normalize();
			body->setQuaternion(q_next);
			body->setTransformationMatrix(q_next.GetMatrix());

			//update ang velocity
			angvel += AngAcc *dt + 0.5 * AngDer *dt * dt;
      angvel_world += AngAcc_world *dt + 0.5 * AngDer_world *dt * dt;
#ifdef DEBUG						
      std::cout<<"AngAcc: "<<AngAcc<<" AngDer: "<<AngDer<<std::endl;
#endif
			
			/*dampening the angular velocity, so that particles may come to rest in ~100 steps in simulaton */
      angvel_world *= world_->airFriction_;
      VECTOR3 vtrans = w2l * angvel_world;
      //vtrans *= 0.98;

      body->setAngVel(vtrans);

			//update Velocity
	    vel += LinAcc * dt + LinDer * dt* dt;

#ifdef DEBUG						
			std::cout << "velocity after: " << vel << std::endl;
      std::cout << "position: " << pos << std::endl;
      std::cout << "angular velocity: " << body->getAngVel();
      std::cout << "angular velocity_world: " << angvel_world;
      std::cout << "angular velocity_local: " << w2l * angvel_world;
      std::cout<<"orientation: "<<eulerAngles;
#endif

			//store linear and angular acceleration of current time step in oldVel_ and oldAngVel_
			body->oldVel_ = LinAcc;
      body->oldAngVel_ = AngAcc_world;

			//
			if (angvel.mag() < CMath<Real>::TOLERANCEZERO)
			{
				body->setAngVel(VECTOR3(0, 0, 0));
			}

			body->force_ = VECTOR3(0, 0, 0);
			body->torque_ = VECTOR3(0, 0, 0);

      body->force_local_ = VECTOR3(0, 0, 0);
      body->torque_local_ = VECTOR3(0, 0, 0);
		
			count++;

			//now update component parameters 

			std::vector<RigidBody*> &bRigidBodies = body->rigidBodies_;
			std::vector<RigidBody*>::iterator rIter2;
			
			//update translational velocity and orientation of the components
			for (rIter2 = bRigidBodies.begin(); rIter2 != bRigidBodies.end(); rIter2++)
			{
				RigidBody *comp = *rIter2;
				VECTOR3 &vel_i = comp->velocity_;
				
				VECTOR3 pos_i = comp->getTransformedPosition();
			
				//update the position
				pos_i += vel * timeControl_->GetDeltaT();

				//angular velocity for all components is the same as that of the compound, the individual translational
				comp->setQuaternion(q_next);
				comp->setTransformationMatrix(q_next.GetMatrix());
				comp->transform_.setOrigin(body->com_);
				comp->transform_.setMatrix(body->getTransformationMatrix());

				//velocity of component i is obtained by  the formula 
				// velocity_(i) = velocity_ + (com_(i)-com_) x angVel_
				//with velocity_ and angVel denoting trans. vel and ang. Vel of the compound

        vel_i = body->velocity_ + VECTOR3::Cross(pos_i - body->com_, angvel_world);
				

			}//end for


		}//end if
		else{
			RigidBody *body = *rIter;

			//if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
			//	continue;

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

			//update position
			pos += vel * dt;

			body->force_ = VECTOR3(0, 0, 0);
			body->torque_ = VECTOR3(0, 0, 0);

			count++;
			count++;
		}//end else
  }//end for
}

}
