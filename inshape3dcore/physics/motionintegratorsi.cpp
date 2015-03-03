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
		if ((*rIter)->shapeId_ == RigidBody::COMPOUND){
			CompoundBody *body = dynamic_cast<CompoundBody*>((*rIter));
			//update compound parameters

			VECTOR3 &pos = body->com_;
			VECTOR3 &vel = body->velocity_;
			//body->SetAngVel(VECTOR3(0,0,0));        
			VECTOR3 angvel = body->getAngVel();
			//angvel         += body->getBiasAngVel();

			Quaternionr q0 = body->getQuaternion();
			Quaternionr q1(angvel.x, angvel.y, angvel.z, 0);

			

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

			//     std::cout<<"position: "<<pos;
			//     std::cout<<"velocity: "<<vel;    

			//Prevent floating point errors
			if (vel.mag() < CMath<Real>::TOLERANCEZERO)
			{
				vel = VECTOR3(0, 0, 0);
			}

			//body->translateTo(VECTOR3(body->com_+VECTOR3(0,0,-0.01)));

			if (body->isAffectedByGravity())
			{
				VECTOR3 grav = world_->getGravityEffect(body);
				vel += (world_->getGravityEffect(body)) * timeControl_->GetDeltaT();
			}

			//update the position
			pos += vel * timeControl_->GetDeltaT();

			//update ang velocity
			//angvel = angvel * body->dampening_;

			if (angvel.mag() < CMath<Real>::TOLERANCEZERO)
			{
				body->setAngVel(VECTOR3(0, 0, 0));
			}

			body->force_ = VECTOR3(0, 0, 0);
			body->torque_ = VECTOR3(0, 0, 0);
			body->setBiasAngVel(VECTOR3(0, 0, 0));
			body->setBiasVelocity(VECTOR3(0, 0, 0));
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

				//update position 
				//add bias velocity
				pos_i += comp->getBiasVelocity() * timeControl_->GetDeltaT();
				
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

				vel_i = body->velocity_ + VECTOR3::Cross(pos_i - body->com_, body->getAngVel());
				

			}//end for


		}//end if
		else{
			RigidBody *body = *rIter;

			//if(body->shapeId_ == RigidBody::BOUNDARYBOX || !body->isAffectedByGravity())
			//	continue;

			VECTOR3 &pos = body->com_;
			VECTOR3 &vel = body->velocity_;
			//body->SetAngVel(VECTOR3(0,0,0));        
			VECTOR3 angvel = body->getAngVel();
			//angvel         += body->getBiasAngVel();

			Quaternionr q0 = body->getQuaternion();
			Quaternionr q1(angvel.x, angvel.y, angvel.z, 0);

			//get the bias velocity

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

			//     std::cout<<"position: "<<pos;
			//     std::cout<<"velocity: "<<vel;    

			//Prevent floating point errors
			if (vel.mag() < CMath<Real>::TOLERANCEZERO)
			{
				vel = VECTOR3(0, 0, 0);
			}

			//body->translateTo(VECTOR3(body->com_+VECTOR3(0,0,-0.01)));

			if (body->isAffectedByGravity())
			{
				VECTOR3 grav = world_->getGravityEffect(body);
				vel += (world_->getGravityEffect(body)) * timeControl_->GetDeltaT();
			}

			//add bias velocity
			pos += body->getBiasVelocity() * timeControl_->GetDeltaT();

			//update the position
			pos += vel * timeControl_->GetDeltaT();

			//update ang velocity
			//angvel = angvel * body->dampening_;

			if (angvel.mag() < CMath<Real>::TOLERANCEZERO)
			{
				body->setAngVel(VECTOR3(0, 0, 0));
			}

			body->force_ = VECTOR3(0, 0, 0);
			body->torque_ = VECTOR3(0, 0, 0);
			body->setBiasAngVel(VECTOR3(0, 0, 0));
			body->setBiasVelocity(VECTOR3(0, 0, 0));
			count++;
		}//end else
  }//end for
}

}
