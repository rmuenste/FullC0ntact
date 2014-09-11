#include "colliderspherecompound.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <colliderfactory.h>
#include <world.h>

namespace i3d {

ColliderSphereCompound::ColliderSphereCompound(void)
{
}

ColliderSphereCompound::~ColliderSphereCompound(void)
{
}

void ColliderSphereCompound::collide(std::vector<Contact> &vContacts)
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  CompoundBody *body1 = dynamic_cast<CompoundBody*>(body1_);
  RigidBody       *p0 = body0_;

  for(int i=0;i<body1->getNumComponents();i++)
  {
    
    RigidBody *p1 = body1->getComponent(i);

    //Check every pair
    ColliderFactory colliderFactory;

    //get a collider
    Collider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->setWorld(world_);

    //compute the potential contact points (extended sphere-sphere)
    //collider->collide(vContacts);

	/**________________sphere-sphere contact detection______________________________________________________________________________________
	_________________________________________________________________________________________________________________________________________*/
	//p0 is the sphere
	const Real contactTolerance = 0.00005;
	VECTOR3 &vel1 = p0->velocity_;
	VECTOR3 &pos1 = p0->com_;

	Sphere<Real> *pSphere = dynamic_cast<Sphere<Real>* >(p0->shape_);
	Real rad1 = pSphere->getRadius();


	VECTOR3 &pos2 = p1->com_;
	//compound body component has different relative velocity: 
	VECTOR3 &vel2 = (body1_->velocity_) + VECTOR3::Cross((pos2 - body1_->com_), body1_->getAngVel());
	

	pSphere = dynamic_cast<Sphere<Real>* >(p1->shape_);
	Real rad2 = pSphere->getRadius();

	Real dist = std::numeric_limits<Real>::max();

	//calc distance and relative orientation
	//we first need to calculate the relative velocity and
	//the velocity along the normal
	VECTOR3 vn = pos1 - pos2;
	vn.Normalize();

	//calculate the relative velocity
	VECTOR3 v12 = vel1 - vel2;

	//calculate the velocity along the normal
	Real velalongnormal = vn * v12;

	//calculate the distance
	dist = (pos2 - pos1).mag() - rad1 - rad2;
	Real dist1 = fabs(vn*vel1);
	Real dist2 = fabs(vn*vel2);
	Real distpertime = (dist1 + dist2)*world_->timeControl_->GetDeltaT();

	if (velalongnormal < -0.005 && distpertime >= dist)
		//if(relativeNormalVelocity < -0.005)
	{
		Contact contact;
		contact.m_dDistance = dist;
		contact.m_vNormal = vn;
		contact.m_vPosition0 = pos1;
		contact.m_vPosition1 = pos2;
		contact.m_pBody0 = body0_;
		contact.m_pBody1 = body1_;
		contact.id0 = contact.m_pBody0->iID_;
		contact.id1 = contact.m_pBody1->iID_;
		//vn needs to be changed; uses the same formula as compound-compound collision
		contact.vn = velalongnormal;
		//calculate the relative velocity of the contact point
		//for two compounds a and b, it is given by following formula: 
		//contact.vn = v_b - v_a + [w_b x (z_ij - c_b) - w_a x (z_ij - c_a)]
		//where: 
		//v_b: velocity of compound b, v_a likewise
		//w_b: angular velocity of compound a, w_b likewise
		//c_a: center of mass of compound a, c_b likewise
		// z_ij: position of the contact point of the colliding spheres i,j
		//the part in rectangular brackets also represents the angular velocity of the contact point
		//contact.vn =(body1_->velocity_ - p0->velocity_) + VECTOR3::Cross(body1_->getAngVel(), (pos2 - body1_->com_)) - VECTOR3::Cross(p0->getAngVel(), (pos1 - p0->com_));
		
		contact.m_dPenetrationDepth = std::min(0.0, dist);
		contact.m_iState = CollisionInfo::TOUCHING;
		//std::cout<<"Pre-contact normal velocity: "<<velalongnormal<<" colliding contact"<<std::endl;
		//std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetAngVel();
		//std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetAngVel();
		//std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
		//std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
		vContacts.push_back(contact);
	}
	else if (velalongnormal < 0.00001 && dist < contactTolerance)
	{
		Contact contact;
		contact.m_dDistance = dist;
		contact.m_vNormal = vn;
		contact.m_vPosition0 = pos1;
		contact.m_vPosition1 = pos2;
		contact.m_pBody0 = body0_;
		contact.m_pBody1 = body1_;
		contact.id0 = contact.m_pBody0->iID_;
		contact.id1 = contact.m_pBody1->iID_;
		//contact.vn = (body1_->velocity_ - p0->velocity_) + VECTOR3::Cross(body1_->getAngVel(), (pos2 - body1_->com_)) - VECTOR3::Cross(p0->getAngVel(), (pos1 - p0->com_));
		contact.vn = velalongnormal;
		contact.m_iState = CollisionInfo::TOUCHING;
		vContacts.push_back(contact);
	}
	else if (dist < 0.1*rad1)
	{
		Contact contact;
		contact.m_dDistance = dist;
		contact.m_vNormal = vn;
		contact.m_vPosition0 = pos1;
		contact.m_vPosition1 = pos2;
		contact.m_pBody0 = body0_;
		contact.m_pBody1 = body1_;
		contact.id0 = contact.m_pBody0->iID_;
		contact.id1 = contact.m_pBody1->iID_;
		contact.vn = velalongnormal;
		//contact.vn = (body1_->velocity_ - p0->velocity_) + VECTOR3::Cross(body1_->getAngVel(), (pos2 - body1_->com_)) - VECTOR3::Cross(p0->getAngVel(), (pos1 - p0->com_));
		contact.m_iState = CollisionInfo::TOUCHING;
		vContacts.push_back(contact);
	}
	else
	{
		return;
		Contact contact;
		contact.m_dDistance = dist;
		contact.m_vNormal = vn;
		contact.m_vPosition0 = pos1;
		contact.m_vPosition1 = pos2;
		contact.m_pBody0 = body0_;
		contact.m_pBody1 = body1_;
		contact.id0 = contact.m_pBody0->iID_;
		contact.id1 = contact.m_pBody1->iID_;
		contact.vn = velalongnormal;
		//contact.vn = (body1_->velocity_ - p0->velocity_) + VECTOR3::Cross(body1_->getAngVel(), (pos2 - body1_->com_)) - VECTOR3::Cross(p0->getAngVel(), (pos1 - p0->com_));
		contact.m_iState = CollisionInfo::VANISHING_CLOSEPROXIMITY;
		vContacts.push_back(contact);
	}
	     delete collider;
  }


   
  }

}


