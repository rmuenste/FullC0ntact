

//===================================================
//                     INCLUDES
//===================================================


#include "collider2compound.h"
#include <rigidbody.h>  
#include <compoundbody.h>
#include <sphere.h>  /**compounds only implemented for spheres right now */

#include <contact.h>
#include "collisioninfo.h"
#include "colliderfactory.h"
#include <world.h>
#include <plane.h>
/**resources for sphere-sphere collision*/
#include <mymath.h>

             




namespace i3d {
	/** constructor*/
	Collider2Compound::Collider2Compound(void)
	{

	}
	/** destructor*/
	Collider2Compound::~Collider2Compound(void)
	{

	}


	void Collider2Compound::collide(std::vector<Contact> &vContacts)
	{
		
		//For now only for compounds consisting of spheres


		/**for both compounds:
		//produce a collider for every body of
		//the compound and concatenate the vector
		of contact points */
		CompoundBody *body1 = dynamic_cast<CompoundBody*>(body1_);
		CompoundBody *body0 = dynamic_cast<CompoundBody*>(body0_);


		//looping through all possible pairs (i,j) of components   
		for (int i = 0; i < body1->getNumComponents(); i++)
		{

			RigidBody *p1 = body1->getComponent(i);

			for (int j = 0; j < body0->getNumComponents(); j++)
			{
				RigidBody *p0 = body0->getComponent(j);
				//Check every pair
				ColliderFactory colliderFactory;


				/**________________________________________________________________
				the center of mass of the compounds is needed for computation of relative velocity of the component spheres, the rest functions in the same was as
				sphere-sphere collision */

				//get a collider
				Collider *collider = colliderFactory.ProduceCollider(p0, p1);

				//attach the world object
				collider->setWorld(world_);

				//compute the potential contact points:

				/** alternative to copying from sphere-sphere collision, use respective collision methods for the bodies. The relative velocity of the components, 
				   and some values regarding the contact point are computed differently though.*/
				//	collider->collide(vContacts);

				/**collision detection (review format of variables, colliderfactory etc)_____________________________________*/
				
				const Real contactTolerance = 0.00005;
				//translational velocity of a component sphere i of a given compound a computes as follows:
				// v + (p_i - c)x w
				//where:
				//v: translational velocity of compound a
				//p_i: center of mass of the component sphere i
				// w: angular velocity of compound a
				//c: center of mass of compound a
				
				VECTOR3 &pos1 = p0_->com_;
				
				//tanslational velocity of the sphere i
				VECTOR3 &vel1 = (body0_->velocity_) + VECTOR3::Cross((pos1 - body0->com),body0->angVel);

				Sphere<Real> *pSphere = dynamic_cast<Sphere<Real>* >(p0_->shape_);
				Real rad1 = pSphere->getRadius();

				//same vectors for sphere j
				VECTOR3 &pos2 = p1_->com_;

				VECTOR3 &vel2 = (body1_->velocity_) + VECTOR3::Cross((pos2 - body1->com), body1->angVel);
				

				pSphere = dynamic_cast<Sphere<Real>* >(p1_->shape_);
				Real rad2 = pSphere->getRadius();

				Real dist = std::numeric_limits<Real>::max();

				//calc distance and relative orientation
				//we first need to calculate the relative velocity and
				//the velocity along the normal
				VECTOR3 vn = pos1 - pos2;
				vn.Normalize();

				
				VECTOR3 v12 =  vel1 - vel2;

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
					/**position of the contact point z_ij on body i is given by this formula: 
					 p_i + (R_i -xi_ij/2)*vn
					 where p_i: com of sphere i
					       R_i: radius of sphere i
						   xi_ij = rad1 + rad2 -fabs(pos2-pos1) == -dist
						   vn: normal vector of the collision*/
					//REVIEW:change the value furhter down as well if this computation is correct
					contact.m_vPosition0 =  pos1+(rad1+dist/2)*vn// pos1; //position of the contact point on body0 (warum ist das pos1??)
					contact.m_vPosition1 = pos2 + (rad2 + dist / 2)*vn//pos2; //position of the contact point on body1 (warum ist das pos2??)
					contact.m_pBody0 = body0_;
					contact.m_pBody1 = body1_;
					contact.id0 = contact.m_pBody0->iID_;
					contact.id1 = contact.m_pBody1->iID_;

					//calculate the relative velocity of the contact point
					//for two compounds a and b, it is given by following formula: 
					//contact.vn = v_b - v_a + [w_b x (z_ij - c_b) - w_a x (z_ij - c_a)]
					//where: 
					//v_b: velocity of compound b, v_a likewise
					//w_b: angular velocity of compound a, w_b likewise
					//c_a: center of mass of compound a, c_b likewise
					// z_ij: position of the contact point of the colliding spheres i,j
					//the part in rectangular brackets also represents the angular velocity of the contact point
					contact.vn = >(body1_->velocity - body0_->velocity) + VECTOR3::Cross(body1_->angVel, (pos2 - body1_->com)) - VECTOR3::Cross(body0_->angVel, (pos1 - body_0->com));
					
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
					contact.vn = (body1_->velocity - body0_->velocity) + VECTOR3::Cross(body1_->angVel, (pos2 - body_1->com)) - VECTOR3::Cross(body0_->angVel, (pos1 - body_0->com));
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
					contact.vn = (body1_->velocity - body0_->velocity) + VECTOR3::Cross(body1_->angVel, (pos2 - body_1->com)) - VECTOR3::Cross(body0_->angVel, (pos1 - body_0->com));
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
					contact.vn = (body1_->velocity - body0_->velocity) + VECTOR3::Cross(body1_->angVel, (pos2 - body_1->com)) - VECTOR3::Cross(body0_->angVel, (pos1 - body_0->com));
					contact.m_iState = CollisionInfo::VANISHING_CLOSEPROXIMITY;
					vContacts.push_back(contact);
				}

			}
          //__________________________________________________________________________________________________________________________________________________________

				delete collider;
			}
		}

	}


}