

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
    CompoundBody *cbody1_ = dynamic_cast<CompoundBody*>(body1_);
    CompoundBody *cbody0_ = dynamic_cast<CompoundBody*>(body0_);


    for (int i = 0; i < cbody0_->getNumComponents(); i++)
    {
      RigidBody *p0 = cbody0_->getComponent(i);


      //since the coordinates of the components are given with compound com_ as center of a transformed coordinate system, 
      //we need to transform back to original world coordinates first before collision quantities may be obtained correctly
      cbody0_->transform_.setOrigin(cbody0_->com_);
      MATRIX3X3 rot = cbody0_->getTransformationMatrix();
      cbody0_->transform_.setMatrix(rot);

      cbody1_->transform_.setOrigin(cbody1_->com_);
      MATRIX3X3 rot2 = cbody1_->getTransformationMatrix();
      cbody1_->transform_.setMatrix(rot2);

      if (p0->getShape() == RigidBody::SPHERE){
        //use collision detection for sphere

        //loop over all components of second body
        for (int j = 0; j < cbody1_->getNumComponents(); j++)
        {
          RigidBody *p1 = cbody1_->getComponent(j);

          /**	VECTOR3 pos00 = cbody0_->getComponent(0)->com_;
            VECTOR3 pos01 = cbody0_->getComponent(1)->com_;
            VECTOR3 pos02 = cbody0_->getComponent(2)->com_;

            VECTOR3 pos10 = cbody1_->getComponent(0)->com_;
            VECTOR3 pos11 = cbody1_->getComponent(1)->com_;
            VECTOR3 pos12 = cbody1_->getComponent(2)->com_; */

          if (p1->getShape() == RigidBody::SPHERE){
            //now use collision detection for spheres 


            //same procedure as in colliderspheresphere, extended for use with compounds
            //any positions of components are obtained by getTransformedPosision()
            const Real contactTolerance = 0.00005;
            VECTOR3 vel1 = p0->velocity_;
            VECTOR3 pos1 = p0->getTransformedPosition();

            Sphere<Real> *pSphere = dynamic_cast<Sphere<Real>* >(p0->shape_);
            Real rad1 = pSphere->getRadius();

            VECTOR3 vel2 = p1->velocity_;
            VECTOR3 pos2 = p1->getTransformedPosition();

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
            Real distmid = (pos2 - pos1).mag();
            Real dist1 = fabs(vn*vel1);
            Real dist2 = fabs(vn*vel2);
            Real distpertime = (dist1 + dist2)*world_->timeControl_->GetDeltaT();

            if (velalongnormal < -0.005 && distpertime >= dist)
              //if(relativeNormalVelocity < -0.005)
            {
              Contact contact;
              contact.m_dDistance = distmid;
              contact.m_vNormal = vn;
              contact.m_vPosition0 = pos1;
              contact.m_vPosition1 = pos2;
              contact.m_pBody0 = body0_;
              contact.m_pBody1 = body1_;
              contact.id0 = contact.m_pBody0->iID_;
              contact.id1 = contact.m_pBody1->iID_;
              contact.vn = velalongnormal;
              contact.m_dPenetrationDepth = std::min(Real(0.0), Real(dist));
              contact.m_iState = CollisionInfo::TOUCHING;
              //set additional component collision parameters 
              contact.cbody0 = cbody0_;
              contact.subId0 = i;
              contact.cbody1 = cbody1_;
              contact.subId1 = j;
              contact.type0 = RigidBody::COMPOUND;
              contact.type1 = RigidBody::COMPOUND;

              //std::cout<<"Pre-contact normal velocity: "<<velalongnormal<<" colliding contact"<<std::endl;
              //std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetAngVel();
              //std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetAngVel();
              //std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
              //std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
              vContacts.push_back(contact);
            }
            else if (velalongnormal < 0.00001 && fabs(dist) < contactTolerance)
            {
              Contact contact;
              contact.m_dDistance = distmid;
              contact.m_vNormal = vn;
              contact.m_vPosition0 = pos1;
              contact.m_vPosition1 = pos2;
              contact.m_pBody0 = body0_;
              contact.m_pBody1 = body1_;
              contact.id0 = contact.m_pBody0->iID_;
              contact.id1 = contact.m_pBody1->iID_;
              contact.vn = velalongnormal;
              contact.m_iState = CollisionInfo::TOUCHING;

              //component parameters 
              contact.cbody0 = cbody0_;
              contact.subId0 = i;
              contact.cbody1 = cbody1_;
              contact.subId1 = j;
              contact.type0 = RigidBody::COMPOUND;
              contact.type1 = RigidBody::COMPOUND;

              vContacts.push_back(contact);
            }
            else if (dist < 0.1*rad1)
            {
              Contact contact;
              contact.m_dDistance = distmid;
              contact.m_vNormal = vn;
              contact.m_vPosition0 = pos1;
              contact.m_vPosition1 = pos2;
              contact.m_pBody0 = body0_;
              contact.m_pBody1 = body1_;
              contact.id0 = contact.m_pBody0->iID_;
              contact.id1 = contact.m_pBody1->iID_;
              contact.vn = velalongnormal;
              contact.m_iState = CollisionInfo::TOUCHING;
              //component parameters 
              contact.cbody0 = cbody0_;
              contact.subId0 = i;
              contact.cbody1 = cbody1_;
              contact.subId1 = j;
              contact.type0 = RigidBody::COMPOUND;
              contact.type1 = RigidBody::COMPOUND;

              vContacts.push_back(contact);
            }
          }

        }

      }

    }

  }
}
