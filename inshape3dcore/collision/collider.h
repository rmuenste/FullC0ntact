/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef COLLIDER_H
#define COLLIDER_H

#include <rigidbody.h>
#include <contact.h>
#include <contactgenerator.h>

namespace i3d {

class World;

/**
* @brief Base class for a collider
*
* A collider is a class that represents an 'intersection test/find' algorithm 
* between two geometric primitives
*
*/
class Collider
{
public:
	Collider(void);

	virtual ~Collider(void);

  /**
   * Setter method for member variable m_pBody0
   */  
  void setBody0(RigidBody *body){body0_=body;};
  
  /**
   * Setter method for member variable m_pBody1
   */    
  void setBody1(RigidBody *body){body1_=body;};

  /**
   * Getter method for member variable m_pBody0
   */      
  RigidBody* getBody0(){return body0_;};
  
  /**
   * Getter method for member variable m_pBody1
   */        
  RigidBody* getBody1(){return body1_;};

  /**
  * Computes whether the rigid bodies collide and in case of collision computes the contact points
  *
  * @param contacts The vector of contact points
  *
  */
  virtual void collide(std::vector<Contact> &contacts);

  
  /**
   * Sets the shape attribute for rigid body0
   */
  void setShape0(int id){shape0_=id;};
  
  /**
   * Sets the shape attribute for rigid body1
   */  
  void setShape1(int id){shape1_=id;};

  /**
   * Sets the world pointer variable
   */
  void setWorld(World *world) {world_ = world;};

  CContactGenerator<Real> *generator_;
  
  /**
   * World object pointer to access world information
   */
	World *world_;

protected:
  
  /**
   * Member variable to store the first body in a collider
   */
  RigidBody *body0_;  
  
  /**
   * Member variable to store the second body in a collider
   */  
  RigidBody *body1_;  

 /**
  * Stores the shape of the first body
  */
 int shape0_;
 
 /**
  * Stores the shape of the second body
  */ 
 int shape1_;

};

}

#endif
