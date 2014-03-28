/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef COMPOUNDBODY_H
#define COMPOUNDBODY_H

#include <rigidbody.h>

namespace i3d {

/**
*  @brief A body that is compounded of several rigid bodies 
*
*  A body that is compounded of several rigid bodies 
*/
class CompoundBody : public RigidBody
{
public:

  /**
  *
  * Creates an empty rigid body
  *
  */
  CompoundBody();

  virtual ~CompoundBody();

  /** 
  * Copy a rigid body
  */
  CompoundBody(const CompoundBody& copy);

  void translateTo(const VECTOR3 &vPos);
  
  /** 
  *
  * Computes the inverse of the inertia tensor and stores it
  * in the member variable m_InvInertiaTensor
  *
  */
  void generateInvInertiaTensor();

  /**
   * Returns the radius of a bounding sphere for the body
   **/
  Real getBoundingSphereRadius() {return 1.0;}

  /**
  * @see CRigidBody::GetID
  */
  int getID() {return iID_;};

  /**
  * @see CRigidBody::SetID
  */
  void setID(int id)
  {
    iID_=id;
    std::vector<RigidBody*>::iterator iter = rigidBodies_.begin();
    for(;iter!=rigidBodies_.end();iter++)
    {
      (*iter)->setID(id);
    }
  };

  /**
  * Get the number of bodies that form the compound body
  */
  inline unsigned int getNumComponents() {return rigidBodies_.size();};

  /**
  * Get the number of bodies that form the compound body
  */
  inline RigidBody* getComponent(int i){return rigidBodies_[i];};
  
  std::vector<RigidBody*> rigidBodies_;

};

}

#endif // RIGIDBODY_H
