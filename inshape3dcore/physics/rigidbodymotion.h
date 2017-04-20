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

#ifndef RIGIDBODYMOTION_H_
#define RIGIDBODYMOTION_H_

#include <vector3.h>
#include <vector>
#include <rigidbody.h>
#include <contact.h>

namespace i3d {

class TimeControl;
class World;

/**
* @brief A numerical integrator for the rigid body system
*
* A class that advances the physics system in time by means of
* numerical integration. This class uses exlicit euler as integrator.
*/
class RigidBodyMotion
{
public:
  RigidBodyMotion(void);

  RigidBodyMotion(World* pDomain);

  virtual ~RigidBodyMotion(void);

  /**
  * @brief Advances the system to the next timestep
  * Advances the system to the next timestep using explicit euler as integrator
  */
  virtual void updatePosition();
  
  /**
  * @brief Integrates the forces acting on the bodies
  * Integrates the forces acting on the bodies using explicit euler as integrator
  */
  virtual void updateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque);  

  void updateForces(std::vector<VECTOR3> &force, std::vector<VECTOR3> &torque, Real scale);  

  World *world_;

  TimeControl *timeControl_;

};

}

#endif
