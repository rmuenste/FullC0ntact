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

#ifndef MOTIONINTEGRATORSI_H_
#define MOTIONINTEGRATORSI_H_

#include <vector3.h>
#include <vector>
#include <rigidbody.h>
#include <contact.h>
#include <rigidbodymotion.h>

namespace i3d {

class CTimeControl;
class CWorld;

/**
* @brief A numerical integrator for the rigid body system
*
* A class that advances the physics system in time by means of
* numerical integration. This class uses exlicit euler as integrator.
*/
class CMotionIntegratorSI : public CRigidBodyMotion
{
public:
	CMotionIntegratorSI(void);

	CMotionIntegratorSI(CWorld* pDomain);

	virtual ~CMotionIntegratorSI(void);

/**
* @brief Advances the system to the next timestep
* Advances the system to the next timestep using explicit euler as integrator
*/
	virtual void UpdatePosition();
  
};

}

#endif
