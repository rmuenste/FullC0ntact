/*
 A motion integrator for the discrete element method (DEM)
 
Author: Giuseppe Battaglia 
*/

#ifndef MOTIONINTEGRATORDEM_H_
#define MOTIONINTEGRATORDEM_H_

#include <vector3.h>
#include <vector>
#include <rigidbody.h>
#include <compoundbody.h>
#include <contact.h>
#include <rigidbodymotion.h>

namespace i3d {

class TimeControl;
class World;

/**
* @brief A numerical integrator for the rigid body system
*
* A class that advances the physics system in time by means of
* numerical integration. This class uses a taylor expansion based approach as integrator.
*/
class MotionIntegratorDEM : public RigidBodyMotion
{
public:
	MotionIntegratorDEM(void);

	MotionIntegratorDEM(World* pDomain);

	virtual ~MotionIntegratorDEM(void);

/**
* @brief Advances the system to the next timestep
* Advances the system to the next timestep using explicit euler as integrator
*/
	virtual void updatePosition();
  
};

}

#endif
