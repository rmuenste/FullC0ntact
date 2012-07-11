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

#ifndef COLLRESPONSE_H
#define COLLRESPONSE_H
#include <collisioninfo.h>
#include <response.h>
#include <list>
#include <vector>
#include <log.h>
#include <contactgraph.h>

namespace i3d {

class CWorld;


/**
* @brief Base class for a collision response model
*
* The base class for any collision response model, if a
* new model is added the interface of of the base class has to
* be implemented.
*/
class CCollResponse
{
  public:
	CCollResponse();

/**
* Creates a new instance of CCollResponse,  which corresponds to a collision response model 
* or collision response solver
* @param CollInfo a list of the collisions in the current timestep
* @param pWorld   a pointer to the physics world
*/
	CCollResponse(std::list<CCollisionInfo> *CollInfo,CWorld *pWorld);

/**
* Sets the collision epsilon
*/
	inline void SetEPS(Real CollEps) {m_dCollEps = CollEps;};
/**
* Returns the collision epsilon
* @return The collision epsilon
*/	
	inline Real GetEPS() const {return m_dCollEps;};

	virtual ~CCollResponse();

/**
* 
* Calls the collision model's response module
*
*/
  virtual void Solve();

  virtual int GetNumIterations() {return 0;};
  
	
	std::list<CCollisionInfo> *m_CollInfo;

/** If the distance between two objects is below m_dCollEps they are declared as collided */
	Real m_dCollEps;

	CWorld *m_pWorld;

	CContactGraph *m_pGraph;

  double dTimeAssembly;
  double dTimeSolver;
  double dTimeSolverPost;
  double dTimeAssemblyDry;

  /** 
  * number of total contact points detected in the narrow phase
  */
  int m_iContactPoints;

};

}

#endif // COLLRESPONSE_H
