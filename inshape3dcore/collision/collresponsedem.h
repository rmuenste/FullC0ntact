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
#ifndef CCOLLRESPONSEDEM_H_
#define CCOLLRESPONSEDEM_H_
#include <collresponse.h>
#include <vectorn.h>

namespace i3d {

/**
 * @brief An Discrete Element Method (DEM) based collision response solver
 */  
class CollResponseDEM : public CollResponse
{
public:
	CollResponseDEM(void);

	CollResponseDEM(std::list<CollisionInfo> *CollInfo, World *pWorld);

	~CollResponseDEM(void);

  void Solve();
  
private:

  /**
  * Calculate and apply the impluses using the DEM-scheme
  */
  void ApplyImpulse(CollisionInfo &ContactInfo, Real &delta);

  /**
  * Applies a stiction force to particles in contact with the boundary
  */
  void ApplyStiction(CollisionInfo &ContactInfo, Real &delta);

  int GetNumIterations() { return iterations_; };

  /**
  * Set the number of iterations
  */
  void setMaxIterations(int i) { iterations_ = i; };

  void setDefEps(Real e) { eps_ = e; };

  Real getDefEps() { return eps_; };

  VectorNr defect_;

  Real oldDefect_;

  Real m_dBiasFactor;

  int  nContactInfos;

  /**
  * The maximum number of iterations of the solver
  */
  int iterations_;

  Real eps_;

};

}

#endif
