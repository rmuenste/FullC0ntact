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

#ifndef TOIESTIMATOR_H
#define TOIESTIMATOR_H
#include <queue>
#include <compare.h>

namespace i3d {

class World;

class TOIEstimator
{
public:

	TOIEstimator(void);
	TOIEstimator(World * pDomain, std::priority_queue<CollisionInfo,std::vector<CollisionInfo> , CmpInfos> *pCollisionHeap);
	~TOIEstimator(void);

	World *m_pWorld;

	//the collision heap member variable
	std::priority_queue<CollisionInfo,std::vector<CollisionInfo> , CmpInfos> *m_pCollisionHeap;

	// the function initializes the collision heap and computes the TOIs for all pairs
	void BuildHeap(void);
	// the function updates the collision heap after a collision changed the velocities of the particles
	void UpdateHeap(void);

private:


	Real CalcLowerBound(const VECTOR3 &p0, const VECTOR3 &p1, const VECTOR3 &v0, const VECTOR3 &v1, Real rad0, Real rad1, Real &dist);

	Real CalcRootDist(const VECTOR3 &p0, const VECTOR3 &p1, const VECTOR3 &v0, const VECTOR3 &v1, Real rad0, Real rad1);

	Real CalcFreeFall(Real dAcceleration, Real dVelocity, Real dDistance);
	Real CalcRiseFall(Real dAcceleration, Real dVelocity, Real dDistance);
	Real CalcRise(Real dAcceleration, Real dVelocity, Real dDistance);
	Real CalcConstVelocity(Real dVelocity, Real dDistance);

};

}

#endif
