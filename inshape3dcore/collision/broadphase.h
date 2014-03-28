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

#ifndef BROADPHASE_H
#define BROADPHASE_H
#include <collisioninfo.h>
#include<list>
#include <set>
namespace i3d {

class BroadPhaseStrategy;
class World;

/**
* @brief Base class for broad phase algorithms
*
* Base class for broad phase algorithms that are used
* during collision detection
*
* TODO: implement different strategies nicely
*/

class BroadPhase
{
	public:
	BroadPhase(World *pDomain, BroadPhaseStrategy *pStrategy);
	BroadPhase(const BroadPhase &copy);
	BroadPhase();
	~BroadPhase();

	inline void SetEPS(Real CollEps) {m_dCollEps = CollEps;};
	inline Real GetEPS() const {return m_dCollEps;};

	virtual void Start();

	BroadPhaseStrategy *m_pStrat;

	World     *m_pWorld;

  std::set<BroadPhasePair,Comp> * m_BroadPhasePairs;

	Real m_dCollEps;

};

}

#endif // BROADPHASE_H
