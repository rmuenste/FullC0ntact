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

class CBroadPhaseStrategy;
class CWorld;

/**
* @brief Base class for broad phase algorithms
*
* Base class for broad phase algorithms that are used
* during collision detection
*
* TODO: implement different strategies nicely
*/

class CBroadPhase
{
	public:
	CBroadPhase(CWorld *pDomain,std::list<CCollisionInfo>* CollInfo, CBroadPhaseStrategy *pStrategy);
	CBroadPhase(CWorld *pDomain, CBroadPhaseStrategy *pStrategy);
	CBroadPhase(const CBroadPhase &copy);
	CBroadPhase();
	~CBroadPhase();

	inline void SetEPS(Real CollEps) {m_dCollEps = CollEps;};
	inline Real GetEPS() const {return m_dCollEps;};


	void GetAllPairs();

	virtual void Start();

	CBroadPhaseStrategy *m_pStrat;
	CWorld     *m_pWorld;
	std::list<CCollisionInfo>* m_CollInfo;
  std::set<CBroadPhasePair,Comp> * m_BroadPhasePairs;

	Real m_dCollEps;

};

}

#endif // BROADPHASE_H
