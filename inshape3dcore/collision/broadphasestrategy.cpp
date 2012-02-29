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

#include "broadphasestrategy.h"
#include <simplespatialhash.h>
#include <world.h>
#include <hspatialhash.h>

namespace i3d {

CBroadPhaseStrategy::CBroadPhaseStrategy(CWorld* pWorld,std::list<CCollisionInfo> *CollInfo)
{
  this->m_pWorld=pWorld;
  this->m_CollInfo=CollInfo;
  m_pTimeControl = m_pWorld->m_pTimeControl;
  m_pImplicitGrid = NULL;
}

CBroadPhaseStrategy::CBroadPhaseStrategy(CWorld* pWorld)
{
  this->m_pWorld=pWorld;
  m_pTimeControl = m_pWorld->m_pTimeControl;
  m_pImplicitGrid = NULL;
}

CBroadPhaseStrategy::~CBroadPhaseStrategy()
{
  if(m_pImplicitGrid != NULL)
  {
    delete m_pImplicitGrid;
    m_pImplicitGrid=NULL;
  }
}

void CBroadPhaseStrategy::Init()
{
  //this broadphase strategy does nothing
}

void CBroadPhaseStrategy::Start()
{
	int i,j;
	//Check every pair
  m_BroadPhasePairs->clear();
	for(i=0;i<this->m_pWorld->m_vRigidBodies.size();i++)
	{
		CRigidBody *p0=m_pWorld->m_vRigidBodies[i];

		//get the center of mass
		VECTOR3 pos0 = p0->m_vCOM;
		j=i+1;
		for(;j<(unsigned int)m_pWorld->m_vRigidBodies.size();j++)
		{
			CRigidBody *p1=m_pWorld->m_vRigidBodies[j];
      CBroadPhasePair pair(p0,p1);
      m_BroadPhasePairs->insert(pair);
    }
  }
}

}
