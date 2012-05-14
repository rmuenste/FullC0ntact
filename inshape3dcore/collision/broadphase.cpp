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

#include "broadphase.h"
#include <world.h>
#include <broadphasestrategy.h>

namespace i3d {

CBroadPhase::CBroadPhase(CWorld *pDomain, CBroadPhaseStrategy *pStrategy)
{
  this->m_pStrat=pStrategy;
  this->m_pWorld=pDomain;
}

CBroadPhase::CBroadPhase()
{
  this->m_pWorld = NULL;
  this->m_pStrat  = NULL;
}

CBroadPhase::CBroadPhase(const CBroadPhase &copy)
{
	this->m_pWorld  = copy.m_pWorld;
	this->m_pStrat   = copy.m_pStrat;
}

CBroadPhase::~CBroadPhase()
{

}

void CBroadPhase::Start()
{
	m_pStrat->Init();
	m_pStrat->Start();
}

}