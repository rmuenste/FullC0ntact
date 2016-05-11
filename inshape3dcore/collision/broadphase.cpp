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

BroadPhase::BroadPhase(World *pDomain, BroadPhaseStrategy *pStrategy)
{
  this->strategy_ = pStrategy;
  this->world_    = pDomain;
}

BroadPhase::BroadPhase()
{
  this->world_     = NULL;
  this->strategy_  = NULL;
}

BroadPhase::BroadPhase(const BroadPhase &copy)
{
	this->world_    = copy.world_;
	this->strategy_ = copy.strategy_;
}

BroadPhase::~BroadPhase()
{

}

void BroadPhase::start()
{
	strategy_->start();
}

}
