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

BroadPhaseStrategy::BroadPhaseStrategy(World* pWorld)
{
  this->world_=pWorld;
  timeControl_ = world_->timeControl_;
  implicitGrid_ = NULL;
}

BroadPhaseStrategy::~BroadPhaseStrategy()
{
  if(implicitGrid_ != NULL)
  {
    delete implicitGrid_;
    implicitGrid_=NULL;
  }
}

void BroadPhaseStrategy::init()
{
  //this broadphase strategy does nothing
}

void BroadPhaseStrategy::start()
{
	int i,j;
	//Check every pair
  broadPhasePairs_->clear();
	for(i=0;i<this->world_->rigidBodies_.size();i++)
	{
		RigidBody *p0=world_->rigidBodies_[i];

		//get the center of mass
		VECTOR3 pos0 = p0->com_;
		j=i+1;
		for(;j<(unsigned int)world_->rigidBodies_.size();j++)
		{
			RigidBody *p1=world_->rigidBodies_[j];
      BroadPhasePair pair(p0,p1);
      broadPhasePairs_->insert(pair);
    }
  }
}

}
