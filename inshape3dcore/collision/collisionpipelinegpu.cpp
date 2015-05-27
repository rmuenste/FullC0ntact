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

#include "collisionpipelinegpu.h"
#include <world.h>
#include <iostream>
#include <timecontrol.h>

namespace i3d {

CollisionPipelineGPU::CollisionPipelineGPU()
{

}

CollisionPipelineGPU::~CollisionPipelineGPU()
{
	
}

void CollisionPipelineGPU::startPipeline()
{
#ifdef FC_CUDA_SUPPORT
  //std::cout<<"Calling gpu update with deltaT: "<<world_->m_pTimeControl->GetDeltaT() <<std::endl;  
  //world_->psystem->update(world_->m_pTimeControl->GetDeltaT());
  int steps = 10;
  float myDeltaT = this->world_->timeControl_->GetDeltaT();
  myDeltaT/=float(steps);
  for(int i=0;i<steps;i++)
  {
    world_->psystem->update(myDeltaT);
  }

  //get particles from gpu
  world_->psystem->transferArrays(0, world_->psystem->getNumParticles());

  for (int i = 0; i<world_->rigidBodies_.size() - 1; i++)
  {
    world_->rigidBodies_[i]->com_.x = world_->psystem->m_hPos[4 * i];
    world_->rigidBodies_[i]->com_.y = world_->psystem->m_hPos[4 * i + 1];
    world_->rigidBodies_[i]->com_.z = world_->psystem->m_hPos[4 * i + 2];

    world_->rigidBodies_[i]->velocity_.x = world_->psystem->m_hVel[4 * i];
    world_->rigidBodies_[i]->velocity_.y = world_->psystem->m_hVel[4 * i + 1];
    world_->rigidBodies_[i]->velocity_.z = world_->psystem->m_hVel[4 * i + 2];
  }
#endif
}

}
