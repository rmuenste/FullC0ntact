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

CCollisionPipelineGPU::CCollisionPipelineGPU()
{

}

CCollisionPipelineGPU::~CCollisionPipelineGPU()
{
	
}

void CCollisionPipelineGPU::StartPipeline()
{
#ifdef FC_CUDA_SUPPORT
  //std::cout<<"Calling gpu update with deltaT: "<<m_pWorld->m_pTimeControl->GetDeltaT() <<std::endl;  
  //m_pWorld->psystem->update(m_pWorld->m_pTimeControl->GetDeltaT());
  int steps = 10;
  float myDeltaT = m_pWorld->m_pTimeControl->GetDeltaT();
  myDeltaT/=float(steps);
  for(int i=0;i<steps;i++)
  {
    m_pWorld->psystem->update(myDeltaT);
  }

  //get particles from gpu
  m_pWorld->psystem->transferArrays(0,m_pWorld->psystem->getNumParticles());

  for(int i=0;i<m_pWorld->m_vRigidBodies.size()-1;i++)
  {
    m_pWorld->m_vRigidBodies[i]->m_vCOM.x = m_pWorld->psystem->m_hPos[4*i]; 
    m_pWorld->m_vRigidBodies[i]->m_vCOM.y = m_pWorld->psystem->m_hPos[4*i+1]; 
    m_pWorld->m_vRigidBodies[i]->m_vCOM.z = m_pWorld->psystem->m_hPos[4*i+2];

    m_pWorld->m_vRigidBodies[i]->m_vVelocity.x = m_pWorld->psystem->m_hVel[4*i]; 
    m_pWorld->m_vRigidBodies[i]->m_vVelocity.y = m_pWorld->psystem->m_hVel[4*i+1]; 
    m_pWorld->m_vRigidBodies[i]->m_vVelocity.z = m_pWorld->psystem->m_hVel[4*i+2];
  }
#endif
}

}
