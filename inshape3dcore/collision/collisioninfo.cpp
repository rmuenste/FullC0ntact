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

#include "collisioninfo.h"
#include <3dmodel.h>
#include <rigidbody.h>

namespace i3d {

CCollisionInfo::CCollisionInfo()
{

}


CCollisionInfo::CCollisionInfo(const CCollisionInfo &copy)
{
  m_dDistance      = copy.m_dDistance;
  iID1             = copy.iID1;
  iID2             = copy.iID2;
  pNode1           = copy.pNode1;
  pNode2           = copy.pNode2;
  m_pBody1         = copy.m_pBody1;
  m_pBody2         = copy.m_pBody2;
  m_vCollNormal    = copy.m_vCollNormal;
  m_vP0            = copy.m_vP0;
  m_vP1            = copy.m_vP1;

  m_TOI            = copy.m_TOI;
  m_iCollisionID   = copy.m_iCollisionID;
  m_dDeltaT        = copy.m_dDeltaT;
  m_vContacts      = copy.m_vContacts;
  
  m_iState         = copy.m_iState;
  m_iPrevState     = copy.m_iPrevState;
  m_iTimeStamp     = copy.m_iTimeStamp;
  m_iPrevTimeStamp = copy.m_iPrevTimeStamp;
  m_iGroup         = copy.m_iGroup;

  m_iLayer         = copy.m_iLayer;
  m_iHeight        = copy.m_iHeight;

}

CCollisionInfo::CCollisionInfo(CRigidBody *pBody1, CRigidBody *pBody2,int id1,int id2)
{
  m_pBody1 = pBody1;
  m_pBody2 = pBody2;
  iID1 = id1;
  iID2 = id2;
  
  m_iState         = 0;
  m_iPrevState     = 0;   
  m_iTimeStamp     = 0;   
  m_iPrevTimeStamp = 0;   
  m_iLayer         = 0;
  m_iHeight        = 0;
}

CCollisionInfo::CCollisionInfo(CRigidBody *pBody1, CRigidBody *pBody2,Real dist,int id1,int id2)
{
  m_pBody1 = pBody1;
  m_pBody2 = pBody2;
  m_dDistance = dist;
  iID1 = id1;
  iID2 = id2;
  
  m_iState         = 0;
  m_iPrevState     = 0;   
  m_iTimeStamp     = 0;   
  m_iPrevTimeStamp = 0;

  m_iLayer         = 0;
  m_iHeight        = 0;
}

CCollisionInfo::~CCollisionInfo()
{

}

}