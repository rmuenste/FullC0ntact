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

CollisionInfo::CollisionInfo()
{
  accumulatedImpulse_ = 0.0;
}


CollisionInfo::CollisionInfo(const CollisionInfo &copy)
{
  iID1             = copy.iID1;
  iID2             = copy.iID2;
  m_pBody0         = copy.m_pBody0;
  m_pBody1         = copy.m_pBody1;

  m_vContacts      = copy.m_vContacts;
  
  m_iState         = copy.m_iState;
  m_iPrevState     = copy.m_iPrevState;
  m_iTimeStamp     = copy.m_iTimeStamp;
  m_iPrevTimeStamp = copy.m_iPrevTimeStamp;
  m_iGroup         = copy.m_iGroup;

  m_iLayer         = copy.m_iLayer;
  m_iHeight        = copy.m_iHeight;
  accumulatedImpulse_ = copy.accumulatedImpulse_;
}

CollisionInfo::CollisionInfo(RigidBody *pBody0, RigidBody *pBody1,int id1,int id2)
{
  m_pBody0 = pBody0;
  m_pBody1 = pBody1;
  iID1 = id1;
  iID2 = id2;
  
  m_iState         = 0;
  m_iPrevState     = 0;   
  m_iTimeStamp     = 0;   
  m_iPrevTimeStamp = 0;   
  m_iLayer         = 0;
  m_iHeight        = 0;
  accumulatedImpulse_ = 0.0;
}

CollisionInfo::CollisionInfo(RigidBody *pBody0, RigidBody *pBody1,Real dist,int id1,int id2)
{
  m_pBody0 = pBody0;
  m_pBody1 = pBody1;
  iID1 = id1;
  iID2 = id2;
  
  m_iState         = 0;
  m_iPrevState     = 0;   
  m_iTimeStamp     = 0;   
  m_iPrevTimeStamp = 0;

  m_iLayer         = 0;
  m_iHeight        = 0;
}

CollisionInfo::~CollisionInfo()
{

}

void CollisionInfo::CacheContacts()
{
  m_vContactCache.clear();
  m_vContactCache = m_vContacts;
}

void CollisionInfo::CheckCache()
{

  for(int i=0;i<m_vContactCache.size();i++)
  {
    for(int j=0;j<m_vContacts.size();j++)
    {
      Real dist0 = (m_vContactCache[i].m_vPosition0 - m_vContacts[j].m_vPosition0).mag();
      Real dist1 = (m_vContactCache[i].m_vPosition1 - m_vContacts[j].m_vPosition1).mag();
      if(dist0 <=CMath<Real>::EPSILON3 && dist1 <=CMath<Real>::EPSILON3)
      {
        m_vContacts[j].m_dAccumulatedNormalImpulse = m_vContactCache[i].m_dAccumulatedNormalImpulse;
        if (m_vContactCache[i].m_iTimeStamp == m_vContacts[j].m_iCreationTime-1)
          m_vContacts[j].contactDisplacement = m_vContactCache[i].contactDisplacement;
        break;
      }
    }
  }

}

}