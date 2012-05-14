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

#ifndef COLLISIONINFO_H
#define COLLISIONINFO_H
#include <mathglobals.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <aabb3.h>
#include <contact.h>
#include <vector>

namespace i3d {

class C3DModel;
class CRigidBody;

/**
* @brief A pair of rigid bodies identified as close proximity by the broadphase
*
* A pair of rigid bodies identified as close proximity by the broadphase
*
*/
class CBroadPhasePair
{
  public:

    CBroadPhasePair(){};

    CBroadPhasePair(CRigidBody *b0, CRigidBody *b1) : m_pBody0(b0), m_pBody1(b1)
    {
      m_iState         = DEFAULT;
      m_iPrevState     = DEFAULT;
      m_iTimeStamp     = DEFAULT;
      m_iPrevTimeStamp = DEFAULT;
    };

    ~CBroadPhasePair(){};

    bool HasSubdomainBoundary() {return (m_pBody0->m_iShape == CRigidBody::SUBDOMAIN || m_pBody1->m_iShape == CRigidBody::SUBDOMAIN);};

    CRigidBody* GetPhysicalBody() const
    {
      if(m_pBody0->m_iShape == CRigidBody::SUBDOMAIN)
        return m_pBody1;
      else
        return m_pBody0;
    };

    CRigidBody *m_pBody0;
    CRigidBody *m_pBody1;

    int m_iState;
    int m_iPrevState;

    int m_iTimeStamp;
    int m_iPrevTimeStamp;

	  enum
	  {
      TOUCHING,
      VANISHED_TOUCHING,
      CLOSEPROXIMITY,
      PERSISTENT_CLOSEPROXIMITY,
      PERSISTENT_TOUCHING,
      VANISHING_CLOSEPROXIMITY,
      COLLIDING,
      OBSOLETE,
      DEFAULT
	  };

};

///@cond HIDDEN_SYMBOLS
class Comp
{
public:
  bool operator()(CBroadPhasePair s1, CBroadPhasePair s2)
  {
    if(s1.m_pBody0->m_iID < s2.m_pBody0->m_iID)
      return true;

    if(s1.m_pBody0->m_iID == s2.m_pBody0->m_iID)
    {
      if(s1.m_pBody1->m_iID < s2.m_pBody1->m_iID)
        return true;
    }

    return false;
  }
};
///@cond

/**
* @brief All the contact information about two colliding bodies
*
* A class that stores all the contacts and information
* between two rigid bodies
*
*/
class CCollisionInfo
{
  public:
    
  CCollisionInfo();

 /**
 * Copy constructor for a collision info
 *
 */    
  CCollisionInfo(const CCollisionInfo &copy);
  
  CCollisionInfo(CRigidBody *pBody0, CRigidBody *pBody1,int id1,int id2);
  
  CCollisionInfo(CRigidBody *pBody0, CRigidBody *pBody1,Real dist,int id1,int id2);

  CCollisionInfo(CRigidBody *pBody0, CRigidBody *pBody1) : m_pBody0(pBody0), m_pBody1(pBody1) {};
  
  ~CCollisionInfo();

  void CacheContacts();

  void CheckCache();
  
  CRigidBody* GetOther(CRigidBody* body)
  {
    if(body==m_pBody0)
      return m_pBody1;
    else
      return m_pBody0;
  };
  
  int iID1;
  int iID2;
  int m_iNumContacts;
  CRigidBody *m_pBody0;
  CRigidBody *m_pBody1;

  int m_iLayer;
  int m_iGroup;
  int m_iHeight;    

  enum
  {
    TOUCHING,
    VANISHED_TOUCHING,
    CLOSEPROXIMITY,
    PERSISTENT_CLOSEPROXIMITY,
    PERSISTENT_TOUCHING,
    VANISHING_CLOSEPROXIMITY,
    COLLIDING,    
    OBSOLETE,
    DEFAULT
  };

  std::vector<CContact> m_vContacts;

  std::vector<CContact> m_vContactCache;
  
  int m_iState;
  int m_iPrevState;

  int m_iTimeStamp;
  int m_iPrevTimeStamp;

  int m_iCreationTime;

};

class CompColl
{
public:
  bool operator()(CCollisionInfo s1, CCollisionInfo s2)
  {
    if(s1.m_pBody0->m_iID < s2.m_pBody0->m_iID)
      return true;

    if(s1.m_pBody0->m_iID == s2.m_pBody0->m_iID)
    {
      if(s1.m_pBody1->m_iID < s2.m_pBody1->m_iID)
        return true;
    }

    return false;
  }
};


}

#endif // COLLISIONINFO_H
