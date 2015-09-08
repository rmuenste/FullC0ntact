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

class Model3D;
class RigidBody;

/**
* @brief A pair of rigid bodies identified as close proximity by the broadphase
*
* A pair of rigid bodies identified as close proximity by the broadphase
*
*/
class BroadPhasePair
{
  public:

    BroadPhasePair(){};

    BroadPhasePair(RigidBody *b0, RigidBody *b1) : m_pBody0(b0), m_pBody1(b1)
    {
      m_iState         = DEFAULT;
      m_iPrevState     = DEFAULT;
      m_iTimeStamp     = DEFAULT;
      m_iPrevTimeStamp = DEFAULT;
    };

    ~BroadPhasePair(){};

    bool HasSubdomainBoundary() {return (m_pBody0->shapeId_ == RigidBody::SUBDOMAIN || m_pBody1->shapeId_ == RigidBody::SUBDOMAIN);};

    RigidBody* GetPhysicalBody() const
    {
      if(m_pBody0->shapeId_ == RigidBody::SUBDOMAIN)
        return m_pBody1;
      else
        return m_pBody0;
    };

    RigidBody *m_pBody0;
    RigidBody *m_pBody1;

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
  bool operator()(BroadPhasePair s1, BroadPhasePair s2)
  {
    if(s1.m_pBody0->iID_ < s2.m_pBody0->iID_)
      return true;

    if(s1.m_pBody0->iID_ == s2.m_pBody0->iID_)
    {
      if(s1.m_pBody1->iID_ < s2.m_pBody1->iID_)
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
class CollisionInfo
{
  public:
    
  CollisionInfo();

 /**
 * Copy constructor for a collision info
 *
 */    
  CollisionInfo(const CollisionInfo &copy);
  
  CollisionInfo(RigidBody *pBody0, RigidBody *pBody1,int id1,int id2);
  
  CollisionInfo(RigidBody *pBody0, RigidBody *pBody1,Real dist,int id1,int id2);

  CollisionInfo(RigidBody *pBody0, RigidBody *pBody1) : m_pBody0(pBody0), m_pBody1(pBody1) {};
  
  ~CollisionInfo();

  void CacheContacts();

  void CheckCache();
  
  RigidBody* GetOther(RigidBody* body)
  {
    if(body==m_pBody0)
      return m_pBody1;
    else
      return m_pBody0;
  };
  
  int iID1;
  int iID2;
  int m_iNumContacts;
  RigidBody *m_pBody0;
  RigidBody *m_pBody1;

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

  std::vector<Contact> m_vContacts;

  std::vector<Contact> m_vContactCache;
  
  int m_iState;
  int m_iPrevState;

  int m_iTimeStamp;
  int m_iPrevTimeStamp;

  int m_iCreationTime;

  Real accumulatedImpulse_;

};

class CompColl
{
public:
  bool operator()(CollisionInfo s1, CollisionInfo s2)
  {
    if(s1.m_pBody0->iID_ < s2.m_pBody0->iID_)
      return true;

    if(s1.m_pBody0->iID_ == s2.m_pBody0->iID_)
    {
      if(s1.m_pBody1->iID_ < s2.m_pBody1->iID_)
        return true;
    }

    return false;
  }
};


}

#endif // COLLISIONINFO_H
