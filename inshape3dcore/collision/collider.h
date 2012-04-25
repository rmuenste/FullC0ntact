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

#ifndef COLLIDER_H
#define COLLIDER_H

#include <rigidbody.h>
#include <contact.h>
#include <contactgenerator.h>

namespace i3d {

class CWorld;

/**
* @brief Base class for a collider
*
* A collider is a class that represents an 'intersection test/find' algorithm 
* between two geometric primitives
*
*/
class CCollider
{
public:
	CCollider(void);

	virtual ~CCollider(void);

  void SetBody0(CRigidBody *pBody){m_pBody0=pBody;};
  void SetBody1(CRigidBody *pBody){m_pBody1=pBody;};

  CRigidBody* GetBody0(){return m_pBody0;};
  CRigidBody* GetBody1(){return m_pBody1;};

/**
* Computes whether the rigid bodies collide and in case of collision computes the contact points
*
* @param  vContacts The vector of contact points
*
*/
	virtual void Collide(std::vector<CContact> &vContacts);

  void SetShape0(int id){m_iShape0=id;};
  void SetShape1(int id){m_iShape1=id;};

  void SetWorld(CWorld *pWorld) {m_pWorld = pWorld;};

  CContactGenerator<Real> *m_pGenerator;
	CWorld *m_pWorld;

protected:
  CRigidBody *m_pBody0;  
  CRigidBody *m_pBody1;  

 int m_iShape0;
 int m_iShape1;

};

}

#endif
