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

#ifndef CONTACT_H
#define CONTACT_H

#include <mathglobals.h>
#include <vector3.h>
#include <rigidbody.h>

namespace i3d {

/**
* @brief A single contact point
*
* A class that stores the contact information for a
* single contact point.
*/
class CContact
{
public:
  /**
  * Creates a new empty contact
  */
	CContact(void);

  /**
  * Copies a contact
  */
	CContact(const CContact &copy);

	~CContact(void);

	/** distance between the objects */
	double m_dDistance;
  
  Real GetSign(CRigidBody *pBody);

	/**
	* The collision normal, the convention is that it points form the
  * second body to the first
  */
	VECTOR3 m_vNormal;
  
  /**
   * For frictional contacts we need a tangent plane
   * at the contact point. This plane is defined by
   * two tangent vectors
   */
  VECTOR3 m_vTangentU;
  VECTOR3 m_vTangentV;  
  
  /** The relative velocity */
	Real vn;
  
  Real m_dAccumulatedNormalImpulse;
  
  Real m_dAccumulatedTangentImpulse;
  
  Real m_dBiasImpulse;
  
  Real m_dMassNormal;
  
  Real m_dMassTangent;
  
  Real m_dRestitution;

  bool m_bResting;

  Real m_dPenetrationDepth;
  
  /** Position of the contact on the first body */
  VECTOR3 m_vPosition0;

  /**
  *  The Position of the contact on the second body 
  *  if the distance is 0 the two are the same
  */
  VECTOR3 m_vPosition1;

  /** Pointer to the first body */
  CRigidBody *m_pBody0;

  /** Pointer to the second body */  
  CRigidBody *m_pBody1;

  /** ids of the bodies */
  int id0;
  int id1;
  
  /** Current state of the contact */  
  int m_iState;
  
  /** Previous state of the contact */    
  int m_iPrevState;

  /** Time of creation of the contact */      
  int m_iTimeStamp;
  int m_iPrevTimeStamp;

  int m_iCreationTime;


};

}

#endif
