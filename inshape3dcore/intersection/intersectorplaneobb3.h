/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef INTERSECTORPLANEOBB3_H
#define INTERSECTORPLANEOBB3_H

#include <obb3.h>
#include <mymath.h>
#include <iostream>
#include <stdlib.h>
#include <limits>
#include <vector>
#include <intersectortools.h>
#include <plane.h>

namespace i3d {

/**
* @brief A class for an intersection test between a plane and an obb3
*
* A class that performs an intersection test between a plane and an obb3
* using the separating axes theorem
*/
template <typename T>
class CIntersectorOBB3Plane
{
public:
  /**
  * Creates an intersector for two obbs
  * @param box the box
  * @param plane the plane
  */
  CIntersectorOBB3Plane(const OBB3<T>& box, const Plane<T>& plane);

  /**
  * Gets the box
  * @return Returns the box
  */
  const OBB3<T>& GetBox() const;

  /**
  * Gets the plane
  * @return Returns the plane
  */
  const Plane<T>& GetPlane() const;

  /**
  * Tests for an intersection between the two boxes
  * @return Returns true if there is an intersection and false if there is none
  *
  */
  bool Test();

  /**
  * Returns the vector of intersection points
  * @return The vector of intersection points
  */
  std::vector<CVector3<T> >& GetContacts() {return m_vContacts;};

  /**
  * Returns the number of intersection points
  * @return The number of intersection points
  */
  int GetQuantity(){return m_iQuantity;};

  /**
  * Returns the time when the objects will intersect
  * @return The intersection time
  */
  T   GetContactTime(){return m_dContactTime;};

  /**
  * Intersection find query for boxes that move with a constant
  * velocity.
  * @param tmax The length of the time interval that is checked
  * @param vel0 The velocity of the first box
  * @param vel1 The velocity of the second box
  * @return True if the boxes intersect in the given time interval
  */
  bool Find(T tmax, const CVector3<T> &vel0, const CVector3<T> &vel1);

  /**
  * Intersection find query for boxes that move with a constant
  * velocity and with a constant angular velocity.
  * @param tmax The length of the time interval that is checked
  * @param nSteps The number of substeps in the integration
  * @param vel0 The velocity of the first box
  * @param vel1 The velocity of the second box
  * @return True if the boxes intersect in the given time interval
  */
  bool Find(T tmax, int nSteps, const CVector3<T> &vel0, const CVector3<T> &vel1,
                    const CVector3<T> &axisAngle0, const CVector3<T> &axisAngle1);

  bool Find(const CVector3<T> &vel0, const CVector3<T> &vel1,
            const CVector3<T> &vAngVel0, const CVector3<T> &vAngVel1,T deltaT);

  /**
  * Returns the contact normal of hte intersection point
  * @return The contact normal
  */
  CVector3<T> GetNormal(){return m_vNormal;};

  bool Penetration() const {return m_bPenetration;};
  T GetPenetrationDepth() const {return cfg0.m_dMinOverlap;};

  CProjCfg<T> cfg0,cfg1;

  T Test2(CVector3<T> &vNormal);

  bool Test3(CVector3<T> &vNormal, T &rOverlap);

private:

  const OBB3<T>* m_Box;
  const Plane<T> *m_Plane;

  bool m_bPenetration;

  int m_iQuantity;
  CVector3<T> m_vPoint[8];
  std::vector<CVector3<T> > m_vContacts;
  T m_dContactTime;
  CVector3<T> m_vNormal;

};


template <typename T>
CIntersectorOBB3Plane<T>::CIntersectorOBB3Plane (const OBB3<T>& box, const Plane<T>& plane) : m_Box(&box),m_Plane(&plane)
{
  m_bPenetration = false;
  m_iQuantity = 0;
}

template <typename T>
const OBB3<T>& CIntersectorOBB3Plane<T>::GetBox() const
{
    return *m_Box;
}

template <typename T>
const Plane<T>& CIntersectorOBB3Plane<T>::GetPlane() const
{
    return *m_Plane;
}

typedef CIntersectorOBB3Plane<float> CIntersectorOBB3Planef;
typedef CIntersectorOBB3Plane<double> CIntersectorOBB3Planed;
typedef CIntersectorOBB3Plane<Real> CIntersectorOBB3Planer;

}

#endif // INTERSECTORPLANEOBB3_H
