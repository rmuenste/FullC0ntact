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

#ifndef INTERSECTOR2OBB3_H
#define INTERSECTOR2OBB3_H

#include <obb3.h>
#include <mymath.h>
#include <iostream>
#include <stdlib.h>
#include <limits>
#include <vector>
#include <intersectortools.h>

namespace i3d {

/**
* @brief A class for an intersection test between 2 obb3s
*
* A class that performs an intersection test between 2 obb3s
* using the separating axes theorem
*/
template <typename T>
class CIntersector2OBB3
{
public:
  /**
  * Creates an intersector for two obbs
  * @param box0 the first box
  * @param box1 the second box
  */
  CIntersector2OBB3(const OBB3<T>& box0, const OBB3<T>& box1);

  /**
  * Gets the first box
  * @return Returns the first box
  */
  const OBB3<T>& GetBox0() const;

  /**
  * Gets the second box
  * @return Returns the second box
  */
  const OBB3<T>& GetBox1() const;

  /**
  * Tests for an intersection between the two boxes
  * @return Returns true if there is an intersection and false if there is none
  *
  */
  bool Test();

  bool Test2(const CVector3<T> &vel0, const CVector3<T> &vel1);

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

  /**
  * Returns true if the objects penetrate
  * @return True in case of penetration
  */  
  bool Penetration() const {return m_bPenetration;};

  /**
  * Returns the penetration depth
  * @return the penetration depth
  */  
  T GetPenetrationDepth() const {return cfg0.m_dMinOverlap;};

  CProjCfg<T> cfg0,cfg1;
  
private:

  /**
  * Computes the projection intervall on the axis
  * @param verts The vertices of the box
  * @param min Minimum projection
  * @param max Maximum projection
  */  
  void ComputeIntervall(CVector3<T> verts[8], const CVector3<T> &axis, T &min, T &max);

  /**
  * Check if the interval overlap 
  */
  inline bool overlap(Real min1, Real max1,
                            Real min2, Real max2) {
      return !(min1 > max2 || max1 < min2);
  }


  const OBB3<T>* m_Box0;
  const OBB3<T>* m_Box1;

  bool m_bPenetration;

  int m_iQuantity;
  CVector3<T> m_vPoint[8];
  std::vector<CVector3<T> > m_vContacts;
  T m_dContactTime;
  CVector3<T> m_vNormal;

};

template <typename T>
CIntersector2OBB3<T>::CIntersector2OBB3 (const OBB3<T>& box0, const OBB3<T>& box1) : m_Box0(&box0),m_Box1(&box1)
{
  m_bPenetration = false;
  m_iQuantity = 0;
}

template <typename T>
const OBB3<T>& CIntersector2OBB3<T>::GetBox0 () const
{
    return *m_Box0;
}

template <typename T>
const OBB3<T>& CIntersector2OBB3<T>::GetBox1 () const
{
    return *m_Box1;
}

typedef CIntersector2OBB3<double> CIntersector2OBB3f;
typedef CIntersector2OBB3<double> CIntersector2OBB3d;
typedef CIntersector2OBB3<Real> CIntersector2OBB3r;

}

#endif // INTERSECTOR2OBB3_H
