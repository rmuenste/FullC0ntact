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

#ifndef _INTERSECMPR_H
#define _INTERSECMPR_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <convexshape.h>

namespace i3d {

template<class T>
class CPortal
{
  public:
    CPortal() {

    };

    ~CPortal(){};

    CVector3<T> &v() {return m_Tetra[0];};
    CVector3<T> &a() {return m_Tetra[1];};
    CVector3<T> &b() {return m_Tetra[2];};
    CVector3<T> &c() {return m_Tetra[3];};
    CVector3<T> p() {return m_P[2];};

    void Swap(int i, int j)
    {
      //swap i and j to invert the normal
      CVector3<T> temp = m_Tetra[i];
      m_Tetra[i] = m_Tetra[j];
      m_Tetra[j] = temp;
      //points of shape0
      temp = m_Points0[i];
      m_Points0[i] = m_Points0[j];
      m_Points0[j] = temp;
      //points of shape1
      temp = m_Points1[i];
      m_Points1[i] = m_Points1[j];
      m_Points1[j] = temp;
    }

    void ReplaceAByNewSupport()
    {
      m_Tetra[1]=m_P[2];
      m_Points0[1]=m_P[0];
      m_Points1[1]=m_P[1];
    };

    void ReplaceBByNewSupport()
    {
      m_Tetra[2]=m_P[2];
      m_Points0[2]=m_P[0];
      m_Points1[2]=m_P[1];
    };

    void ReplaceCByNewSupport()
    {
      m_Tetra[3]=m_P[2];
      m_Points0[3]=m_P[0];
      m_Points1[3]=m_P[1];
    };

    void Replace(int i, const CVector3<T> &point)
    {

    }

    void GenerateNewSupport(const CVector3<T> &s0, const CVector3<T> &s1)
    {
      m_P[0] = s0;
      m_P[1] = s1;
      m_P[2] = s0 - s1;
    }

    CVector3<T> n,n_old;
    CVector3<T> m_Points0[4];
    CVector3<T> m_Points1[4];
    CVector3<T> m_Tetra[4];
    CVector3<T> m_P[3];

};

/**
* @brief Computes whether two convex objects intersect
*
* Computes whether two convex objects intersect
*
*/      
template<class T>
class CIntersectorMPR
{

public:

	/* constructors */
	CIntersectorMPR();

  CIntersectorMPR(const CConvexShape<T> &shape0, const CConvexShape<T> &shape1);

	/* deconstructors */
	~CIntersectorMPR(void){};

	/* member functions */
	bool Intersection();

private:

  void FindInitialPortal();

  void CheckPortalRay();

  void RefinePortal();

  void GenerateContactPoints();

  CVector3<T> m_vPoint0;
  CVector3<T> m_vPoint1;

  const CConvexShape<T> *m_pShape0;
  const CConvexShape<T> *m_pShape1; 

  CPortal<T> m_Portal;

  bool intersection;

};

typedef CIntersectorMPR<float> CIntersectorMPRf;
typedef CIntersectorMPR<double> CIntersectorMPRd;

}

#endif