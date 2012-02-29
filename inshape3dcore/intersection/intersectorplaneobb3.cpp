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


#include "intersectorplaneobb3.h"
#include <mymath.h>

namespace i3d {

template <typename T>
bool CIntersectorOBB3Plane<T>::Test()
{
    return true;
}

template <class T>
bool CIntersectorOBB3Plane<T>::Find(T tmax, const CVector3<T> &vel0, const CVector3<T> &vel1)
{

  //T tlast = CMath<T>::MAXREAL;
  //T tfirst = T(0);
  //int iside = -1;
  //const T parallelTolerance = (T)1 - E3;
  //int i,j;

  //CVector3<T> relVel = vel1 - vel0;

  //CVector3<T> vAxis;

  //CVector3<T> vertices0[8];
  //m_Box0->ComputeVertices(vertices0);

  //cfg0.m_dMinOverlap = std::numeric_limits<T>::max();
  //cfg1.m_dMinOverlap = std::numeric_limits<T>::max();

  ////test the normals of box0
  //for(i=0;i<3;i++)
  //{
  //  vAxis = m_Box0->m_vUVW[i];
  //  //if there is no intersection for this axis, we found a separating axis
  //  if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
  //    return false;
  //}

  ////test the normals of box1
  //for(i=0;i<3;i++)
  //{
  //  vAxis = m_Box1->m_vUVW[i];
  //  //if there is no intersection for this axis, we found a separating axis
  //  if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
  //    return false;
  //}
  //
  ////cross product of edges
  //for(i=0;i<3;i++)
  //{
  //  for(j=0;j<3;j++)
  //  {
  //    if(fabs(m_Box0->m_vUVW[i]*m_Box1->m_vUVW[j]) > parallelTolerance)
  //    {
  //      //we found a pair of parallel axes
  //      //that means if the face normals did not separate the objects
  //      //the cross product of edges also will not separate the objects
  //      //ComputeContactSet()
  //      
  //      //easy way out, if the contact will
  //      //appear take place after
  //      //the max time
  //      if(tfirst > tmax)
  //        return false;

  //      //the normal can loose its length=1 property
  //      cfg0.m_vNormal.Normalize();
  //      cfg1.m_vNormal.Normalize();

  //      //if the boxes at current time are still separated
  //      if(tfirst > T(0.0))
  //      {
  //        CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,tfirst,m_iQuantity,m_vContacts);
  //        m_dContactTime = tfirst;
  //        m_vNormal = cfg0.m_vNormal;
  //      }
  //      else  //the boxes are interpenetrating
  //      {
  //        CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
  //        m_vNormal = cfg0.m_vMinNormal;
  //        cfg0.m_bPenetration = true;
  //        cfg1.m_bPenetration = true;
  //      }
  //      return true;
  //    }
  //    //compute the axis and check
  //    vAxis = CVector3<T>::Cross(m_Box0->m_vUVW[i],m_Box1->m_vUVW[j]);
  //    if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
  //      return false;
  //  }//end for j
  //}//end for i

  //if(tfirst <= T(0) || iside == CProjCfg<T>::NONE)
  //{
  //  std::cerr<<"Penetration detected CIntersector2OBB3<T>::Find"<<std::endl;

  //  //ComputeContactSet by minimumOverlap
  //  CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
  //  m_vNormal = cfg0.m_vMinNormal;
  //  cfg0.m_bPenetration = true;
  //  cfg1.m_bPenetration = true;
  //  return true;
  //}

  //if(tfirst > tmax)
  //  return false;

  ////compute the contact set
  //CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,tfirst,m_iQuantity,m_vContacts);
  //m_dContactTime = tfirst;
  //m_vNormal = cfg0.m_vNormal;
  return true;
}

template <class T>
bool CIntersectorOBB3Plane<T>::Find(const CVector3<T> &vel0, const CVector3<T> &vel1,
                                    const CVector3<T> &vAngVel0, const CVector3<T> &vAngVel1, T deltaT)
{
  //T tlast = CMath<T>::MAXREAL;
  //T tfirst = T(0);
  //int iside = -1;
  //const T parallelTolerance = (T)1 - E3;
  //int i,j;

  //CVector3<T> relVel = vel1 - vel0;

  //CVector3<T> vAxis;

  //CVector3<T> vertices0[8];
  //m_Box0->ComputeVertices(vertices0);

  //cfg0.m_dMinOverlap = std::numeric_limits<T>::max();
  //cfg1.m_dMinOverlap = std::numeric_limits<T>::max();

  //vAxis = m_Plane->m_vNormal;
  //CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,cfg0,cfg1);

  ////test the normals of box1
  //for(i=0;i<3;i++)
  //{
  //  vAxis = m_Box->m_vUVW[i];
  //  //if there is no intersection for this axis, we found a separating axis
  //  CIntersectorTools<T>::Find(vAxis,*m_Box,*m_Box1,cfg0,cfg1);
  //}
  //
  //bool parallel = false;
  ////cross product of edges
  //for(i=0;i<3;i++)
  //{
  //  if(parallel)
  //    break;
  //  for(j=0;j<3;j++)
  //  {
  //    if(parallel)
  //      break;
  //    if(fabs(m_Box0->m_vUVW[i]*m_Box1->m_vUVW[j]) > parallelTolerance)
  //    {
  //      //we found a pair of parallel axes
  //      //that means if the face normals did not separate the objects
  //      //the cross product of edges also will not separate the objects
  //      //ComputeContactSet()
  //      parallel = true;
  //      continue;
  //    }

  //    vAxis = CVector3<T>::Cross(m_Box0->m_vUVW[i],m_Box1->m_vUVW[j]);
  //    CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,cfg0,cfg1);
  //  }//end for j
  //}//end for i

  //cfg0.m_vMinNormal.Normalize();
  //cfg1.m_vMinNormal.Normalize();

  //m_bPenetration = true;

  ////ComputeContactSet by minimumOverlap
  //CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
  //m_vNormal = cfg0.m_vMinNormal;

  return true;
}

template <class T>
bool CIntersectorOBB3Plane<T>::Find(T tmax, int nSteps, const CVector3<T> &vel0, const CVector3<T> &vel1,
                                    const CVector3<T> &axisAngle0, const CVector3<T> &axisAngle1)
{

  //T stepsize = tmax/T(nSteps);

  //COBB3<T> Box0(*m_Box0);
  //COBB3<T> Box1(*m_Box1);

  //COBB3<T> Box2(*m_Box1);
  //
  ////if(tmax > 0)
  ////{
  ////  CVector3<T> newCenter1 = Box2.m_vCenter + tmax * vel1;
  ////  CVector3<T> diff1      = Box2.m_vCenter - newCenter1;
  ////  CVector3<T> angTerm    = CVector3<T>::Cross(axisAngle1,diff1);
  ////  CVector3<T> newvel1    = (vel1 + CVector3<T>::Cross(axisAngle1,diff1));

  ////  //move the boxes
  ////  Box2.m_vCenter += tmax * newvel1;
  ////  for(int j=0;j<3;j++)
  ////  {
  ////    CVector3<T> update = stepsize * CVector3<T>::Cross(axisAngle1,Box2.m_vUVW[j]);
  ////    Box2.m_vUVW[j] += update;
  ////  }

  ////  CVector3<T> vertices[8];
  ////  Box2.ComputeVertices(vertices);
  ////  std::cout<<""<<std::endl;
  ////  for(int k=0;k<8;k++)
  ////  {
  ////    std::cout<<vertices[k];
  ////  }
  ////}

  //for(int i=1;i<=nSteps;i++)
  //{
  //  T time = stepsize * T(i);
  //  CVector3<T> newCenter0 = Box0.m_vCenter + time * vel0;
  //  CVector3<T> newCenter1 = Box1.m_vCenter + time * vel1;
  //  CVector3<T> diff0      = Box0.m_vCenter - newCenter0;
  //  CVector3<T> diff1      = Box1.m_vCenter - newCenter1;
  //  CVector3<T> newvel0    = (vel0 + CVector3<T>::Cross(axisAngle0,diff0));
  //  CVector3<T> newvel1    = (vel1 + CVector3<T>::Cross(axisAngle1,diff1));
  //  CVector3<T> vertices[8];
  //  Box0.ComputeVertices(vertices);
  //  CIntersector2OBB3<T> intr(Box0,Box1);
  //  if(intr.Find(stepsize,newvel0,newvel1))
  //  {
  //    m_vContacts = intr.GetContacts();
  //    m_vNormal = intr.GetNormal();
  //    m_iQuantity = intr.GetQuantity();
  //    m_dContactTime = intr.GetContactTime();
  //    return true;
  //  }

  //  //move the boxes
  //  Box0.m_vCenter += stepsize * newvel0;
  //  Box1.m_vCenter += stepsize * newvel1;

  //  //Box0.m_vCenter += newvel0;
  //  //Box1.m_vCenter += newvel1;


  //  for(int j=0;j<3;j++)
  //  {
  //    Box0.m_vUVW[j] += stepsize * CVector3<T>::Cross(axisAngle0,Box0.m_vUVW[j]);

  //    Box1.m_vUVW[j] += stepsize * CVector3<T>::Cross(axisAngle1,Box1.m_vUVW[j]);
  //  }


  //}

  return false;
}


//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorOBB3Plane<Real>;
//----------------------------------------------------------------------------

}
