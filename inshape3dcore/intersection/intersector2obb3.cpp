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


#include "intersector2obb3.h"
#include <mymath.h>

namespace i3d {

template <typename T>
bool CIntersector2OBB3<T>::Test()
{
    //we test the 6 face normals and 9 edge x edge pairs as
    //potential separating axes
    //if we find parallel face normals then the edge x edge axes
    //dont need to be checked

    //we want to detect parallel face normals, so test N_a * N_b == 1
    //to make the test numerically robust, we define a tolerance: fabs(N_a * N_b) > 1 - tolerance
    const T parallelTolerance = (T)1 - E5;
    bool parallelFaceNormals = false;
    //const T margin = E4;
    const T margin = T(0);
    int i;

    // Convenience variables.
    const CVector3<T>* A = m_Box0->m_vUVW;
    const CVector3<T>* B = m_Box1->m_vUVW;
    const T* EA = m_Box0->m_Extents;
    const T* EB = m_Box1->m_Extents;

		CVector3<T> vertices[8];

    // Compute difference of box centers, D = C1-C0.
    CVector3<T> D = m_Box1->m_vCenter - m_Box0->m_vCenter;

		//std::cout<<"Center B: "<<m_Box1->m_vCenter<<std::endl;		
		
    T C[3][3];     // matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
    T AbsC[3][3];  // |c_{ij}|
    T AD[3];       // Dot(A_i,D)
    T r0, r1, r;   // interval radii and distance between centers
    T r01;         // = R0 + R1

    // axis C0+t*A0
    for (i = 0; i < 3; ++i)
    {
				// matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
        C[0][i] = A[0]*(B[i]);
        AbsC[0][i] = fabs(C[0][i]);
        if (AbsC[0][i] > parallelTolerance)
        {
            parallelFaceNormals = true;
        }
    }
    //Using face normal A[0] as axis, so project the distance D = C1 - C0 on this axis
    AD[0] = A[0]*(D);
		
		//calc the absolute value
    r = fabs(AD[0]);
		//calculate the r_1: ext_0*Abs(C_00)+ext_1*Abs(C_01)+ext_2*Abs(C_02)
    r1 = EB[0]*AbsC[0][0] + EB[1]*AbsC[0][1] + EB[2]*AbsC[0][2];
		//add radii
    r01 = EA[0] + r1;
		//if the projection of D is larger, we have found a seperating axis if(r > r01+margin)
    if(r > r01+margin)
    {
        return false;
    }
    
    //Using face normal A[1] as axis, so project the distance D = C1 - C0 on this axis
    for (i = 0; i < 3; ++i)
    {
        C[1][i] = A[1]*(B[i]);
        AbsC[1][i] = fabs(C[1][i]);
        if (AbsC[1][i] > parallelTolerance)
        {
            parallelFaceNormals = true;
        }
    }
    AD[1] = A[1]*(D);
    r = fabs(AD[1]);
    r1 = EB[0]*AbsC[1][0] + EB[1]*AbsC[1][1] + EB[2]*AbsC[1][2];
    r01 = EA[1] + r1;
    if(r > r01+margin)
    {
				//std::cout<<"Seperating axis2: "<<A[1]<<std::endl;
        return false;
    }

    //Using face normal A[2] as axis, so project the distance D = C1 - C0 on this axis
    for (i = 0; i < 3; ++i)
    {
        C[2][i] = A[2]*(B[i]);
        AbsC[2][i] = fabs(C[2][i]);
        if (AbsC[2][i] > parallelTolerance)
        {
            parallelFaceNormals = true;
        }
    }
    AD[2] = A[2]*(D);
    r = fabs(AD[2]);
    r1 = EB[0]*AbsC[2][0] + EB[1]*AbsC[2][1] + EB[2]*AbsC[2][2];
    r01 = EA[2] + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using face normal B[0] as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(B[0]*(D));
    r0 = EA[0]*AbsC[0][0] + EA[1]*AbsC[1][0] + EA[2]*AbsC[2][0];
    r01 = r0 + EB[0];
    if(r > r01+margin)
    {
        return false;
    }

    //Using face normal B[1] as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(B[1]*(D));
    r0 = EA[0]*AbsC[0][1] + EA[1]*AbsC[1][1] + EA[2]*AbsC[2][1];
    r01 = r0 + EB[1];
    if(r > r01+margin)
    {
        return false;
    }

    //Using face normal B[2] as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(B[2]*(D));
    r0 = EA[0]*AbsC[0][2] + EA[1]*AbsC[1][2] + EA[2]*AbsC[2][2];
    r01 = r0 + EB[2];
    if(r > r01+margin)
    {
        return false;
    }

    //if we found parallel face normals, then we do not need to check the edge x edge axes
    if (parallelFaceNormals)
    {
        return true;
    }

    //Check the cases where the axis is a cross product of a face normal of A and B
    //Using A0xB0 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[2]*C[1][0] - AD[1]*C[2][0]);
    r0 = EA[1]*AbsC[2][0] + EA[2]*AbsC[1][0];
    r1 = EB[1]*AbsC[0][2] + EB[2]*AbsC[0][1];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A0xB1 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[2]*C[1][1] - AD[1]*C[2][1]);
    r0 = EA[1]*AbsC[2][1] + EA[2]*AbsC[1][1];
    r1 = EB[0]*AbsC[0][2] + EB[2]*AbsC[0][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A0xB2 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[2]*C[1][2] - AD[1]*C[2][2]);
    r0 = EA[1]*AbsC[2][2] + EA[2]*AbsC[1][2];
    r1 = EB[0]*AbsC[0][1] + EB[1]*AbsC[0][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A1xB0 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[0]*C[2][0] - AD[2]*C[0][0]);
    r0 = EA[0]*AbsC[2][0] + EA[2]*AbsC[0][0];
    r1 = EB[1]*AbsC[1][2] + EB[2]*AbsC[1][1];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A1xB1 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[0]*C[2][1] - AD[2]*C[0][1]);
    r0 = EA[0]*AbsC[2][1] + EA[2]*AbsC[0][1];
    r1 = EB[0]*AbsC[1][2] + EB[2]*AbsC[1][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A1xB2 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[0]*C[2][2] - AD[2]*C[0][2]);
    r0 = EA[0]*AbsC[2][2] + EA[2]*AbsC[0][2];
    r1 = EB[0]*AbsC[1][1] + EB[1]*AbsC[1][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A2xB0 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[1]*C[0][0] - AD[0]*C[1][0]);
    r0 = EA[0]*AbsC[1][0] + EA[1]*AbsC[0][0];
    r1 = EB[1]*AbsC[2][2] + EB[2]*AbsC[2][1];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A2xB1 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[1]*C[0][1] - AD[0]*C[1][1]);
    r0 = EA[0]*AbsC[1][1] + EA[1]*AbsC[0][1];
    r1 = EB[0]*AbsC[2][2] + EB[2]*AbsC[2][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    //Using A2xB2 as axis, so project the distance D = C1 - C0 on this axis
    r = fabs(AD[1]*C[0][2] - AD[0]*C[1][2]);
    r0 = EA[0]*AbsC[1][2] + EA[1]*AbsC[0][2];
    r1 = EB[0]*AbsC[2][1] + EB[1]*AbsC[2][0];
    r01 = r0 + r1;
    if(r > r01+margin)
    {
        return false;
    }

    return true;
}

template <class T>
bool CIntersector2OBB3<T>::Find(T tmax, const CVector3<T> &vel0, const CVector3<T> &vel1)
{

  T tlast = CMath<T>::MAXREAL;
  T tfirst = T(0);
  int iside = -1;
  const T parallelTolerance = T(0.999);//(T)1 - E3;
  int i,j;

  CVector3<T> relVel = vel1 - vel0;

  CVector3<T> vAxis;

  CVector3<T> vertices0[8];
  m_Box0->ComputeVertices(vertices0);

  cfg0.m_dMinOverlap = std::numeric_limits<T>::max();
  cfg1.m_dMinOverlap = std::numeric_limits<T>::max();

  cfg0.m_dMinSeparation = std::numeric_limits<T>::max();
  cfg1.m_dMinSeparation = std::numeric_limits<T>::max();

  //test the normals of box0
  for(i=0;i<3;i++)
  {
    vAxis = m_Box0->m_vUVW[i];
    //if there is no intersection for this axis, we found a separating axis
    if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
      return false;
  }

  //test the normals of box1
  for(i=0;i<3;i++)
  {
    vAxis = m_Box1->m_vUVW[i];
    //if there is no intersection for this axis, we found a separating axis
    if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
      return false;
  }
  
  //cross product of edges
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
    {
      T dotPro = fabs(m_Box0->m_vUVW[i]*m_Box1->m_vUVW[j]);
      if( dotPro > parallelTolerance)
      {
        //we found a pair of parallel axes
        //that means if the face normals did not separate the objects
        //the cross product of edges also will not separate the objects
        //ComputeContactSet()
        
        //easy way out, if the contact will
        //appear take place after
        //the max time
        if(tfirst > tmax)
          return false;

        //the normal can loose its length=1 property
        cfg0.m_vNormal.Normalize();
        cfg1.m_vNormal.Normalize();

        //if the boxes at current time are still separated
        if(tfirst > T(0.0))
        {
          CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,tfirst,m_iQuantity,m_vContacts);
          m_dContactTime = tfirst;
          m_vNormal = cfg0.m_vNormal;
        }
        else  //the boxes are interpenetrating
        {
          CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
          m_vNormal = cfg0.m_vMinNormal;
          cfg0.m_bPenetration = true;
          cfg1.m_bPenetration = true;
        }
        return true;
      }
      //compute the axis and check
      vAxis = CVector3<T>::Cross(m_Box0->m_vUVW[i],m_Box1->m_vUVW[j]);
      if(!CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,relVel,tmax,tfirst,tlast,iside,cfg0,cfg1))
        return false;
    }//end for j
  }//end for i

  if(tfirst <= T(0) || iside == CProjCfg<T>::NONE)
  {
    //std::cerr<<"Penetration detected CIntersector2OBB3<T>::Find"<<std::endl;

    //ComputeContactSet by minimumOverlap
    CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
    m_vNormal = cfg0.m_vMinNormal;
    cfg0.m_bPenetration = true;
    cfg1.m_bPenetration = true;
    return true;
  }

  if(tfirst > tmax)
    return false;

  //compute the contact set
  CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,tfirst,m_iQuantity,m_vContacts);
  m_dContactTime = tfirst;
  m_vNormal = cfg0.m_vNormal;
  return true;
}

template <class T>
bool CIntersector2OBB3<T>::Find(const CVector3<T> &vel0, const CVector3<T> &vel1,
                                const CVector3<T> &vAngVel0, const CVector3<T> &vAngVel1, T deltaT)
{
  T tlast = CMath<T>::MAXREAL;
  T tfirst = T(0);
  int iside = -1;
  const T parallelTolerance = (T)1 - E1;
  //const T parallelTolerance = (T)1 - E4;
  int i,j;

  CVector3<T> relVel = vel1 - vel0;

  CVector3<T> vAxis;

  CVector3<T> vertices0[8];
  m_Box0->ComputeVertices(vertices0);

  cfg0.m_dMinOverlap = std::numeric_limits<T>::max();
  cfg1.m_dMinOverlap = std::numeric_limits<T>::max();

  //we know that there is an intersection
  for(i=0;i<3;i++)
  {
    vAxis = m_Box0->m_vUVW[i];
    CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,cfg0,cfg1);
  }

  //test the normals of box1
  for(i=0;i<3;i++)
  {
    vAxis = m_Box1->m_vUVW[i];
    CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,cfg0,cfg1);
  }
  
  bool parallel = false;
  //cross product of edges
  for(i=0;i<3;i++)
  {
    if(parallel)
      break;
    for(j=0;j<3;j++)
    {
      if(parallel)
        break;
      if(fabs(m_Box0->m_vUVW[i]*m_Box1->m_vUVW[j]) > parallelTolerance)
      {
        //we found a pair of parallel axes
        //that means if the face normals did not separate the objects
        //the cross product of edges also will not separate the objects
        //ComputeContactSet()
        parallel = true;
        continue;
      }

      vAxis = CVector3<T>::Cross(m_Box0->m_vUVW[i],m_Box1->m_vUVW[j]);
      CIntersectorTools<T>::Find(vAxis,*m_Box0,*m_Box1,cfg0,cfg1);
    }//end for j
  }//end for i

  cfg0.m_vMinNormal.Normalize();
  cfg1.m_vMinNormal.Normalize();

  m_bPenetration = true;

  //ComputeContactSet by minimumOverlap
  CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
  m_vNormal = cfg0.m_vMinNormal;

  return true;
}

template <class T>
bool CIntersector2OBB3<T>::Find(T tmax, int nSteps, const CVector3<T> &vel0, const CVector3<T> &vel1,
                                const CVector3<T> &axisAngle0, const CVector3<T> &axisAngle1)
{

  T stepsize = tmax/T(nSteps);

  COBB3<T> Box0(*m_Box0);
  COBB3<T> Box1(*m_Box1);

  COBB3<T> Box2(*m_Box1);
  
  //if(tmax > 0)
  //{
  //  CVector3<T> newCenter1 = Box2.m_vCenter + tmax * vel1;
  //  CVector3<T> diff1      = Box2.m_vCenter - newCenter1;
  //  CVector3<T> angTerm    = CVector3<T>::Cross(axisAngle1,diff1);
  //  CVector3<T> newvel1    = (vel1 + CVector3<T>::Cross(axisAngle1,diff1));

  //  //move the boxes
  //  Box2.m_vCenter += tmax * newvel1;
  //  for(int j=0;j<3;j++)
  //  {
  //    CVector3<T> update = stepsize * CVector3<T>::Cross(axisAngle1,Box2.m_vUVW[j]);
  //    Box2.m_vUVW[j] += update;
  //  }

  //  CVector3<T> vertices[8];
  //  Box2.ComputeVertices(vertices);
  //  std::cout<<""<<std::endl;
  //  for(int k=0;k<8;k++)
  //  {
  //    std::cout<<vertices[k];
  //  }
  //}

  for(int i=1;i<=nSteps;i++)
  {
    T time = stepsize * T(i);
    CVector3<T> newCenter0 = Box0.m_vCenter + time * vel0;
    CVector3<T> newCenter1 = Box1.m_vCenter + time * vel1;
    CVector3<T> diff0      = Box0.m_vCenter - newCenter0;
    CVector3<T> diff1      = Box1.m_vCenter - newCenter1;
    CVector3<T> newvel0    = (vel0 + CVector3<T>::Cross(axisAngle0,diff0));
    CVector3<T> newvel1    = (vel1 + CVector3<T>::Cross(axisAngle1,diff1));
    CVector3<T> vertices[8];
    Box0.ComputeVertices(vertices);
    CIntersector2OBB3<T> intr(Box0,Box1);
    if(intr.Find(stepsize,newvel0,newvel1))
    {
      //std::cout<<intr.cfg0.m_dMinSeparation<<std::endl;
      if(intr.cfg0.m_dMinOverlap < std::numeric_limits<T>::max())
      {
        //std::cout<<intr.cfg0.m_dMinOverlap<<std::endl;
        cfg0=intr.cfg0;
        cfg1=intr.cfg1;
      }
      m_vContacts = intr.GetContacts();
      m_vNormal = intr.GetNormal();
      m_iQuantity = intr.GetQuantity();
      m_dContactTime = intr.GetContactTime();
      return true;
    }

    //move the boxes
    Box0.m_vCenter += stepsize * newvel0;
    Box1.m_vCenter += stepsize * newvel1;

    //Box0.m_vCenter += newvel0;
    //Box1.m_vCenter += newvel1;


    for(int j=0;j<3;j++)
    {
      Box0.m_vUVW[j] += stepsize * CVector3<T>::Cross(axisAngle0,Box0.m_vUVW[j]);

      Box1.m_vUVW[j] += stepsize * CVector3<T>::Cross(axisAngle1,Box1.m_vUVW[j]);
    }


  }


  return false;
}

template <typename T>
void CIntersector2OBB3<T>::ComputeIntervall(CVector3<T> verts[8], const CVector3<T> &axis, T &min, T &max)
{

  min = axis * verts[0];
  max=min;

  for(int i=1;i<8;i++)
  {
    T proj = axis * verts[i];
    if(proj < min)
      min=proj;
    if(proj > max)
      max=proj;
  }

}

template <typename T>
bool CIntersector2OBB3<T>::Test2(const CVector3<T> &vel0, const CVector3<T> &vel1)
{
    //we test the 6 face normals and 9 edge x edge pairs as
    //potential separating axes
    //if we find parallel face normals then the edge x edge axes
    //dont need to be checked

    //we want to detect parallel face normals, so test N_a * N_b == 1
    //to make the test numerically robust, we define a tolerance: fabs(N_a * N_b) > 1 - tolerance
    const T parallelTolerance = (T)1 - E5;
    bool parallelFaceNormals = false;
    //const T margin = E4;
    const T margin = T(0);
    T minA[15],maxA[15],minB[15],maxB[15],alength[15];
    CVector3<T> axis[15];
    int i,j,count;
    int iside = -1;
    count=0;


    //Convenience variables.
    const CVector3<T>* A = m_Box0->m_vUVW;
    const CVector3<T>* B = m_Box1->m_vUVW;
    const T* EA = m_Box0->m_Extents;
    const T* EB = m_Box1->m_Extents;

		CVector3<T> verticesA[8];
		CVector3<T> verticesB[8];

    m_Box0->ComputeVertices(verticesA);
    m_Box1->ComputeVertices(verticesB);

    for(i=0;i<3;i++,count++)
    {
      axis[count]=A[i];
      alength[count]=axis[count].mag();
      ComputeIntervall(verticesA,A[i],minA[count],maxA[count]);
      ComputeIntervall(verticesB,A[i],minB[count],maxB[count]);

      if(!overlap(minA[count],maxA[count],minB[count],maxB[count]))
      {
        return false;
      }

    }

    for(i=0;i<3;i++,count++)
    {
      axis[count]=B[i];
      alength[count]=axis[count].mag();
      ComputeIntervall(verticesA,B[i],minA[count],maxA[count]);
      ComputeIntervall(verticesB,B[i],minB[count],maxB[count]);

      if(!overlap(minA[count],maxA[count],minB[count],maxB[count]))
      {
        return false;
      }

    }

    for(i=0;i<3;i++)
    {
      for(j=0;j<3;j++,count++)
      {
        axis[count]=CVector3<T>::Cross(A[i],B[j]);
        alength[count]=axis[count].mag();
        ComputeIntervall(verticesA,axis[count],minA[count],maxA[count]);
        ComputeIntervall(verticesB,axis[count],minB[count],maxB[count]);

        if(!overlap(minA[count],maxA[count],minB[count],maxB[count]))
        {
          return false;
        }
      }
    }

    // Calculate minimal translation distance (MTD)
    CVector3<T> vNormal;
    T mtd = std::numeric_limits<T>::max();
    T l=0.0;
    int id=0;

    for(i=0;i<15;i++)
    {
      T min = (minA[i] > minB[i]) ? minA[i] : minB[i];
      T max = (maxA[i] < maxB[i]) ? maxA[i] : maxB[i];

      T d = max - min;
      T length = axis[i].mag();

      if(length < CMath<T>::TOLERANCEZERO)
        continue;

      d/=length;
      if(d < mtd)
      {
        mtd=d;
        vNormal = axis[i];
        l=length;
        id=i;
      }

    }

  CProjCfg<T> cfg0;
  CProjCfg<T> cfg1;
  vNormal.Normalize();
  //compute the relative orientation for the axis
  CIntersectorTools<T>::GetProjCfg(vNormal,*m_Box0,cfg0);
  CIntersectorTools<T>::GetProjCfg(vNormal,*m_Box1,cfg1);
  memcpy(&cfg0.m_iMinFeatureIndex,&cfg0.m_iFeatureIndex,8*sizeof(int));
  memcpy(&cfg1.m_iMinFeatureIndex,&cfg1.m_iFeatureIndex,8*sizeof(int));
  cfg0.m_vMinNormal=cfg0.m_vNormal=vNormal;
  cfg1.m_vMinNormal=cfg1.m_vNormal=vNormal;
  cfg0.m_iMinFeature=cfg0.m_iFeature;
  cfg1.m_iMinFeature=cfg1.m_iFeature;
  cfg0.m_dMinOverlap=mtd;
  cfg1.m_dMinOverlap=mtd;

  if(cfg1.m_dMax > cfg0.m_dMax)
  {
    cfg0.m_iMinOrientation = 2;
    cfg1.m_iMinOrientation = 1;
  }
  else
  {
    cfg0.m_iMinOrientation = 1;
    cfg1.m_iMinOrientation = 2;
  }


  CIntersectorTools<T>::ComputeContactSet(*m_Box0,*m_Box1,iside,cfg0,cfg1,vel0,vel1,m_iQuantity,m_vContacts);
  this->m_vNormal = vNormal;
  return true;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersector2OBB3<Real>;

//----------------------------------------------------------------------------

}

/*

void ProjectGeneral()
{
  if the axis is perpendicular to
  two axes of the other box, then the feature for this box is face 

  if the axis is perpendicular to
  just axis of the other box, then the feature for this box is edge

  if the axis is not perpendicular to any of the other boxes axes,
  then the feature for this box is vertex

}

*/

