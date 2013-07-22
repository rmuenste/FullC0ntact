/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include "distancemap.h"

namespace i3d {

template <class T>
CDistanceMap<T>::CDistanceMap(const CAABB3<T> &aabb)
{
  
  m_bxBox = aabb;
  
  //32x32x32
  m_pVertexCoords = NULL;
  m_pNormals = NULL;
  m_pContactPoints = NULL;
  m_iFBM = NULL;
  
  m_dCellSize = (2.0*m_bxBox.m_Extends[0])/32.0f;
  
  m_iCells[0] = (2.0*m_bxBox.m_Extends[0])/m_dCellSize;
  m_iCells[1] = (2.0*m_bxBox.m_Extends[1])/m_dCellSize;
  m_iCells[2] = (2.0*m_bxBox.m_Extends[2])/m_dCellSize;

  int vx = m_iCells[0]+1;

  int vxy=vx*vx;
  
  int vxyz = vxy*vx;
  
  m_iDim[0]=vx;
  m_iDim[1]=vxy;
  
  m_pVertexCoords = new CVector3<T>[vxyz];
  m_dDistance = new T[vxyz];
  m_iFBM = new int[vxyz];
  m_pNormals = new CVector3<T>[vxyz];
  m_pContactPoints = new CVector3<T>[vxyz];
  
  //generate the vertex coordinates
  for(int k=0;k<vx;k++)
  {  
    for(int j=0;j<vx;j++)
    {
      for(int i=0;i<vx;i++)
      {
        m_pVertexCoords[k*vxy+j*vx+i].x=m_bxBox.m_Verts[0].x+i*m_dCellSize;
        m_pVertexCoords[k*vxy+j*vx+i].y=m_bxBox.m_Verts[0].y+j*m_dCellSize;
        m_pVertexCoords[k*vxy+j*vx+i].z=m_bxBox.m_Verts[0].z+k*m_dCellSize;
      }
    }
  }
}

template <class T>
CDistanceMap<T>::~CDistanceMap()
{
  if(m_pVertexCoords != NULL)
  {
    delete[] m_pVertexCoords;
    m_pVertexCoords = NULL;
  }
  if(m_dDistance != NULL)
  {
    delete[] m_dDistance;
    m_dDistance = NULL;
  }
  if(m_pNormals != NULL)
  {
    delete[] m_pNormals;
    m_pNormals = NULL;
  }
  if(m_pContactPoints != NULL)
  {
    delete[] m_pContactPoints;
    m_pContactPoints = NULL;
  }
  if(m_iFBM != NULL)
  {
    delete[] m_iFBM;
    m_iFBM = NULL;
  }        
  
}

template <class T>
void CDistanceMap<T>::VertexIndices(int icellx,int icelly, int icellz, int indices[8])
{
  int baseIndex=icellz*m_iDim[1]+icelly*m_iDim[0]+icellx; 

  indices[0]=baseIndex;         //xmin,ymin,zmin
  indices[1]=baseIndex+1;       //xmax,ymin,zmin

  indices[2]=baseIndex+m_iDim[0]+1; //xmax,ymax,zmin
  indices[3]=baseIndex+m_iDim[0];   //xmin,ymax,zmin

  
  indices[4]=baseIndex+m_iDim[1];  
  indices[5]=baseIndex+m_iDim[1]+1;  

  indices[6]=baseIndex+m_iDim[0]+m_iDim[1]+1;  
  indices[7]=baseIndex+m_iDim[0]+m_iDim[1];

}

template <class T>
T CDistanceMap<T>::TrilinearInterpolateDistance(const CVector3<T> &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  T x_d= (vQuery.x - m_pVertexCoords[indices[0]].x)/(m_pVertexCoords[indices[1]].x - m_pVertexCoords[indices[0]].x);
  T y_d= (vQuery.y - m_pVertexCoords[indices[0]].y)/(m_pVertexCoords[indices[2]].y - m_pVertexCoords[indices[0]].y);
  T z_d= (vQuery.z - m_pVertexCoords[indices[0]].z)/(m_pVertexCoords[indices[4]].z - m_pVertexCoords[indices[0]].z);  
  
  T c00 = m_dDistance[indices[0]] * (1.0 - x_d) + m_dDistance[indices[1]] * x_d;
  
  T c10 = m_dDistance[indices[3]] * (1.0 - x_d) + m_dDistance[indices[2]] * x_d;
  
  T c01 = m_dDistance[indices[4]] * (1.0 - x_d) + m_dDistance[indices[5]] * x_d;
  
  T c11 = m_dDistance[indices[7]] * (1.0 - x_d) + m_dDistance[indices[6]] * x_d;      
  
  //next step
  T c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  T c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  T c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

template <class T>
CVector3<T> CDistanceMap<T>::TrilinearInterpolateCP(const CVector3<T> &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  T x_d= (vQuery.x - m_pVertexCoords[indices[0]].x)/(m_pVertexCoords[indices[1]].x - m_pVertexCoords[indices[0]].x);
  T y_d= (vQuery.y - m_pVertexCoords[indices[0]].y)/(m_pVertexCoords[indices[2]].y - m_pVertexCoords[indices[0]].y);
  T z_d= (vQuery.z - m_pVertexCoords[indices[0]].z)/(m_pVertexCoords[indices[4]].z - m_pVertexCoords[indices[0]].z);  
  
  CVector3<T> c00 = m_pContactPoints[indices[0]] * (1.0 - x_d) + m_pContactPoints[indices[1]] * x_d;
  
  CVector3<T> c10 = m_pContactPoints[indices[3]] * (1.0 - x_d) + m_pContactPoints[indices[2]] * x_d;
  
  CVector3<T> c01 = m_pContactPoints[indices[4]] * (1.0 - x_d) + m_pContactPoints[indices[5]] * x_d;
  
  CVector3<T> c11 = m_pContactPoints[indices[7]] * (1.0 - x_d) + m_pContactPoints[indices[6]] * x_d;      
  
  //next step
  CVector3<T> c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  CVector3<T> c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  CVector3<T> c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

template <class T>
std::pair<T,CVector3<T> >  CDistanceMap<T>::Query(const CVector3<T> &vQuery)
{
  T dist = T(1000.0);
  CVector3<T> normal;
  CVector3<T> center;
  CVector3<T> origin;  
  std::pair<T,CVector3<T> > res(dist,normal);
  
  if(!m_bxBox.Inside(vQuery))
  {
    return res;
  }
  
  //calculate the cell indices
  T invCellSize = 1.0/m_dCellSize;
  
  //bring to mesh origin
  
  //calculate the cell indices
  int x = (int)(fabs(vQuery.x-m_bxBox.m_Verts[0].x) * invCellSize);
  int y = (int)(fabs(vQuery.y-m_bxBox.m_Verts[0].y) * invCellSize);
  int z = (int)(fabs(vQuery.z-m_bxBox.m_Verts[0].z) * invCellSize);  

//   T invCellSize = 1.0/m_dCellSize;
//   int x = (int)(fabs(origin.x-vQuery.x) * invCellSize);
//   int y = (int)(fabs(origin.y-vQuery.y) * invCellSize);
//   int z = (int)(fabs(origin.z-vQuery.z) * invCellSize);  
  
  //vertex indices
  int indices[8];
  
  //look up distance for the cell
  VertexIndices(x,y,z,indices);
   
  int index=-1;
  T mindist = CMath<T>::MAXREAL;
  dist = 1000.0;
  
//   for(int i=0;i<8;i++)
//   {
//     center+=m_pVertexCoords[indices[i]];
//     //std::cout<<"DistanceMapPoint: "<<m_pVertexCoords[indices[i]];
//     T d = (m_pVertexCoords[indices[i]] - vQuery).mag();
//     if(d < dist)
//     {
//       index=i;
//       dist=d;
//       //normal = m_pNormals[indices[i]];
//       normal = m_pContactPoints[indices[i]];     
//       mindist=m_dDistance[indices[i]];        
//     }  
//   }
  center=center*0.125;
//   std::cout<<"center: "<<center*0.125;
//   std::cout<<"size: "<<m_dCellSize<<std::endl;  
  
  res.first  = TrilinearInterpolateDistance(vQuery,indices);
  res.second = TrilinearInterpolateCP(vQuery,indices);
  
  //res.first=mindist;  
  //res.second=normal;
  
  return res;
}

template <class T>
void CDistanceMap<T>::ConvertToUnstructuredGrid(CUnstrGridr& ugrid)
{
  int NEL=0;
  int NVT=0;
    
  int vx = m_iCells[0]+1;

  int vxy=vx*vx;
  
  int vxyz = vxy*vx;
    
  NVT=vxyz;
  
  NEL=m_iCells[0]*m_iCells[1]*m_iCells[2];
  
  ugrid.m_iNVT          = NVT;
  ugrid.m_iNEL          = NEL;  
  ugrid.m_pVertexCoords = new VECTOR3[NVT];
  ugrid.m_pHexas        = new CHexa[NEL];
  ugrid.m_myTraits      = new DTraits[NVT];
  
  //needed vertexcoords,hexas,traits
  int ive=0;

  
  //start with the highest level
  for(ive=0;ive<NVT;ive++)
  {
    ugrid.m_pVertexCoords[ive]=m_pVertexCoords[ive];
    ugrid.m_myTraits[ive].distance=m_dDistance[ive];
    ugrid.m_myTraits[ive].iTag=1;
  }//end for  

  int iel=0;
  for(int ielz=0;ielz<m_iCells[2];ielz++)
    for(int iely=0;iely<m_iCells[1];iely++)    
      for(int ielx=0;ielx<m_iCells[0];ielx++)
      {
        VertexIndices(ielx,iely,ielz,ugrid.m_pHexas[iel++].m_iVertInd);    
      }//end for    

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceMap<Real>;

//----------------------------------------------------------------------------

}

