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
  
  m_bBox = aabb;
  
  //32x32x32
  
  m_dCellSize = m_bBox.m_Extends[0]/32.0f;
  
  int x = (m_bxBox.m_Extends[0])/m_dCellSize;
  int y = (m_bxBox.m_Extends[1])/m_dCellSize;
  int z = (m_bxBox.m_Extends[2])/m_dCellSize;

  int vx = x+1;

  int vxy=vx*vx;
  
  int vxyz = vxy*vx;
  
  m_pVertexCoords = new CVector3<T>[vxyz];

  // pass a bounding box that is m_dCellSize bigger in each dimension
  // m_pCells = new CellType[x*y*z];

//   m_iTotalEntries=0;
//   
//   
//   // the total number of vertices
//   m_iNVT = 8;
// 
//   // the total number of elements
//   m_iNEL = 1;
// 
//   this->m_pVertexCoords = new CVector3<T>[m_iNVT+1];
//   m_piVertAtBdr         = new int[m_iNVT];  
//   m_pHexas              = new CHexa[1];
// 
//   m_myTraits            = new Traits[m_iNVT];
//   
//   m_vMax=aabb.m_Verts[1];
//   m_vMin=aabb.m_Verts[0];
// 
//   m_pVertexCoords[0] = m_vMin;                                             
//   m_pVertexCoords[1] = CVector3<T>(m_vMax.x,m_vMin.y,m_vMin.z);            
//   m_pVertexCoords[2] = CVector3<T>(m_vMax.x,m_vMax.y,m_vMin.z);            
//   m_pVertexCoords[3] = CVector3<T>(m_vMin.x,m_vMax.y,m_vMin.z);            
//   m_pVertexCoords[4] = CVector3<T>(m_vMin.x,m_vMin.y,m_vMax.z);            
//   m_pVertexCoords[5] = CVector3<T>(m_vMax.x,m_vMin.y,m_vMax.z);            
//   m_pVertexCoords[6] = CVector3<T>(m_vMax.x,m_vMax.y,m_vMax.z);            
//   m_pVertexCoords[7] = CVector3<T>(m_vMin.x,m_vMax.y,m_vMax.z);            
//   m_piVertAtBdr[0]   = 0;
//   m_piVertAtBdr[1]   = 1;
//   m_piVertAtBdr[2]   = 2;
//   m_piVertAtBdr[3]   = 3;
//   m_piVertAtBdr[4]   = 4;
//   m_piVertAtBdr[5]   = 5;
//   m_piVertAtBdr[6]   = 6;
//   m_piVertAtBdr[7]   = 7;
// 
//   m_pHexas[0].m_iVertInd[0]      = 0;
//   m_pHexas[0].m_iVertInd[1]      = 1;
//   m_pHexas[0].m_iVertInd[2]      = 2;
//   m_pHexas[0].m_iVertInd[3]      = 3;
//   m_pHexas[0].m_iVertInd[4]      = 4;
//   m_pHexas[0].m_iVertInd[5]      = 5;
//   m_pHexas[0].m_iVertInd[6]      = 6;
//   m_pHexas[0].m_iVertInd[7]      = 7;
//     
  
  
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceMap<Real>;

//----------------------------------------------------------------------------

}

