//===================================================
//                     INCLUDES
//===================================================
#include "uniformgrid.h"
#include <mymath.h>
#include <aabb3.h>
#include <stdio.h>
#include <stdlib.h>

namespace i3d {

template<class T, class CellType>
CUniformGrid<T,CellType>::CUniformGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element)
{
  
  m_dCellSize = 2.0 * element.GetBoundingSphereRadius();
  
  int x = (m_bxBox.m_Extends[0]+m_dCellSize)/m_dCellSize;
  int y = (m_bxBox.m_Extends[1]+m_dCellSize)/m_dCellSize;
  int z = (m_bxBox.m_Extends[2]+m_dCellSize)/m_dCellSize;

  int xy = x*y;
  int xz = x*z;
  int yz = y*z;

  // int nGhostLayerCells = (2 * xy + 2 * xz + 2 * yz) + (4 * x + 4 * y + 4 * z) + 8;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];

  //GhostCell version
  //m_pCells = new CellType[x*y*z + nGhostLayerCells];

}

template<class T, class CellType>
void CUniformGrid<T,CellType>::InitGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element)
{
   
  m_dCellSize = 2.0 * element.GetBoundingSphereRadius();
  printf("cell size: %f\n",m_dCellSize);
  printf("extends : %f %f %f\n",boundingBox.m_Extends[0],boundingBox.m_Extends[1],boundingBox.m_Extends[2]);  
  m_bxBox = boundingBox;
    
  int x = (2.0*m_bxBox.m_Extends[0]+m_dCellSize)/m_dCellSize;
  int y = (2.0*m_bxBox.m_Extends[1]+m_dCellSize)/m_dCellSize;
  int z = (2.0*m_bxBox.m_Extends[2]+m_dCellSize)/m_dCellSize;

  // int xy = x*y;
  // int xz = x*z;
  // int yz = y*z;
  
  // int nGhostLayerCells = (2 * xy + 2 * xz + 2 * yz) + (4 * x + 4 * y + 4 * z) + 8;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];
  
  m_iDimension[0] = x;
  m_iDimension[1] = y;
  m_iDimension[2] = z;
    
}

template<class T, class CellType>
void CUniformGrid<T,CellType>::Insert(int elementID, const CVector3<T> &center)
{
  
  CVector3<T> origin(m_bxBox.m_vCenter.x-m_bxBox.m_Extends[0],
		     m_bxBox.m_vCenter.y-m_bxBox.m_Extends[1],
		     m_bxBox.m_vCenter.z-m_bxBox.m_Extends[2]);
  
  // calculate the cell index in xyz notation
  T invCellSize = 1.0/m_dCellSize;
  int indexx = (int)(fabs(origin.x-center.x) * invCellSize);
  int indexy = (int)(fabs(origin.y-center.y) * invCellSize);
  int indexz = (int)(fabs(origin.z-center.z) * invCellSize);
  
  // convert to linear array index
  int index = indexz * m_iDimension[0]*m_iDimension[1] +
              indexy * m_iDimension[0] + indexx;
  
  if(index > m_iDimension[0]*m_iDimension[1]*m_iDimension[2])
  {
    printf("Index out of bounds: %d %d %d %d\n",index,indexx,indexy,indexz);
    printf("dimensions: %d %d %d %d\n",m_iDimension[0]*m_iDimension[1]*m_iDimension[2],
	                               m_iDimension[0],m_iDimension[1],m_iDimension[2]);
    printf("origin: %f %f %f\n",origin.x,origin.y,origin.z);
    printf("center: %f %f %f\n",center.x,center.y,center.z);    
    //printf("");
    exit(0);
  }
  
  m_pCells[index].Insert(elementID);
	      
}

template<class T, class CellType>
CUniformGrid<T,CellType>::~CUniformGrid()
{

  if(m_pCells != NULL)
  {
    delete[] m_pCells;
    m_pCells = NULL;
  }

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CUniformGrid<Real,CUGCell>;

//----------------------------------------------------------------------------
}
