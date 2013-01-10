//===================================================
//                     INCLUDES
//===================================================
#include "uniformgrid.h"
#include <mymath.h>
#include <aabb3.h>

namespace i3d {

template<class T, class CellType>
CUniformGrid<T,CellType>::CUniformGrid(CAABB3<T> boundingBox, CAABB3<T> element)
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
void CUniformGrid<T,CellType>::InitGrid(CAABB3<T> boundingBox, CAABB3<T> element)
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
