//===================================================
//                     INCLUDES
//===================================================
#include "uniformgrid.h"
#include <mymath.h>

namespace i3d {

template<class T>
CUniformGrid<T>::CUniformGrid()
{
  int x = m_bxBox.m_Extends[0]/m_dCellSize;
  int y = m_bxBox.m_Extends[1]/m_dCellSize;
  int z = m_bxBox.m_Extends[2]/m_dCellSize;

  int xy = x*y;
  int xz = x*z;
  int yz = y*z;

  int nGhostLayerCells = (2 * xy + 2 * xz + 2 * yz) + (4 * x + 4 * y + 4 * z) + 8;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];

  //GhostCell version
  //m_pCells = new CellType[x*y*z + nGhostLayerCells];

}

template<class T>
CUniformGrid<T>::~CUniformGrid()
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
template class CUniformGrid<Real>;

//----------------------------------------------------------------------------
}
