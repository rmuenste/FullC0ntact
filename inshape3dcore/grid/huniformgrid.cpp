//===================================================
//                     INCLUDES
//===================================================
#include "huniformgrid.h"
#include <mymath.h>
#include <aabb3.h>
#include <stdio.h>
#include <stdlib.h>
#include <rigidbody.h>

namespace i3d {

template<class T, class CellType>
CHUniformGrid<T,CellType>::CHUniformGrid(const CAABB3<T> &boundingBox, int levels)
{
  
  m_bxBox = boundingBox;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pLevels = new CUniformGrid<T,CellType>[levels];

  m_iLevels = levels;

}

template<class T, class CellType>
CHUniformGrid<T,CellType>::CHUniformGrid()
{
  
  m_iLevels = 0;

}

template<class T, class CellType>
void CHUniformGrid<T,CellType>::InitGrid(const CAABB3<T> &boundingBox, int levels)
{

  m_bxBox = boundingBox;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pLevels = new CUniformGrid<T,CellType>[levels];

  m_iLevels = levels;   

}

template<class T, class CellType>
void CHUniformGrid<T,CellType>::InitGridLevel(int level, T cellSize)
{
  
  m_pLevels[level].InitGrid(this->m_bxBox,cellSize);

}

template<class T, class CellType>
void CHUniformGrid<T,CellType>::InsertElements(std::list< std::pair<double,int> > &elementList, CUnstructuredGrid<T, DTraits> &grid)
{

	std::list< std::pair<double,int> >::iterator liter = elementList.begin();  
  
  for(;liter!=elementList.end();liter++)
  {

    //determine at which level we should insert
    T size = (*liter).first;
    int level=0;
    while(size > m_pLevels[level].m_dCellSize)
    {
      level++;
    }

    CHexa &hexa = grid.m_pHexas[(*liter).second];
    CUnstructuredGrid<T, DTraits>::VertElemIter ive = grid.VertElemBegin(&hexa);
    CVector3<T> center(0,0,0);
    for(;ive!=grid.VertElemEnd(&hexa);ive++)
    {
      center+=(*ive);
    }    
    center*=0.125;
    m_pLevels[level].Insert((*liter).second, center);

  }

}

template<class T, class CellType>
CHUniformGrid<T,CellType>::~CHUniformGrid()
{

  if(m_pLevels != NULL)
  {
    delete[] m_pLevels;
    m_pLevels = NULL;
  }

}

template<class T, class CellType>
void CHUniformGrid<T,CellType>::Query(CRigidBody *body)
{
 
  for(int i=0;i<m_iLevels;i++)
  {
    m_pLevels[i].Query(body);
  }

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CHUniformGrid<Real,CUGCell>;

//----------------------------------------------------------------------------
}
