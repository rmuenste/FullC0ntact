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
UniformGridHierarchy<T,CellType>::UniformGridHierarchy(const AABB3<T> &boundingBox, int levels)
{
  
  boundingBox_ = boundingBox;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  levels_ = new UniformGrid<T,CellType>[levels];

  nLevels_ = levels;

}

template<class T, class CellType>
UniformGridHierarchy<T,CellType>::UniformGridHierarchy()
{
  
  nLevels_ = 0;

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::initGrid(const AABB3<T> &boundingBox, int levels)
{

  boundingBox_ = boundingBox;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  levels_ = new UniformGrid<T,CellType>[levels];

  nLevels_ = levels;   

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::initGridLevel(int level, T cellSize)
{
  
  levels_[level].InitGrid(this->boundingBox_,cellSize);

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::insertElements(std::list< std::pair<double,int> > &elementList, UnstructuredGrid<T, DTraits> &grid)
{

	std::list< std::pair<double,int> >::iterator liter = elementList.begin();  
  
  for(;liter!=elementList.end();liter++)
  {

    //determine at which level we should insert
    T size = (*liter).first;
    int level=0;
    while(size > levels_[level].m_dCellSize)
    {
      level++;
    }

    Hexa &hexa = grid.hexas_[(*liter).second];
    typename UnstructuredGrid<T, DTraits>::VertElemIter ive = grid.VertElemBegin(&hexa);
    Vector3<T> center(0,0,0);
    for(;ive!=grid.VertElemEnd(&hexa);ive++)
    {
       center+=(*ive);
    }    
    center*=0.125;
    levels_[level].Insert((*liter).second, center);
  }

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::insertElement(int iel, const Vector3<T> &center, T size)
{
  //determine at which level we should insert 
  int level=0;
  while(size > levels_[level].m_dCellSize)
    {
      level++;
    }

  levels_[level].Insert(iel,center);
}

template<class T, class CellType>
UniformGridHierarchy<T,CellType>::~UniformGridHierarchy()
{

  if(levels_ != NULL)
  {
    delete[] levels_;
    levels_ = NULL;
  }

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::reset()
{

  if(levels_ != NULL)
  {
    delete[] levels_;
    levels_ = NULL;
  }
  
  nLevels_ = 0;  

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::query(RigidBody *body)
{
 
  for(int i=0;i<nLevels_;i++)
  {
    levels_[i].Query(body);
  }

}

template<class T, class CellType>
void UniformGridHierarchy<T,CellType>::pointQuery(const Vector3<T> &q, std::list<int> &elemlist)
{
 
  for(int i=0;i<nLevels_;i++)
  {
    levels_[i].PointQuery(q,elemlist);
  }

}  

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class UniformGridHierarchy<Real,ElementCell>;

//----------------------------------------------------------------------------
}
