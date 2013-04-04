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
#ifndef HUNIFORMGRID_H
#define HUNIFORMGRID_H

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <uniformgrid.h>
#include <list>
#include <unstructuredGrid.h>

namespace i3d {

class CRigidBody;

/**
 * @brief The class implements a uniform grid data structure
 * 
 */
template<class T, class CellType>
class CHUniformGrid
{
public:

  CHUniformGrid();
  
  CHUniformGrid(const CAABB3<T> &boundingBox, int levels);  

  ~CHUniformGrid();

  void InitGridLevel(int level, T cellSize);

  // Insert
  void InsertElements(std::list< std::pair<double,int> > &elementList, CUnstructuredGrid<T, DTraits> &grid);

  void InitGrid(const CAABB3<T> &boundingBox, int levels);

  // PointQuery  
  void Query(CRigidBody *body);
  
  // boundarybox
  CAABB3<T> m_bxBox;

  int m_iLevels;

  CUniformGrid<T,CellType> *m_pLevels;
  

};

}
#endif
