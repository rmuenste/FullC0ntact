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
#ifndef UNIFORMGRID_H
#define UNIFORMGRID_H

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>

namespace i3d {


class CUGCell
{
public:

	CUGCell(){};

	~CUGCell(){};

	std::list<int> m_lElements;

};

/**
 * @brief The class implements a uniform grid data structure
 * 
 */
template<class T, class CellType>
class CUniformGrid
{
public:

  CUniformGrid(){};
  
  CUniformGrid(CAABB3<T> boundingBox, CAABB3<T> element);  

  ~CUniformGrid();

  void InitGrid(CAABB3<T> boundingBox, CAABB3<T> element);    
  
  // boundarybox
  CAABB3<T> m_bxBox;

  // PointQuery

  // cell size
  T m_dCellSize;

  // neighborIterator

  // ghostCellLayer

  // Insert
  void Insert(CVector3<T> center, int ielementID);

  // Remove
  void Remove();

  // grid
  // grid size: cells.x * cells.y * cells.z
  // mapXYZ2Array: z * cells.x * cells.y + y * cells.x + x

  CellType *m_pCells;

};

}
#endif
