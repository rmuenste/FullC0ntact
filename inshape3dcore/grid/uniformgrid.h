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

class CRigidBody;


class CUGCell
{
public:

	CUGCell(){};

	~CUGCell(){};

	void Insert(int iel){m_lElements.push_back(iel);};
	
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

  CUniformGrid();
  
  CUniformGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element);  

  ~CUniformGrid();

  void InitGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element);

  void InitGrid(const CAABB3<T> &boundingBox, T cellSize);

  // PointQuery  
  void Query(CRigidBody *body);
  
  // boundarybox
  CAABB3<T> m_bxBox;
  
  // dimension
  int m_iDimension[3];

  // cell size
  T m_dCellSize;

  // Insert
  void Insert(int ielementID, const CVector3<T> &center);

  // Remove
  void Remove();

  CellType *m_pCells;

};

}
#endif
