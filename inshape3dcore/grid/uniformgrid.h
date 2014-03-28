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

class RigidBody;


class ElementCell
{
public:

	ElementCell(){};

	~ElementCell(){};

	void Insert(int iel){m_lElements.push_back(iel);};
	
	std::list<int> m_lElements;

};

/**
 * @brief The class implements a uniform grid data structure
 * 
 */
template<class T, class CellType>
class UniformGrid
{
public:

  UniformGrid();
  
/**
 * @brief Construct a HUniformGrid from a boundingBox
 *
 * Construct a HUniformGrid from a boundingBox  
 *  
 */  
  UniformGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element);  

  ~UniformGrid();

/**
 * @brief Inits a new grid from a certain element size
 *  
 *  Inits a new grid from a certain element size
 * 
 */  
  void InitGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element);

/**
 * @brief Inits a new grid level with a given cellSize 
 *  
 *  Inits a new grid level with a given cellSize 
 * 
 */  
  void InitGrid(const CAABB3<T> &boundingBox, T cellSize);

/**
 * @brief Checks which cells of the grid are intersected by the body's bounding box 
 *
 * Checks which cells of the grid are intersected by the body's bounding box. As 
 * a side effect these cells are stored in the body's cell list. 
 * 
 */  
  void Query(RigidBody *body);
  
/**
 * @brief Checks in which cell a point is located 
 * 
 * The function checks in which cell the query point is located. Cells 
 * of the uniform grid store objects based on their center point and bounding sphere 
 * radius only because of this the elemlist returned contains the objects contained in the 
 * cell that contains the point AND the objects from neighboring cells that could potentially 
 * intersect the cell where the query point is located. 
 * 
 */  
  void PointQuery(const CVector3<T> &q, std::list<int> &elemlist);
  
/**
 * @brief Resets the grid, so it can be rebuild with different parameters
 *
 * Resets the grid, so it can be rebuild with different parameters  
 *
 */    
  void Reset();

/**
 * @brief Inserts an element with a certain integer id into the grid 
 * 
 * The element gets inserted into the grid depending on the coordinate 
 * of its center point. The inserted key is the element's id.
 * 
 */  
  void Insert(int ielementID, const CVector3<T> &center);

/**
 * @brief Removes an element from the grid
 * 
 * Removes an element from the grid
 * 
 */  
  void Remove();
    
  inline int GetNumEntries(){return m_iTotalEntries;};
  
  // boundarybox
  CAABB3<T> m_bxBox;
  
  // dimension
  int m_iDimension[3];

  // cell size
  T m_dCellSize;

  CellType *m_pCells;

  int m_iTotalEntries;

};

}
#endif
