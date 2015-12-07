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
#include <unstructuredgrid.h>

namespace i3d {

class RigidBody;

/**
 * @brief The class implements a uniform grid data structure
 * 
 */
template<class T, class CellType, class Traits>
class UniformGridHierarchy
{
public:

  UniformGridHierarchy();
  
  /**
  * @brief Construct a HUniformGrid from a boundingBox
  *
  * Construct a HUniformGrid from a boundingBox  
  *  
  */
  UniformGridHierarchy(const AABB3<T> &boundingBox, int levels);  

  ~UniformGridHierarchy();

  /**
  * @brief Inits a new grid level with a given cellSize 
  *  
  *  Inits a new grid level with a given cellSize 
  */
  void initGridLevel(int level, T cellSize);

  /**
  * @brief Inserts an element 
  * 
  */
  void insertElements(std::list< std::pair<double,int> > &elementList, UnstructuredGrid<T, DTraits> &grid);

  /**
  * @brief Initializes a HUniformGrid from a bounding box
  *
  * Initializes a HUniformGrid that has been declared already, but 
  * not yet initialized.
  * 
  */
  void initGrid(const AABB3<T> &boundingBox, int levels);

  /**
  * @brief Inserts an element with a certain number into the grid 
  * 
  * The element gets inserted into the grid depending on the coordinate 
  * of its center point. The inserted key is the element's number.
  * 
  */  
  void insertElement(int iel, const Vector3<T> &center, T size);

  /**
  * @brief Checks which cells of the grid are intersected by the body's bounding box 
  *
  * Checks which cells of the grid are intersected by the body's bounding box. As 
  * a side effect these cells are stored in the body's cell list. 
  * 
  */   
  void query(RigidBody *body);
  
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
  void pointQuery(const Vector3<T> &q, std::list<int> &elemlist);  
  
  /**
  * @brief Resets the grid, so it can be rebuild with different parameters
  *
  * Resets the grid, so it can be rebuild with different parameters  
  *
  */  
  void reset();  
  
  /**
   * Bounding box of the grid
   */
  AABB3<T> boundingBox_;

  /**
   * Number of levels in the grid
   */
  int nLevels_;
  
  /**
   * Pointer to the particular levels in the grid
   */
  UniformGrid<T,CellType> *levels_;
  

};

}
#endif
