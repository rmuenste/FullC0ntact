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
#ifndef IMPLICITGRID_H
#define IMPLICITGRID_H



//===================================================
//                     INCLUDES
//===================================================
#include <spatialhash.h>
#include <rigidbody.h>
#include <compoundbody.h>
#include <subdomainboundary.h>

namespace i3d {

/**
* @brief This class acts as an interface to a spatial subdivision class that can be used in a broadphase algorithm
* 
* This class acts as an interface to a spatial subdivision class that can be used in a broadphase algorithm
* 
*/
class ImplicitGrid {

public: 

  ImplicitGrid();
  
/**
 * @brief Constructor
 * 
 */
  ImplicitGrid(BasicSpatialHash *spatialHash);
  
  ImplicitGrid(BasicSpatialHash *spatialHash, Real cellSize); 

  ~ImplicitGrid(); 

/**
 * @brief Insert a rigid body into the grid 
 *
 * Insert a rigid body into the grid 
 * 
 */  
  void addObject(RigidBody *body);

  void Insert(CompoundBody *body);

  void Insert(SubdomainBoundary *body);

/**
 * @brief Remove a body from the grid 
 *
 * Remove a body from the grid 
 * 
 */  
  void removeObject(RigidBody *body);

/**
 * @brief Removes all stored objects from the grid 
 *
 * Removes all stored objects from the grid so 
 * it can be rebuild.
 *
 */  
  void clear();

/**
 * @brief Set the cell size of the grid
 *
 */  
  inline void setCellSize(Real size) {cellSize_=size;};
  
  inline Real getCellSize() const {return cellSize_;};
  
/**
 * @brief Returns a pointer to the actual spatial subdivision class
 *
 *  Returns a pointer to the actual spatial subdivision class
 * 
 */    
  inline BasicSpatialHash* getSpatialHash() {return spatialHash_;};

private:

  BasicSpatialHash *spatialHash_;
  Real               cellSize_;

};

}
#endif
