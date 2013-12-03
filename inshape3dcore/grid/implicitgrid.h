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
class CImplicitGrid {

public: 

  CImplicitGrid();
  
/**
 * @brief Constructor
 * 
 */
  CImplicitGrid(CBasicSpatialHash *pSpatialHash);
  
  CImplicitGrid(CBasicSpatialHash *pSpatialHash, Real cellSize); 

  ~CImplicitGrid(); 

/**
 * @brief Insert a rigid body into the grid 
 *
 * Insert a rigid body into the grid 
 * 
 */  
  void Insert(CRigidBody *body);

  void Insert(CCompoundBody *body);

  void Insert(CSubdomainBoundary *body);

/**
 * @brief Remove a body from the grid 
 *
 * Remove a body from the grid 
 * 
 */  
  void Remove(CRigidBody *body);

/**
 * @brief Removes all stored objects from the grid 
 *
 * Removes all stored objects from the grid so 
 * it can be rebuild.
 *
 */  
  void Clear();

/**
 * @brief Set the cell size of the grid
 *
 */  
  inline void SetCellSize(Real size) {m_dCellSize=size;};
  
  inline Real GetCellSize() const {return m_dCellSize;};
  
/**
 * @brief Returns a pointer to the actual spatial subdivision class
 *
 *  Returns a pointer to the actual spatial subdivision class
 * 
 */    
  inline CBasicSpatialHash* GetSpatialHash() {return m_pSpatialHash;};

private:

  CBasicSpatialHash *m_pSpatialHash;
  Real          m_dCellSize;

};

}
#endif
