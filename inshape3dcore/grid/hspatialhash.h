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
#ifndef HSPATIALHASH_H
#define HSPATIALHASH_H

#define MAX_LEVELS_HGRID 5

//===================================================
//                     INCLUDES
//===================================================
#include<spatialhash.h>
#include <list>
#include <unstructuredgrid.h>
#include <simplespatialhash.h>

namespace i3d {
  
/**
 * @brief The class implements the hierarchical spatial hash data structure
 */
class CHSpatialHash : public CBasicSpatialHash {

public: 

  CHSpatialHash(); 

  ~CHSpatialHash();

  /**
   * Starts the initialization of the hierarchical grid
   */
  void InitHGrid();
  
  CHSpatialHash(int ncells); 

  CHSpatialHash(int ncells, Real dim[6], std::vector<CRigidBody*> &vRigidBodies);

/**
 * @brief Insert a new element in the CSpatialHashEntry into the grid
 *
 * Insert a new element in the CSpatialHashEntry into the grid
 * 
 */  
  void Insert(CSpatialHashEntry &e);
  
/**
 * @brief Remove a certain cell from the grid
 * 
 * Remove a certain cell from the grid 
 *
 */  
  void Remove(const CCellCoords &cell);

/**
 * @brief Remove all entries from the grid so it can be rebuild with new parameters
 *
 * Remove all entries from the grid so it can be rebuild with new parameters 
 * 
 */  
  void Clear();

/**
 * @brief Returns whether a cell is empty or not
 * 
 * Returns whether a cell is empty or not 
 *
 */  
  bool IsEmpty(const CCellCoords &cell);

/**
 * @brief Returns a vector of all entries that are contained in a cell
 *  
 * Returns a vector of all entries that are contained in a cell
 * 
 */
  std::vector<CSpatialHashEntry> *GetCellEntries(CCellCoords &cell);  
  
/**
 * @brief Convert the grid to a vtk unstructuredgrid for output purposes 
 *
 * Convert the grid to a vtk unstructuredgrid for output purposes 
 * 
 */  
  void ConvertToUnstructuredGrid(CUnstrGridr &ugrid);
  
/**
 * @brief Returns the number of cells that are not empty
 *
 *  Returns the number of cells that are not empty
 *
 */  
  int GetUsedCells(int level) {return m_pLevels[level]->GetUsedCells();};

/**
 * @brief Returns the total number of cells
 *
 *  Returns the total number of cells
 *
 */    
  int GetNCells() {return m_iNCells;};

/**
 * @brief Returns the grid size of the level
 *
 * Returns the grid size of the level
 *
 */      
  Real GetGridSize(int level) {return m_pLevels[level]->GetCellSize();};

/**
 * @brief Returns the index of the maximum level
 *
 * Returns the index of the maximum level
 *
 */        
  int GetMaxLevel(){return (m_iMaxLevel-1);}

/**
 * @brief Returns the index of the minimum level
 *
 * Returns the index of the minimum level
 *
 */          
  int GetMinLevel(){return 0;}

/**
 * @brief Returns a CCellCoords object on the given level for the given position
 *
 * Returns a CCellCoords object on the given level for the given position. 
 * The CCellCoords object stores information about the cell location of the query point
 * in the grid.
 * 
 */          

  inline CCellCoords GetCell(const VECTOR3 &p, int level)
  {
    //calculate the cell indices
    Real invCellSize = (Real)1.0/m_pLevels[level]->GetCellSize();
    
    CCellCoords cell;
    
    //calculate the cell indices
    cell.x = (int)(p.x * invCellSize);
    cell.y = (int)(p.y * invCellSize);
    cell.z = (int)(p.z * invCellSize);
    cell.level = level;    
    return cell;
  }
  
  /**
   * Returns whether this level is a special 'boundary leve'
   */
  bool IsBoundaryLevel(int iLevel)
  {
    if(iLevel < MAX_LEVELS_HGRID)
    {
      return m_bIsBoundaryLevel[iLevel];
    }
    else
      return false;
  }
  
  /**
   * Returns whether a certain cell is a special 'boundary cell'
   */
  inline bool IsBoundary(const CCellCoords &cell)
  {
    //return (cell.x==0 || cell.y==0 || cell.z==0 || cell.x==m_pLevels[cell.level]->GetMaxX() m_iMaxIndices[cell.level][0] || cell.y==m_iMaxIndices[cell.level][1] || cell.z==m_iMaxIndices[cell.level][2]);
    return m_pLevels[cell.level]->IsBoundary(cell);
  }
  
  /**
   * Returns a level of the hierarchy as a pointer to a SimpleSpatialHash
   */
  CSimpleSpatialHash* GetGridLevel(int level) {return m_pLevels[level];};

private:

  /**
   * Compute the hashfunction
   */
  int hash(int x, int y, int z);
  
  /**
   * Estimate appropriate grid cell sizes for a collection of rigid bodies
   */
  void EstimateCellSize(std::vector<CRigidBody*> &vRigidBodies);

  /**
   * Constant used in hashfunction
   */
  const static int m_iPrime1 = 73856093;
  /**
   * Constant used in hashfunction
   */  
  const static int m_iPrime2 = 19349663;
  /**
   * Constant used in hashfunction
   */  
  const static int m_iPrime3 = 83492791;

  /**
   * List of cell sizes of the particular grid levels
   */
  std::list<Real> lSizes;
  
  /**
   * Number of cells
   */
  int  m_iNCells;  
  
  /**
   * Maximum number of levels
   */
  int  m_iMaxLevel;  
  
  /**
   * Boolean that stores whether the grid sizes have been initialized
   */
  bool m_bSizeInitialized;  
  
  /**
   * Boolean array that stores the special property of the levels
   */
  bool m_bIsBoundaryLevel[MAX_LEVELS_HGRID];  
  
  /**
   * Extents of the grid
   */
  Real m_dBoundaryExtents[6];    
  
  /**
   * Pointer to the levels of the hierarchical grid
   */
  CSimpleSpatialHash *m_pLevels[MAX_LEVELS_HGRID];
    
};

}
#endif
