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



//===================================================
//                     INCLUDES
//===================================================
#include <spatialhash.h>
#include <list>
#include <unstructuredgrid.h>
#include <simplespatialhash.h>

namespace i3d {
  
/**
 * @brief The class implements the hierarchical spatial hash data structure
 */
class SpatialHashHierarchy : public BasicSpatialHash {

public: 

  SpatialHashHierarchy(); 

  ~SpatialHashHierarchy();

  /**
   * Starts the initialization of the hierarchical grid
   */
  void initHierachicalGrid();
  
  SpatialHashHierarchy(int ncells); 

  SpatialHashHierarchy(int ncells, Real dim[6], std::vector<RigidBody*> &vRigidBodies);

/**
 * @brief Insert a new element in the CSpatialHashEntry into the grid
 *
 * Insert a new element in the CSpatialHashEntry into the grid
 * 
 */  
  void insert(CSpatialHashEntry &e) override;
  
  void insert(SubdomainBoundary *body) override
  {
    throw std::logic_error("Class SpatialHashHierarchy: Function not implemented");
  }

  void insert(CompoundBody *body) override
  {
	throw std::logic_error("Class SpatialHashHierarchy: Function not implemented");
  }

/**
 * @brief Remove a certain cell from the grid
 * 
 * Remove a certain cell from the grid 
 *
 */  
  void remove(const CellCoords &cell);

/**
 * @brief Remove all entries from the grid so it can be rebuild with new parameters
 *
 * Remove all entries from the grid so it can be rebuild with new parameters 
 * 
 */  
  void clear();

/**
 * @brief Returns whether a cell is empty or not
 * 
 * Returns whether a cell is empty or not 
 *
 */  
  bool isEmpty(const CellCoords &cell);

/**
 * @brief Returns a vector of all entries that are contained in a cell
 *  
 * Returns a vector of all entries that are contained in a cell
 * 
 */
  std::vector<CSpatialHashEntry> *getCellEntries(CellCoords &cell);  
  
/**
 * @brief Convert the grid to a vtk unstructuredgrid for output purposes 
 *
 * Convert the grid to a vtk unstructuredgrid for output purposes 
 * 
 */  
  void convertToUnstructuredGrid(CUnstrGridr &ugrid) override;
  
/**
 * @brief Returns the number of cells that are not empty
 *
 *  Returns the number of cells that are not empty
 *
 */  
  int getUsedCells(int level) {return levels_[level]->GetUsedCells();};

/**
 * @brief Returns the total number of cells
 *
 *  Returns the total number of cells
 *
 */    
  int getNCells() {return nCells_;};

/**
 * @brief Returns the grid size of the level
 *
 * Returns the grid size of the level
 *
 */      
  Real getGridSize(int level) {return levels_[level]->getCellSize();};

/**
 * @brief Returns the index of the maximum level
 *
 * Returns the index of the maximum level
 *
 */        
  int getMaxLevel(){return (maxLevel_-1);}

/**
 * @brief Returns the index of the minimum level
 *
 * Returns the index of the minimum level
 *
 */          
  int getMinLevel(){return 0;}

/**
 * @brief Returns a CCellCoords object on the given level for the given position
 *
 * Returns a CCellCoords object on the given level for the given position. 
 * The CCellCoords object stores information about the cell location of the query point
 * in the grid.
 * 
 */          

  inline CellCoords getCell(const VECTOR3 &p, int level)
  {
    //calculate the cell indices
    Real invCellSize = (Real)1.0/levels_[level]->getCellSize();
    
    return CellCoords((int)(p.x * invCellSize),
                      (int)(p.x * invCellSize),
                      (int)(p.x * invCellSize),
                      level);
  }
  
  /**
   * Returns whether this level is a special 'boundary leve'
   */
  bool isBoundaryLevel(int iLevel)
  {
    if(iLevel < MAX_LEVELS_HGRID)
    {
      return boundaryLevel_[iLevel];
    }
    else
      return false;
  }
  
  /**
   * Returns whether a certain cell is a special 'boundary cell'
   */
  inline bool isBoundary(const CellCoords &cell)
  {
    //return (cell.x==0 || cell.y==0 || cell.z==0 || cell.x==m_pLevels[cell.level]->GetMaxX() m_iMaxIndices[cell.level][0] || cell.y==m_iMaxIndices[cell.level][1] || cell.z==m_iMaxIndices[cell.level][2]);
    return levels_[cell.level]->isBoundary(cell);
  }
  
  /**
   * Returns a level of the hierarchy as a pointer to a SimpleSpatialHash
   */
  SimpleSpatialHash* getGridLevel(int level) {return levels_[level];};

  void printInfo()
  {
    std::cout<<"--------------------"<<std::endl;
    std::cout<<"HGrid Statistics: "<<std::endl;
    std::cout<<"HGrid Levels: "<<maxLevel_<<std::endl;
    for(int level=0;level<maxLevel_;level++)
    {
      std::cout<<"Size Level "<<level<<" : "<<levels_[level]->getCellSize()<<std::endl;    
    }
    std::cout<<"--------------------"<<std::endl;  
  }

private:

  /**
   * Compute the hashfunction
   */
  int hash(int x, int y, int z);
  
  /**
   * Estimate appropriate grid cell sizes for a collection of rigid bodies
   */
  void estimateCellSize(std::vector<RigidBody*> &rigidBodies);

  /**
   * Constant used in hashfunction
   */
  const static int prime1_ = 73856093;
  /**
   * Constant used in hashfunction
   */  
  const static int prime2_ = 19349663;
  /**
   * Constant used in hasfunction
   */  
  const static int prime3_ = 83492791;

  /**
   * List of cell sizes of the particular grid levels
   */
  std::list<Real> sizes_;
  
  /**
   * Number of cells
   */
  int  nCells_;  
  
  /**
   * Maximum number of levels
   */
  int  maxLevel_;  
  
  /**
   * Boolean that stores whether the grid sizes have been initialized
   */
  bool sizeInitialized_;  
  
  /**
   * Boolean array that stores the special property of the levels
   */
  bool boundaryLevel_[MAX_LEVELS_HGRID];  
  
  /**
   * Extents of the grid
   */
  Real boundaryExtents_[6];    
  
  /**
   * Pointer to the levels of the hierarchical grid
   */
  SimpleSpatialHash *levels_[MAX_LEVELS_HGRID];
    
};

}
#endif
