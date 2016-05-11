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
#ifndef SIMPLESPATIALHASH_H
#define SIMPLESPATIALHASH_H



//===================================================
//                     INCLUDES
//===================================================
#include <spatialhash.h>
#include <compoundbody.h>
#include <subdomainboundary.h>
#include <stdexcept>


namespace i3d {

/**
 * @brief The class implements a non-hierarchical spatial hash data structure with a fixed grid size
 * 
 */  
class SimpleSpatialHash : public BasicSpatialHash {

public: 

  /**
   * Standard constructor
   */
  SimpleSpatialHash(); 

  /**
   * Initialize a grid with a certain number of cell storage
   */
  SimpleSpatialHash(int ncells); 

  SimpleSpatialHash(int ncells, Real size, Real dim[6]) : nCells_(ncells), cellSize_(size)
  {
    memcpy(extents_,dim,6*sizeof(Real));
    maxX_ = int(dim[1]/size);
    maxY_ = int(dim[3]/size);
    maxZ_ = int(dim[5]/size);

    cells_ = new std::vector<CSpatialHashEntry>[ncells];
    isCellUsed_ = new bool[ncells];
    nUsedCells_=0;
    for(int i=0;i<ncells;i++)
      isCellUsed_[i]=false;

  }

  /**
   * Destructor
   */  
  ~SimpleSpatialHash(); 

  void insert(SubdomainBoundary *body) override
  {
    throw std::logic_error("Function not implemented");
  }

  void insert(CompoundBody *body) override
  {
	throw std::logic_error("Function not implemented");
  }

  void convertToUnstructuredGrid(CUnstrGridr& ugrid) override
  {
	throw std::logic_error("Function not implemented");
  }

  /**
   * Insert an entry into the grid
   */
  void insert(CSpatialHashEntry &e);
  
  /**
   * Remove an entry
   */
  void remove(const CellCoords &cell);

  /**
   * Clear the information stored in the grid
   */
  void clear();

  /**
   * Check whether a cell is empty
   */
  bool isEmpty(const CellCoords &cell);
  
  /**
   * Returns the entries of a cell
   */
  std::vector<CSpatialHashEntry> *getCellEntries(CellCoords &cell);

/**
 * @brief An iterator that iterates over the elements of the CSimpleSpatialHash
 */  
  class hashiterator
  {
  public:
  	
	typedef std::vector<CSpatialHashEntry> value_type;
	typedef std::vector<CSpatialHashEntry>* pointer;
	typedef std::vector<CSpatialHashEntry>& reference;

    hashiterator() : _curpos(nullptr), _pHash(nullptr)
	{

	};

    hashiterator(std::vector<int>::iterator iter, SimpleSpatialHash *pHash) : _pHash(pHash), _iter(iter)
    {
      if(_iter!=pHash->usedCells_.end())
        _curpos = &pHash->cells_[(*iter)];
      else
      {
        //error
      }
    };

	reference operator*() {return *_curpos;}

	pointer Get() {return _curpos;}

	int GetPos() {return (*_iter);};

    //prefix operator
	hashiterator& operator++()
	{
		_iter++;
		if(_iter!=_pHash->usedCells_.end())
			_curpos = &_pHash->cells_[(*_iter)];
		return *this;
	}

    //postfix operator
	  hashiterator operator++(int)
	  {
		  hashiterator old_it = *this;
		  ++(*this);
		  return old_it;
	  }

    bool operator !=(hashiterator rhs){return _iter != rhs._iter;};

  protected:
    std::vector<CSpatialHashEntry>* _curpos;
    std::vector<int>::iterator _iter;
    SimpleSpatialHash *_pHash;
//    int _pos;
  };

  /**
   * Returns an iterator the beginning of the grid
   */
  hashiterator begin() {return hashiterator(usedCells_.begin(),this);};
  
  /**
   * Returns an iterator the end of the grid
   */  
  hashiterator end() {return hashiterator(usedCells_.end(),this);};

  /**
   * Get the number of used cells
   */
  int GetUsedCells() {return nUsedCells_;};

  /**
   * Get the number of cells
   */
  int GetNCells() {return nCells_;};
  
  /**
   * Get the cell size of the grid
   */
  Real getCellSize() {return cellSize_;};  
  
  /**
   * Get the maximum x-index
   */
  int GetMaxX() {return maxX_;};
  
  /**
   * Get the maximum y-index
   */  
  int GetMaxY() {return maxY_;};
  
  /**
   * Get the maximum z-index
   */  
  int GetMaxZ() {return maxZ_;};  

  /**
   * Check whether the cell is a special 'boundary cell'
   */
  inline bool isBoundary(const CellCoords &cell)
  {
    return (cell.x==0 || cell.y==0 || cell.z==0 || cell.x==maxX_ || cell.y==maxY_ || cell.z==maxZ_);
  }

  friend class SimpleSpatialHash::hashiterator;

private:

  /**
   * Compute the hashfunction
   */  
  int hash(int x, int y, int z);
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
   * Pointer to the cells of the grid
   */
  std::vector<CSpatialHashEntry> *cells_;
  
  /**
   * Number of cells in the grid
   */
  int              nCells_;
  
  /**
   * Maximum x-index of the grid
   */
  int              maxX_;
  
  /**
   * Maximum y-index of the grid
   */  
  int              maxY_;
  
  /**
   * Maximum z-index of the grid
   */  
  int              maxZ_;
  
  /**
   * Indices to the non-empty cells
   */
  std::vector<int> usedCells_;
  
  /**
   * Temporary storage
   */
  bool             *isCellUsed_;
  
  /**
   * Number of non-empty cells
   */  
  int              nUsedCells_;
  
  /**
   * Min-Max extents of the grid
   */
  Real             extents_[6];
  
  /**
   * Grid cell size
   */
  Real             cellSize_;

};

}
#endif
