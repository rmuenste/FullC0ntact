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

namespace i3d {

/**
 * @brief The class implements a non-hierarchical spatial hash data structure with a fixed grid size
 * 
 */  
class CSimpleSpatialHash : public CBasicSpatialHash {

public: 

  /**
   * Standard constructor
   */
  CSimpleSpatialHash(); 

  /**
   * Initialize a grid with a certain number of cell storage
   */
  CSimpleSpatialHash(int ncells); 

  CSimpleSpatialHash(int ncells, Real size, Real dim[6]) : m_iNCells(ncells), m_dCellSize(size)
  {
    memcpy(m_dDimension,dim,6*sizeof(Real));
    m_imaxX = int(dim[1]/size);
    m_imaxY = int(dim[3]/size);
    m_imaxZ = int(dim[5]/size);

    m_pCells = new std::vector<CSpatialHashEntry>[ncells];
    m_bUsedCells = new bool[ncells];
    m_iUsedCells=0;
    for(int i=0;i<ncells;i++)
      m_bUsedCells[i]=false;

  }

  /**
   * Destructor
   */  
  ~CSimpleSpatialHash(); 

  /**
   * Insert an entry into the grid
   */
  void Insert(CSpatialHashEntry &e);
  
  /**
   * Remove an entry
   */
  void Remove(const CCellCoords &cell);

  /**
   * Clear the information stored in the grid
   */
  void Clear();

  /**
   * Check whether a cell is empty
   */
  bool IsEmpty(const CCellCoords &cell);
  
  /**
   * Returns the entries of a cell
   */
  std::vector<CSpatialHashEntry> *GetCellEntries(CCellCoords &cell);

/**
 * @brief An iterator that iterates over the elements of the CSimpleSpatialHash
 */  
  class hashiterator
  {
  public:
  	
	  typedef std::vector<CSpatialHashEntry> value_type;
	  typedef std::vector<CSpatialHashEntry>* pointer;
	  typedef std::vector<CSpatialHashEntry>& reference;
    hashiterator(){};
    hashiterator(std::vector<int>::iterator iter, CSimpleSpatialHash *pHash) : _pHash(pHash), _iter(iter)
    {
      if(_iter!=pHash->m_vUsedCells.end())
        _curpos = &pHash->m_pCells[(*iter)];
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
      if(_iter!=_pHash->m_vUsedCells.end())
        _curpos = &_pHash->m_pCells[(*_iter)];
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
    CSimpleSpatialHash *_pHash;
//    int _pos;
  };

  /**
   * Returns an iterator the beginning of the grid
   */
  hashiterator begin() {return hashiterator(m_vUsedCells.begin(),this);};
  
  /**
   * Returns an iterator the end of the grid
   */  
  hashiterator end() {return hashiterator(m_vUsedCells.end(),this);};

  /**
   * Get the number of used cells
   */
  int GetUsedCells() {return m_iUsedCells;};

  /**
   * Get the number of cells
   */
  int GetNCells() {return m_iNCells;};
  
  /**
   * Get the cell size of the grid
   */
  Real GetCellSize() {return m_dCellSize;};  
  
  /**
   * Get the maximum x-index
   */
  int GetMaxX() {return m_imaxX;};
  
  /**
   * Get the maximum y-index
   */  
  int GetMaxY() {return m_imaxY;};
  
  /**
   * Get the maximum z-index
   */  
  int GetMaxZ() {return m_imaxZ;};  

  /**
   * Check whether the cell is a special 'boundary cell'
   */
  inline bool IsBoundary(const CCellCoords &cell)
  {
    return (cell.x==0 || cell.y==0 || cell.z==0 || cell.x==m_imaxX || cell.y==m_imaxY || cell.z==m_imaxZ);
  }

  friend class CSimpleSpatialHash::hashiterator;

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
  std::vector<CSpatialHashEntry> *m_pCells;
  
  /**
   * Number of cells in the grid
   */
  int              m_iNCells;
  
  /**
   * Maximum x-index of the grid
   */
  int              m_imaxX;
  
  /**
   * Maximum y-index of the grid
   */  
  int              m_imaxY;
  
  /**
   * Maximum z-index of the grid
   */  
  int              m_imaxZ;
  
  /**
   * Indices to the non-empty cells
   */
  std::vector<int> m_vUsedCells;
  
  /**
   * Temporary storage
   */
  bool             *m_bUsedCells;
  
  /**
   * Number of non-empty cells
   */  
  int              m_iUsedCells;
  
  /**
   * Min-Max extents of the grid
   */
  Real             m_dDimension[6];
  
  /**
   * Grid cell size
   */
  Real             m_dCellSize;

};

}
#endif
