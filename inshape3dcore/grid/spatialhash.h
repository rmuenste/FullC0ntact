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
#ifndef SPATIALHASH_H
#define SPATIALHASH_H

#define MAX_LEVELS_HGRID 7

//===================================================
//                     INCLUDES
//===================================================
#include <vector>
#include <rigidbody.h>
#include <compoundbody.h>
#include <subdomainboundary.h>
#include <vector3_int.hpp>

namespace i3d {

/**
 * @brief The coordinates of a cell in a spatial hash
 *
 */  
  
class CellCoords {

public:
  CellCoords() : gridIndex_(0,0,0), level(0)
  {};

  CellCoords(int _x, int _y, int _z, int _level=0) : gridIndex_(_x,_y,_z), level(_level)
  {};

  CellCoords(const CellCoords &copy)
  {
    level = copy.level;
    gridIndex_ = copy.gridIndex_;
  }
  
  ~CellCoords()
  {};

  Vector3<int> gridIndex_;
  int level;

  inline int x() const {return gridIndex_.x;};
  inline int y() const {return gridIndex_.y;};
  inline int z() const {return gridIndex_.z;};

  bool operator !=(CellCoords rhs)
  {
    bool equal = (gridIndex_.x==rhs.gridIndex_.x)&&
                 (gridIndex_.y==rhs.gridIndex_.y)&&
                 (gridIndex_.z==rhs.gridIndex_.z)&&
                 (level==rhs.level);
    return !equal;
  };

  inline CellCoords GetEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y,gridIndex_.z,level);
  }

  inline CellCoords GetSouth()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y,gridIndex_.z-1,level);
  }

  inline CellCoords GetSouthEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y,gridIndex_.z-1,level);
  }

  inline CellCoords GetSouthWest()
  {
    return CellCoords(gridIndex_.x-1,gridIndex_.y,gridIndex_.z-1,level);
  }

  inline CellCoords GetFrontEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y-1,gridIndex_.z,level);
  }

  inline CellCoords GetFrontSouth()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y-1,gridIndex_.z-1,level);
  }

  inline CellCoords GetFrontSouthEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y-1,gridIndex_.z-1,level);
  }

  inline CellCoords GetFrontSouthWest()
  {
    return CellCoords(gridIndex_.x-1,gridIndex_.y-1,gridIndex_.z-1,level);
  }

  inline CellCoords GetBackEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y+1,gridIndex_.z,level);
  }

  inline CellCoords GetBackWest()
  {
    return CellCoords(gridIndex_.x-1,gridIndex_.y+1,gridIndex_.z,level);
  }

  inline CellCoords GetBackNorthEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y+1,gridIndex_.z+1,level);
  }

  inline CellCoords GetBackSouth()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y+1,gridIndex_.z-1,level);
  }

  inline CellCoords GetBackNorth()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y+1,gridIndex_.z+1,level);
  }

  inline CellCoords GetBackSouthEast()
  {
    return CellCoords(gridIndex_.x+1,gridIndex_.y+1,gridIndex_.z-1,level);
  }

  inline CellCoords GetBackSouthWest()
  {
    return CellCoords(gridIndex_.x-1,gridIndex_.y+1,gridIndex_.z-1,level);
  }

  inline CellCoords GetBackNorthWest()
  {
    return CellCoords(gridIndex_.x-1,gridIndex_.y+1,gridIndex_.z+1,level);
  }

  inline CellCoords GetBack()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y+1,gridIndex_.z,level);
  }

  inline CellCoords GetFront()
  {
    return CellCoords(gridIndex_.x,gridIndex_.y-1,gridIndex_.z,level);
  }


};

/**
 * @brief An entry in a spatial hash
 * 
 */

class CSpatialHashEntry {

public:
  CSpatialHashEntry(){};

  CSpatialHashEntry(RigidBody *body, const CellCoords & cell)
  {
    m_pBody = body;
    m_Cell  = cell;
    m_iType = DEFAULT;
  };
  
  CSpatialHashEntry(RigidBody *body, const CellCoords & cell, int itype)
  {
    m_pBody = body;
    m_Cell  = cell;
    m_iType = itype;
  };
  
  CSpatialHashEntry(const CSpatialHashEntry &rhs)
  {
    m_pBody=rhs.m_pBody;
    m_Cell =rhs.m_Cell;
    m_iLevel=rhs.m_iLevel;
    m_iType=rhs.m_iType;
  };

  ~CSpatialHashEntry(){};

  RigidBody *m_pBody;
  CellCoords m_Cell;
  int         m_iLevel;
  int         m_iType;
  
  enum
  {
    DEFAULT,
    SUBDOMAIN
  };

};

/**
 * @brief The Base class and interface for a spatial hash implementation
 * 
 */

class BasicSpatialHash {

public: 

  BasicSpatialHash();

  virtual ~BasicSpatialHash();

  virtual void insert(CSpatialHashEntry &e)=0;

  virtual void insert(SubdomainBoundary *body)=0;

  virtual void insert(CompoundBody *body)=0;

  virtual void remove(const CellCoords &cell)=0;

  virtual void clear()=0;

  virtual std::vector<CSpatialHashEntry> *getCellEntries(CellCoords &cell)=0;

  virtual void convertToUnstructuredGrid(CUnstrGridr& ugrid)=0;

  AABB3r boundingBox_[MAX_LEVELS_HGRID];

};

}
#endif
