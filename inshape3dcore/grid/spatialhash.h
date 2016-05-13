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

namespace i3d {

/**
 * @brief The coordinates of a cell in a spatial hash
 *
 */  
  
class CellCoords {

public:
  CellCoords() : level(0) {};
  CellCoords(int _x, int _y, int _z, int _level=0) : x(_x), y(_y), z(_z), level(_level) {};
  
  ~CellCoords(){};

  bool operator !=(CellCoords rhs){return !((x==rhs.x)&&(y==rhs.y)&&(z==rhs.z)&&(level==rhs.level));};

  inline CellCoords GetEast()
  {
    return CellCoords(x+1,y,z,level);
  }

  inline CellCoords GetSouth()
  {
    return CellCoords(x,y,z-1,level);
  }

  inline CellCoords GetSouthEast()
  {
    return CellCoords(x+1,y,z-1,level);
  }

  inline CellCoords GetSouthWest()
  {
    return CellCoords(x-1,y,z-1,level);
  }

  inline CellCoords GetFrontEast()
  {
    return CellCoords(x+1,y-1,z,level);
  }

  inline CellCoords GetFrontSouth()
  {
    return CellCoords(x,y-1,z-1,level);
  }

  inline CellCoords GetFrontSouthEast()
  {
    return CellCoords(x+1,y-1,z-1,level);
  }

  inline CellCoords GetFrontSouthWest()
  {
    return CellCoords(x-1,y-1,z-1,level);
  }

  inline CellCoords GetBackEast()
  {
    return CellCoords(x+1,y+1,z,level);
  }

  inline CellCoords GetBackWest()
  {
    return CellCoords(x-1,y+1,z,level);
  }

  inline CellCoords GetBackNorthEast()
  {
    return CellCoords(x+1,y+1,z+1,level);
  }

  inline CellCoords GetBackSouth()
  {
    return CellCoords(x,y+1,z-1,level);
  }

  inline CellCoords GetBackNorth()
  {
    return CellCoords(x,y+1,z+1,level);
  }

  inline CellCoords GetBackSouthEast()
  {
    return CellCoords(x+1,y+1,z-1,level);
  }

  inline CellCoords GetBackSouthWest()
  {
    return CellCoords(x-1,y+1,z-1,level);
  }

  inline CellCoords GetBackNorthWest()
  {
    return CellCoords(x-1,y+1,z+1,level);
  }

  inline CellCoords GetBack()
  {
    return CellCoords(x,y+1,z,level);
  }

  inline CellCoords GetFront()
  {
    return CellCoords(x,y-1,z,level);
  }

  CellCoords(const CellCoords &copy)
  {
    x = copy.x;
    y = copy.y;
    z = copy.z;
    level = copy.level;
  }

  int x,y,z;
  int level;

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
