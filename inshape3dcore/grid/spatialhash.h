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



//===================================================
//                     INCLUDES
//===================================================
#include <vector>
#include <rigidbody.h>

namespace i3d {

/**
 * @brief The coordinates of a cell in a spatial hash
 *
 */  
  
class CCellCoords {

public:
  CCellCoords() : level(0) {};
  CCellCoords(int _x, int _y, int _z, int _level=0) : x(_x), y(_y), z(_z), level(_level) {};
  
  ~CCellCoords(){};

  bool operator !=(CCellCoords rhs){return !((x==rhs.x)&&(y==rhs.y)&&(z==rhs.z)&&(level==rhs.level));};

  inline CCellCoords GetEast()
  {
    return CCellCoords(x+1,y,z,level);
  }

  inline CCellCoords GetSouth()
  {
    return CCellCoords(x,y,z-1,level);
  }

  inline CCellCoords GetSouthEast()
  {
    return CCellCoords(x+1,y,z-1,level);
  }

  inline CCellCoords GetSouthWest()
  {
    return CCellCoords(x-1,y,z-1,level);
  }

  inline CCellCoords GetFrontEast()
  {
    return CCellCoords(x+1,y-1,z,level);
  }

  inline CCellCoords GetFrontSouth()
  {
    return CCellCoords(x,y-1,z-1,level);
  }

  inline CCellCoords GetFrontSouthEast()
  {
    return CCellCoords(x+1,y-1,z-1,level);
  }

  inline CCellCoords GetFrontSouthWest()
  {
    return CCellCoords(x-1,y-1,z-1,level);
  }

  inline CCellCoords GetBackEast()
  {
    return CCellCoords(x+1,y+1,z,level);
  }

  inline CCellCoords GetBackWest()
  {
    return CCellCoords(x-1,y+1,z,level);
  }

  inline CCellCoords GetBackNorthEast()
  {
    return CCellCoords(x+1,y+1,z+1,level);
  }

  inline CCellCoords GetBackSouth()
  {
    return CCellCoords(x,y+1,z-1,level);
  }

  inline CCellCoords GetBackNorth()
  {
    return CCellCoords(x,y+1,z+1,level);
  }

  inline CCellCoords GetBackSouthEast()
  {
    return CCellCoords(x+1,y+1,z-1,level);
  }

  inline CCellCoords GetBackSouthWest()
  {
    return CCellCoords(x-1,y+1,z-1,level);
  }

  inline CCellCoords GetBackNorthWest()
  {
    return CCellCoords(x-1,y+1,z+1,level);
  }

  inline CCellCoords GetBack()
  {
    return CCellCoords(x,y+1,z,level);
  }

  inline CCellCoords GetFront()
  {
    return CCellCoords(x,y-1,z,level);
  }

  CCellCoords(const CCellCoords &copy)
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

  CSpatialHashEntry(CRigidBody *body, const CCellCoords & cell)
  {
    m_pBody = body;
    m_Cell  = cell;
    m_iType = DEFAULT;
  };
  
  CSpatialHashEntry(CRigidBody *body, const CCellCoords & cell, int itype)
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

  CRigidBody *m_pBody;
  CCellCoords m_Cell;
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

class CBasicSpatialHash {

public: 

  CBasicSpatialHash();
  virtual ~CBasicSpatialHash();

  virtual void Insert(CSpatialHashEntry &e)=0;

  virtual void Remove(const CCellCoords &cell)=0;

  virtual void Clear()=0;

  virtual std::vector<CSpatialHashEntry> *GetCellEntries(CCellCoords &cell)=0;

};

}
#endif
