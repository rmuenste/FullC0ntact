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

namespace i3d {

/**
* @brief This class acts as an interface to the spatialhash in a broadphase algorithm
* 
*/
class CImplicitGrid {

public: 

  CImplicitGrid(); 
  CImplicitGrid(CSpatialHash *pSpatialHash); 
  CImplicitGrid(CSpatialHash *pSpatialHash, Real cellSize); 

  ~CImplicitGrid(); 

  void Insert(CRigidBody *body);

  void Insert(CCompoundBody *body);

  void Remove(CRigidBody *body);

  void Clear();

  inline void SetCellSize(Real size) {m_dCellSize=size;};
  inline Real GetCellSize() const {return m_dCellSize;};
  inline CSpatialHash* GetSpatialHash() {return m_pSpatialHash;};

private:

  CSpatialHash *m_pSpatialHash;
  Real          m_dCellSize;

};

}
#endif
