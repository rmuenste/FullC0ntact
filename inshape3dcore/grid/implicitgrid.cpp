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


//===================================================
//                     INCLUDES
//===================================================


#include "implicitgrid.h"

namespace i3d {

ImplicitGrid::ImplicitGrid() 
{

}

ImplicitGrid::ImplicitGrid(BasicSpatialHash *spatialHash)
{

  spatialHash_=spatialHash;
  
}

ImplicitGrid::ImplicitGrid(BasicSpatialHash *spatialHash, Real cellSize)
{
  spatialHash_ = spatialHash;
  cellSize_    = cellSize;
}

ImplicitGrid::~ImplicitGrid() 
{
  delete spatialHash_;
}

void ImplicitGrid::addObject(RigidBody *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CellCoords cell;
  VECTOR3 center = body->com_;

  CSpatialHashEntry entry(body,cell);

  spatialHash_->insert(entry);

}

void ImplicitGrid::Insert(CompoundBody *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CellCoords cell;

  for(int i=0;i<body->getNumComponents();i++)
  {
    RigidBody *pBody = body->getComponent(i);      
    VECTOR3 center    = pBody->com_;
    CSpatialHashEntry entry(pBody,cell);
    spatialHash_->insert(entry);
  }

}

void ImplicitGrid::Insert(SubdomainBoundary *body)
{
  //calc grid coordinates
  //insert into spatial hash
  CellCoords cell;

  for(int i=0;i<body->rigidBodies_.size();i++)
  {
    RigidBody *currentBody = body->rigidBodies_[i];
    VECTOR3 center    = currentBody->com_;
    CSpatialHashEntry entry(currentBody,cell,CSpatialHashEntry::SUBDOMAIN);
    spatialHash_->insert(entry);
  }

}

void ImplicitGrid::removeObject(RigidBody *body)
{

}

void ImplicitGrid::clear()
{
  spatialHash_->clear();
}

}
