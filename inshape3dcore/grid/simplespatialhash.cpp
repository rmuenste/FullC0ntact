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
#include "simplespatialhash.h"
#include <iostream>


namespace i3d {

SimpleSpatialHash::SimpleSpatialHash() 
{
  cells_ = NULL;
  isCellUsed_ = NULL;
  nUsedCells_=0;
}

SimpleSpatialHash::SimpleSpatialHash(int ncells)
{
  nCells_ = ncells;
  cells_ = new std::vector<CSpatialHashEntry>[ncells];
  isCellUsed_ = new bool[ncells];
  nUsedCells_=0;
  for(int i=0;i<ncells;i++)
    isCellUsed_[i]=false;
}

SimpleSpatialHash::~SimpleSpatialHash() 
{
  if(cells_ != nullptr)
  {
    delete [] cells_;
    cells_ = nullptr;
  }
  if(isCellUsed_ != nullptr)
  {
    delete [] isCellUsed_;
    isCellUsed_ = nullptr;
  }
}

int SimpleSpatialHash::hash(int x, int y, int z)
{
  int value     =     ((x * m_iPrime1) % nCells_) 
                    + ((y * m_iPrime2) % nCells_) 
                    + ((z * m_iPrime3) % nCells_);

  value = value % nCells_; 

  if(value < 0)
  {
    value = nCells_ + value;
  }
  return value;
}

void SimpleSpatialHash::insert(CSpatialHashEntry &e)
{

  //get the position of the rb
  VECTOR3 center = e.m_pBody->com_;

  //calculate the cell indices
  Real invCellSize = (Real)1.0/cellSize_;
  
  //calculate the cell indices
  e.m_Cell.x = (int)(center.x * invCellSize);
  e.m_Cell.y = (int)(center.y * invCellSize);
  e.m_Cell.z = (int)(center.z * invCellSize);

  //compute the hash function
  int index = hash(e.m_Cell.x,e.m_Cell.y,e.m_Cell.z);

  //insert into hash
  cells_[index].push_back(e);
  if(isCellUsed_[index]==false)
  {
    usedCells_.push_back(index);
    isCellUsed_[index]=true;
    nUsedCells_++;
  }

}

bool SimpleSpatialHash::isEmpty(const CellCoords &cell)
{
  //compute the hash function
  int index = hash(cell.x,cell.y,cell.z);
  //check if empty
  return cells_[index].empty();
}

void SimpleSpatialHash::remove(const CellCoords &cell)
{

}

void SimpleSpatialHash::clear()
{
  std::vector<int>::iterator i = usedCells_.begin();
  for(;i!=usedCells_.end();i++)
  {
    cells_[(*i)].clear();
    isCellUsed_[(*i)]=false;
  }
  usedCells_.clear();
  nUsedCells_=0;

}

std::vector<CSpatialHashEntry>* SimpleSpatialHash::getCellEntries(CellCoords &cell)
{
  int index = hash(cell.x,cell.y,cell.z);
  return &cells_[index];
}

}
