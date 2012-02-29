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

CSimpleSpatialHash::CSimpleSpatialHash() 
{
  m_pCells = NULL;
  m_bUsedCells = NULL;
  m_iUsedCells=0;
}

CSimpleSpatialHash::CSimpleSpatialHash(int ncells)
{
  m_iNCells = ncells;
  m_pCells = new std::vector<CSpatialHashEntry>[ncells];
  m_bUsedCells = new bool[ncells];
  m_iUsedCells=0;
  for(int i=0;i<ncells;i++)
    m_bUsedCells[i]=false;
}

CSimpleSpatialHash::~CSimpleSpatialHash() 
{
  if(m_pCells != NULL)
  {
    delete [] m_pCells;
    m_pCells = NULL;
  }
  if(m_bUsedCells != NULL)
  {
    delete [] m_bUsedCells;
    m_bUsedCells = NULL;
  }
}

int CSimpleSpatialHash::hash(int x, int y, int z)
{
  int value     =     ((x * m_iPrime1) % m_iNCells) 
                    + ((y * m_iPrime2) % m_iNCells) 
                    + ((z * m_iPrime3) % m_iNCells);

  value = value % m_iNCells; 

  if(value < 0)
  {
    value = m_iNCells + value;
  }
  return value;
}

void CSimpleSpatialHash::Insert(CSpatialHashEntry &e)
{

  //get the position of the rb
  VECTOR3 center = e.m_pBody->m_vCOM;

  //calculate the cell indices
  Real invCellSize = (Real)1.0/m_dCellSize;
  
  //calculate the cell indices
  e.m_Cell.x = (int)(center.x * invCellSize);
  e.m_Cell.y = (int)(center.y * invCellSize);
  e.m_Cell.z = (int)(center.z * invCellSize);

  //compute the hash function
  int index = hash(e.m_Cell.x,e.m_Cell.y,e.m_Cell.z);

  //insert into hash
  m_pCells[index].push_back(e);
  if(m_bUsedCells[index]==false)
  {
    m_vUsedCells.push_back(index);
    m_bUsedCells[index]=true;
    m_iUsedCells++;
  }

}

bool CSimpleSpatialHash::IsEmpty(const CCellCoords &cell)
{
  //compute the hash function
  int index = hash(cell.x,cell.y,cell.z);
  //check if empty
  return m_pCells[index].empty();
}

void CSimpleSpatialHash::Remove(const CCellCoords &cell)
{

}

void CSimpleSpatialHash::Clear()
{
  std::vector<int>::iterator i = m_vUsedCells.begin();
  for(;i!=m_vUsedCells.end();i++)
  {
    m_pCells[(*i)].clear();
    m_bUsedCells[(*i)]=false;
  }
  m_vUsedCells.clear();
  m_iUsedCells=0;

}

std::vector<CSpatialHashEntry>* CSimpleSpatialHash::GetCellEntries(CCellCoords &cell)
{
  int index = hash(cell.x,cell.y,cell.z);
  return &m_pCells[index];
}

}
