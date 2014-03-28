/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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
#include "collisionhash.h"
#include <iostream>

namespace i3d {
 
  
CollisionHash::CollisionHash()
{

  m_pBuckets = NULL;
  
}

CollisionHash::CollisionHash(int ncells) : m_iNCells(ncells)
{

  m_pBuckets = new std::list<CollisionInfo> [m_iNCells];
  
}

CollisionHash::~CollisionHash()
{

  if(m_pBuckets != NULL)
  {
    delete[] m_pBuckets;
    m_pBuckets = NULL;
  }
  
}

int CollisionHash::hash(int i, int j)
{
  int value     =     ((i * m_iPrime1) % m_iNCells) 
                    + ((j * m_iPrime2) % m_iNCells);

  value = value % m_iNCells; 

  if(value < 0)
  {
    value = m_iNCells + value;
  }
  return value;
}

void CollisionHash::Insert(CollisionInfo& info)
{
  int id = hash(info.iID1,info.iID2);
  
  m_pBuckets[id].push_back(info);
  CollisionInfo *i = &m_pBuckets[id].back();

  //add the edge to the bodies edge list
  RigidBody *body0 = i->m_pBody0;
  body0->addEdge(i);
  RigidBody *body1 = i->m_pBody1;
  body1->addEdge(i);
  m_vUsedCells.insert(id);

}

CollisionInfo* CollisionHash::Find(int i, int j)
{

  int id = hash(i,j);
  
  if(m_pBuckets[id].empty())
  {
    return NULL;
  }
  else
  {
    std::list<CollisionInfo>::iterator k = m_pBuckets[id].begin();
    for(;k!=m_pBuckets[id].end();k++)
    {
      CollisionInfo *info = &(*k);
      if(info->iID1 == i && info->iID2 == j)
        return info;
    }
    return NULL;
  }
}

void CollisionHash::Remove(CollisionInfo& info)
{
  //get the id of the hash bucket
  int id = hash(info.iID1,info.iID2);
  
  //check if the bucket is valid
  if(m_pBuckets[id].empty())
    return;
  else
  {
    //loop over the list 
    std::list<CollisionInfo>::iterator i = m_pBuckets[id].begin();
    while(i!=m_pBuckets[id].end())
    {
      CollisionInfo *collinfo = &(*i);
      if(collinfo->iID1 == info.iID1 && collinfo->iID2 == info.iID2)
      {
        //found the edge:
        //first remove the edge in the bodies edge list
        collinfo->m_pBody0->removeEdge(collinfo);
        collinfo->m_pBody1->removeEdge(collinfo);
        //now remove the edge
        i=m_pBuckets[id].erase(i);
        break;
      }
      i++;
    }
  }
}

bool CollisionHash::IsEmpty()
{
  return m_vUsedCells.empty();
}


void CollisionHash::Clear()
{
  
  std::set<int>::iterator i = m_vUsedCells.begin();
  for(;i!=m_vUsedCells.end();i++)
  {
    m_pBuckets[*i].clear();
  }
  m_vUsedCells.clear();
  
}

void CollisionHash::Update()
{
  
  std::set<int>::iterator i = m_vUsedCells.begin();
  while(i!=m_vUsedCells.end())
  {
    std::set<int>::iterator curr=i++;
    if(m_pBuckets[*curr].empty())
    {
      m_vUsedCells.erase(curr);
    }
  }
}


}
