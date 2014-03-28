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

  buckets_ = NULL;
  
}

CollisionHash::CollisionHash(int ncells) : nCells_(ncells)
{

  buckets_ = new std::list<CollisionInfo> [nCells_];
  
}

CollisionHash::~CollisionHash()
{

  if(buckets_ != NULL)
  {
    delete[] buckets_;
    buckets_ = NULL;
  }
  
}

int CollisionHash::hash(int i, int j)
{
  int value     =     ((i * prime1_) % nCells_) 
                    + ((j * prime2_) % nCells_);

  value = value % nCells_; 

  if(value < 0)
  {
    value = nCells_ + value;
  }
  return value;
}

void CollisionHash::insert(CollisionInfo& info)
{
  int id = hash(info.iID1,info.iID2);
  
  buckets_[id].push_back(info);
  CollisionInfo *i = &buckets_[id].back();

  //add the edge to the bodies edge list
  RigidBody *body0 = i->m_pBody0;
  body0->addEdge(i);
  RigidBody *body1 = i->m_pBody1;
  body1->addEdge(i);
  usedCells_.insert(id);

}

CollisionInfo* CollisionHash::find(int i, int j)
{

  int id = hash(i,j);
  
  if(buckets_[id].empty())
  {
    return NULL;
  }
  else
  {
    std::list<CollisionInfo>::iterator k = buckets_[id].begin();
    for(;k!=buckets_[id].end();k++)
    {
      CollisionInfo *info = &(*k);
      if(info->iID1 == i && info->iID2 == j)
        return info;
    }
    return NULL;
  }
}

void CollisionHash::remove(CollisionInfo& info)
{
  //get the id of the hash bucket
  int id = hash(info.iID1,info.iID2);
  
  //check if the bucket is valid
  if(buckets_[id].empty())
    return;
  else
  {
    //loop over the list 
    std::list<CollisionInfo>::iterator i = buckets_[id].begin();
    while(i!=buckets_[id].end())
    {
      CollisionInfo *collinfo = &(*i);
      if(collinfo->iID1 == info.iID1 && collinfo->iID2 == info.iID2)
      {
        //found the edge:
        //first remove the edge in the bodies edge list
        collinfo->m_pBody0->removeEdge(collinfo);
        collinfo->m_pBody1->removeEdge(collinfo);
        //now remove the edge
        i=buckets_[id].erase(i);
        break;
      }
      i++;
    }
  }
}

bool CollisionHash::isEmpty()
{
  return usedCells_.empty();
}


void CollisionHash::clear()
{
  
  std::set<int>::iterator i = usedCells_.begin();
  for(;i!=usedCells_.end();i++)
  {
    buckets_[*i].clear();
  }
  usedCells_.clear();
  
}

void CollisionHash::update()
{
  
  std::set<int>::iterator i = usedCells_.begin();
  while(i!=usedCells_.end())
  {
    std::set<int>::iterator curr=i++;
    if(buckets_[*curr].empty())
    {
      usedCells_.erase(curr);
    }
  }
}


}
