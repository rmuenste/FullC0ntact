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

#ifndef COLLISIONHASH_H
#define COLLISIONHASH_H

//===================================================
//                     INCLUDES
//===================================================
#include <iostream>
#include <collisioninfo.h>
#include <list>
#include <set>

namespace i3d {

/**
 * @brief A hashtable for CCollisionInfo objects
 *
 */  
  
class CollisionHash
{
  
public:  
  
  std::list<CollisionInfo> *getBucket(int i, int j);
  
  int hash(int i, int j);
  
  const static int prime1_ = 73856093;
  
  const static int prime2_ = 19349663;
  
  std::list<CollisionInfo> *buckets_;
  
  std::set<int>             usedCells_;
  
  int nCells_;
    
  CollisionHash();
  
  CollisionHash(int ncells);  
  
  ~CollisionHash();
  
  /**
   * Insert a new collision pair
   */
  void insert(CollisionInfo &info);
  
  /**
   * Remove a collision pair
   */
  void remove(CollisionInfo &info);
  
  /**
   * Clear the container
   */
  void clear();

  void update();
  
  /**
   * Checks whether the container is empty
   */
  bool isEmpty();
  
  /**
   * Search for a collision pair i,j
   */
  CollisionInfo* find(int i, int j);
    
/**
 * @brief An iterator that iterates over the elements in the CCollisionHash
 */  
  class iterator
  {
  public:
    
    typedef CollisionInfo* pointer;
    typedef CollisionInfo& reference;
    iterator() : _bucket(nullptr), _pHash(nullptr)
    {

    };

    iterator(std::set<int>::iterator iter, CollisionHash *pHash) : _iter(iter),  _pHash(pHash)
    {
      if(_iter!=pHash->usedCells_.end())
      {
        _bucket = &pHash->buckets_[(*iter)];
        _liter  = _bucket->begin();
      }
      else
      {
        //error
      }
    };

    reference operator*() {return *_liter;}

    pointer Get() {return &(*_liter);}

    int GetPos() {return (*_iter);};

    //prefix operator
    iterator& operator++()
    {
      _liter++;
      if(_liter==_bucket->end())
      {
        _iter++;
        if(_iter != _pHash->usedCells_.end())
        {
          _bucket = &_pHash->buckets_[(*_iter)];
          _liter  = _bucket->begin();
        }
      }
      return *this;
    }

    //postfix operator
    iterator operator++(int)
    {
      iterator old_it = *this;
      ++(*this);
      return old_it;
    }

    bool operator !=(iterator rhs){return _iter != rhs._iter;};
    bool operator ==(iterator rhs){return _iter == rhs._iter;};    

  protected:
    std::list<CollisionInfo> *_bucket;
    std::list<CollisionInfo>::iterator _liter;        
    std::set<int>::iterator _iter;
    CollisionHash *_pHash;
  };

  iterator begin() {return iterator(usedCells_.begin(),this);};
  iterator end() {return iterator(usedCells_.end(),this);};
  
};

}

#endif // COLLISIONHASH_H
