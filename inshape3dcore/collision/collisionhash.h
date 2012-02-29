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
  
class CCollisionHash
{
  
public:  
  
  CCollisionHash();
  
  CCollisionHash(int ncells);  
  
  ~CCollisionHash();
  
  void Insert(CCollisionInfo &info);
  
  void Remove(CCollisionInfo &info);
  
  void Clear();

  void Update();
  
  bool IsEmpty();
  
  CCollisionInfo* Find(int i, int j);
  
  std::list<CCollisionInfo> *GetBucket(int i, int j);
  
  int hash(int i, int j);
  
  const static int m_iPrime1 = 73856093;
  const static int m_iPrime2 = 19349663;
  
  std::list<CCollisionInfo> *m_pBuckets;
  std::set<int>              m_vUsedCells;
  
  int m_iNCells;
  
/**
 * @brief An iterator that iterates over the elements in the CCollisionHash
 */  
  class iterator
  {
  public:
    
    typedef CCollisionInfo* pointer;
    typedef CCollisionInfo& reference;
    iterator(){};
    iterator(std::set<int>::iterator iter, CCollisionHash *pHash) : _pHash(pHash), _iter(iter)
    {
      if(_iter!=pHash->m_vUsedCells.end())
      {
        _bucket = &pHash->m_pBuckets[(*iter)];
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
        if(_iter != _pHash->m_vUsedCells.end())
        {
          _bucket = &_pHash->m_pBuckets[(*_iter)];
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
    std::list<CCollisionInfo> *_bucket;
    std::list<CCollisionInfo>::iterator _liter;        
    std::set<int>::iterator _iter;
    CCollisionHash *_pHash;
  };

  iterator begin() {return iterator(m_vUsedCells.begin(),this);};
  iterator end() {return iterator(m_vUsedCells.end(),this);};
  
};

}

#endif // COLLISIONHASH_H
