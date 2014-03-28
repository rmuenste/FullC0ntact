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
#include "broadphasestrategygrid.h"
#include <simplespatialhash.h>
#include <world.h>

namespace i3d {

CBroadPhaseStrategyGrid::CBroadPhaseStrategyGrid(World* pDomain) : BroadPhaseStrategy(pDomain)
{

}

CBroadPhaseStrategyGrid::~CBroadPhaseStrategyGrid()
{

}

void CBroadPhaseStrategyGrid::Init()
{
  //clear and insert
  //or
  //update
  m_pImplicitGrid->clear();
  m_BroadPhasePairs->clear();
  std::vector<RigidBody*>::iterator i = m_pWorld->rigidBodies_.begin();
  for(;i!=m_pWorld->rigidBodies_.end();i++)
  {

    if((*i)->shapeId_==RigidBody::BOUNDARYBOX)
      continue;

    int id=-1;
    id=(*i)->iID_;
    m_pImplicitGrid->addObject((*i));

  }
  
}

void CBroadPhaseStrategyGrid::Start()
{
  //perform the actual collision detection

  //iterate through the used cells of spatial hash
  SimpleSpatialHash *pHash = dynamic_cast<SimpleSpatialHash*>(m_pImplicitGrid->getSpatialHash());
  SimpleSpatialHash::hashiterator iter = pHash->begin();

  for(;iter!=pHash->end();iter++)
  {
    //Get the entries of the hash bucket
    std::vector<CSpatialHashEntry>* vec = iter.Get();
    //loop through the entries of the hash bucket
    std::vector<CSpatialHashEntry>::iterator viter = vec->begin();
    std::vector<CSpatialHashEntry>::iterator viter2;
    //10 tests for this strategy
    //check current cell 1
    for(;viter!=vec->end();viter++)
    {
      //get the cell index
      CellCoords cell = viter->m_Cell;

      //get the rigid body
      RigidBody *pBody = viter->m_pBody;

      bool boundaryadded = false;

      if(pHash->isBoundary(cell) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      //check current cell
      viter2=viter;
      viter2++;
      for(;viter2!=vec->end();viter2++)
      {
        //check for a hash collision
        if(viter2->m_Cell != cell)
          continue;

        //add close proximity pair
        if(viter2->m_pBody->iID_ < pBody->iID_)
        {
          BroadPhasePair pair(viter2->m_pBody,pBody);
          m_BroadPhasePairs->insert(pair);
        }
        else
        {
          BroadPhasePair pair(pBody,viter2->m_pBody);
          m_BroadPhasePairs->insert(pair);
        }
      }//end for
      //check east cell 2
      CellCoords cell2 = cell.GetEast();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check southwest cell 3
      cell2 = cell.GetSouthWest();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for

      }
      //check south cell 4
      cell2 = cell.GetSouth();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check southeast cell 5
      cell2 = cell.GetSouthEast();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check back cell 6
      cell2 = cell.GetBack();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check back east cell 7
      cell2 = cell.GetBackEast();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check back southwest cell 8
      cell2 = cell.GetBackSouthWest();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check back south cell 9
      cell2 = cell.GetBackSouth();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }
      //check back southeast cell 10
      cell2 = cell.GetBackSouthEast();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }//end if hash is empty

      //check back southeast cell 11
      cell2 = cell.GetBackNorthWest();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }//end if hash is empty

      //check back southeast cell 12
      cell2 = cell.GetBackNorth();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }//end if hash is empty

      //check back southeast cell 13
      cell2 = cell.GetBackNorthEast();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }//end if hash is empty

      //check back south west cell 14
      cell2 = cell.GetBackWest();

      //check for boundary
      if(pHash->isBoundary(cell2) && !boundaryadded)
      {
        BroadPhasePair pair(pBody,this->m_pWorld->rigidBodies_.back());
        m_BroadPhasePairs->insert(pair);
        boundaryadded=true;
      }

      if(!pHash->isEmpty(cell2))
      {
        std::vector<CSpatialHashEntry>::iterator i;
        std::vector<CSpatialHashEntry> *vec = pHash->getCellEntries(cell2);
        //loop through the entries and add
        for(i = vec->begin();i!=vec->end();i++)
        {
          //check for a hash collision
          if(i->m_Cell != cell2)
            continue;

          //add close proximity pair
          if(i->m_pBody->iID_ < pBody->iID_)
          {
            BroadPhasePair pair(i->m_pBody,pBody);
            m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,i->m_pBody);
            m_BroadPhasePairs->insert(pair);
          }
        }//end for
      }//end if hash is empty

    }//end viter
  }//end for iter
}//end Start

}
