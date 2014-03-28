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
#include "broadphasestrategyrmt.h"
#include <world.h>
#include <hspatialhash.h>
#include <subdomainboundary.h>

namespace i3d {

CBroadPhaseStrategyRmt::CBroadPhaseStrategyRmt(World* pDomain) : BroadPhaseStrategy(pDomain)
{

}

CBroadPhaseStrategyRmt::~CBroadPhaseStrategyRmt()
{
  
}

void CBroadPhaseStrategyRmt::Init()
{
  //the current state is to clear and insert
  //it may be more efficient and as accurate just to update the
  //position of the bodies in the grid
  m_pImplicitGrid->Clear();
  m_BroadPhasePairs->clear();
  std::vector<RigidBody*>::iterator i = m_pWorld->rigidBodies_.begin();
  
  //Iterate over all rigid bodies and insert
  //into the spatial hash
  for(;i!=m_pWorld->rigidBodies_.end();i++)
  {
    int id = -1;
    RigidBody *body = *i;
    id = body->iID_;

    //make a dynamic cast
    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;
    //make a dynamic cast
    else if(body->shapeId_ == RigidBody::SUBDOMAIN)
    {
      SubdomainBoundary *subdomain = dynamic_cast<SubdomainBoundary*>(body);
      m_pImplicitGrid->Insert(subdomain);
      //remember the id of the subdomain
      m_iSubdomainID = subdomain->iID_;
    }
    //make a dynamic cast
    else if(body->shapeId_ == RigidBody::COMPOUND)
    {
      CompoundBody *compoundBody = dynamic_cast<CompoundBody*>(body);
      m_pImplicitGrid->Insert(compoundBody);
    }
    else
    {
      //insert the rigid body
      m_pImplicitGrid->Insert(body);
    }


  }//end for

  //insert boundary ?

  //insert subdomainboundary ?
  
}//end init

void CBroadPhaseStrategyRmt::Start()
{

  //perform the actual collision detection

  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(m_pImplicitGrid->GetSpatialHash());

  //start with the lowest level
  for(int level=0;level <= pHash->GetMaxLevel();level++)
  {
    
    CSimpleSpatialHash::hashiterator iter = pHash->GetGridLevel(level)->begin();

    //check on the same level
    for(;iter!=pHash->GetGridLevel(level)->end();iter++)
    {
      //Get the entries of the hash bucket
      std::vector<CSpatialHashEntry>* vec = iter.Get();
      //loop through the entries of the hash bucket
      std::vector<CSpatialHashEntry>::iterator viter = vec->begin();
      std::vector<CSpatialHashEntry>::iterator viter2;
      //14 tests for this strategy
      //check current cell 1
      for(;viter!=vec->end();viter++)
      {

        //get the cell index
        CCellCoords cell = viter->m_Cell;

        //get the rigid body
        RigidBody *pBody = viter->m_pBody;

        //check current cell 1
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
            if(pair.HasSubdomainBoundary())
              m_BroadPhasePairs->insert(pair);
          }
          else
          {
            BroadPhasePair pair(pBody,viter2->m_pBody);
            if(pair.HasSubdomainBoundary())
              m_BroadPhasePairs->insert(pair);
          }
        }//end for
        //check east cell 2
        CCellCoords cell2 = cell.GetEast();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check southwest cell 3
        cell2 = cell.GetSouthWest();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for

        }
        //check south cell 4
        cell2 = cell.GetSouth();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check southeast cell 5
        cell2 = cell.GetSouthEast();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check back cell 6
        cell2 = cell.GetBack();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check back east cell 7
        cell2 = cell.GetBackEast();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check back southwest cell 8
        cell2 = cell.GetBackSouthWest();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check back south cell 9
        cell2 = cell.GetBackSouth();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }
        //check back southeast cell 10
        cell2 = cell.GetBackSouthEast();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }//end if hash is empty

        //check back southeast cell 11
        cell2 = cell.GetBackNorthWest();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }//end if hash is empty

        //check back southeast cell 12
        cell2 = cell.GetBackNorth();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }//end if hash is empty

        //check back southeast cell 13
        cell2 = cell.GetBackNorthEast();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for
        }//end if hash is empty

        //check back south west cell 14
        cell2 = cell.GetBackWest();

        if(!pHash->IsEmpty(cell2))
        {
          std::vector<CSpatialHashEntry>::iterator i;
          std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cell2);
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
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
            else
            {
              BroadPhasePair pair(pBody,i->m_pBody);
              if(pair.HasSubdomainBoundary())
                m_BroadPhasePairs->insert(pair);
            }
          }//end for

        }//end if hash is empty

      }//end viter loop over objects in cell current level

      //we checked for collision with objects on the current level,
      //but it still remains to check the higher levels
      for(int nextLevel=level+1;nextLevel<=pHash->GetMaxLevel();nextLevel++)
      {
                  
        //check for a quick way out
        //if no cells are used at this level
        //we can continue to the next
        if(pHash->GetUsedCells(nextLevel) == 0)
          continue;

        //for every object in the cell
        for(viter = vec->begin();viter!=vec->end();viter++)
        {
          //compute object overlap with cells at current level (AABBoverlap + max overlap at current level)
          RigidBody *body = viter->m_pBody;
        
          //compute max overlap at level
          //the max overlap at a level is the maximum distance, 
          //that an object in the neighbouring cell can penetrate
          //into the cell under consideration
          Real overlaplevel = 0.5 * pHash->GetGridSize(nextLevel);
          Real delta = body->getBoundingSphereRadius() + overlaplevel;
          
          //compute the minimum and maximum cell indices
          int x0=int((body->com_.x-delta)/pHash->GetGridSize(nextLevel));//div by cell size
          int y0=int((body->com_.y-delta)/pHash->GetGridSize(nextLevel));
          int z0=int((body->com_.z-delta)/pHash->GetGridSize(nextLevel));

          int x1=int((body->com_.x+delta)/pHash->GetGridSize(nextLevel));
          int y1=int((body->com_.y+delta)/pHash->GetGridSize(nextLevel));
          int z1=int((body->com_.z+delta)/pHash->GetGridSize(nextLevel));

          //loop over the overlapped cells
          for(int x=x0;x<=x1;x++)
            for(int y=y0;y<=y1;y++)
              for(int z=z0;z<=z1;z++)
              {
                CCellCoords cellNextLevel(x,y,z,nextLevel);
                std::vector<CSpatialHashEntry>::iterator i;
                std::vector<CSpatialHashEntry> *vec = pHash->GetCellEntries(cellNextLevel);
                //add all entries in cell
                for(i = vec->begin();i!=vec->end();i++)
                {
                  CSpatialHashEntry &hentry = *i;
                  RigidBody *pBody = hentry.m_pBody;
                  //add close proximity pair
                  if(pBody->iID_ < body->iID_)
                  {
                    BroadPhasePair pair(pBody,body);
                    if(hentry.m_iType == CSpatialHashEntry::SUBDOMAIN)
                      m_BroadPhasePairs->insert(pair);
                  }
                  else
                  {
                    BroadPhasePair pair(body,pBody);
                    if(hentry.m_iType == CSpatialHashEntry::SUBDOMAIN)
                      m_BroadPhasePairs->insert(pair);
                  }
                }//end for

              }//for z

        }//end viter


      }//end for higher levels

    }//end for iter loop for cells
    
  }//end for level

}

}
