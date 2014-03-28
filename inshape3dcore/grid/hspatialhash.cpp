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


#include "hspatialhash.h"

#include <algorithm>
#include <compoundbody.h>

namespace i3d {

SpatialHashHierarchy::SpatialHashHierarchy() 
{
  
  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    levels_[j]=NULL;
  }  
  
}

SpatialHashHierarchy::SpatialHashHierarchy(int ncells, Real dim[6], std::vector<RigidBody*> &vRigidBodies) : nCells_(ncells) 
{

  //copy the grid dimension
  memcpy(boundaryExtents_,dim,6*sizeof(Real));

  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    levels_[j]=NULL;
  }    
  
  //analyze the rigid bodies and create
  //the spatial hash hierarchy
  estimateCellSize(vRigidBodies);

}

void SpatialHashHierarchy::estimateCellSize(std::vector<RigidBody*> &vRigidBodies)
{
  int sizes=0;
  int k,j;
  std::vector<RigidBody*>::iterator i = vRigidBodies.begin();

  Real max=(*i)->getBoundingSphereRadius();
  Real min=(*i)->getBoundingSphereRadius();
  for(;i!=vRigidBodies.end();i++)
  {
    RigidBody *body = *i;
    if(body->shapeId_ == RigidBody::COMPOUND)
    {
      CompoundBody *pBody = dynamic_cast<CompoundBody*>(body);
      for(int n=0;n<pBody->GetNumComponents();n++)
      {
        Real rad = pBody->GetComponent(n)->getBoundingSphereRadius();
        sizes_.push_back(rad);
      }
    }
    else
    {
      Real rad = body->getBoundingSphereRadius();
      sizes_.push_back(rad);
    }
    //end for
  }//end for

  //we get to the target count of grid sizes by applying
  //a merge procedure, in such a way that iteratively
  //the two spheres with the minimum difference are merged.
  
  //sort the array to apply the merge procedure
  sizes_.sort();

  //successively merge the minimal pair until we reached the target level count
  std::list<Real>::iterator liter = sizes_.begin()++;
  std::list<Real>::iterator min1;
  std::list<Real>::iterator min2;
  Real minspacing=std::numeric_limits<Real>::max();

  //the list is sorted, we can now easily find and remove
  //sizes that are too close to process in a meaningful ways
  for(;liter!=sizes_.end();)
  {

    std::list<Real>::iterator temp;
    std::list<Real>::iterator temp1;
    std::list<Real>::iterator temp2;
    temp1=liter;
    temp=temp1;
    temp2=++temp;
    
    if(temp2==sizes_.end())
      break;
    
    Real t2=*temp2;
    Real t1=*temp1;    
    Real distance=fabs(t2 - t1);
    if(distance < 0.0001)
    {
      liter=sizes_.erase(liter);
    }
    else
    {
      liter++;
    }
  }//end for

  int isize=sizes_.size();
  liter = sizes_.begin()++;
  //now start merging the minimal pair
  //do this while size > target size
  while(sizes_.size() > MAX_LEVELS_HGRID)
  {
    for(;liter!=sizes_.end();liter++)
    {
      std::list<Real>::iterator temp;
      std::list<Real>::iterator temp1;
      std::list<Real>::iterator temp2;
      temp1=liter;
      temp=temp1;
      temp2=temp++;
      Real distance=abs((*temp2) - (*temp1));
      if(distance < minspacing)
      {
        minspacing=distance;
        min1=temp1;
        min2=temp2;
      }
    }//end for
    
    //we have found the minimum pair, we
    //replace the second value by the average and remove it
    *min2 = ((*min1) + (*min2))/2.0;
    sizes_.erase(min1);
  }//end while  

  int  maxIndices[MAX_LEVELS_HGRID][3];
  Real gridSize[MAX_LEVELS_HGRID];  
  
  //The list now contains the final sizes
  //of our grid levels, we now initialize the grid
  //with these values
  liter = sizes_.begin();
  for(j=0;liter!=sizes_.end();liter++,j++)
  { 
    //create a specific grid level
    levels_[j] = new SimpleSpatialHash(nCells_,(*liter)*2.0,boundaryExtents_);
  }

  //save the sizes and initialize grid
  maxLevel_=sizes_.size();
    
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    boundaryLevel_[j]=false;
  }  

//#ifndef FEATFLOWLIB
 std::cout<<"--------------------"<<std::endl;
 std::cout<<"HGrid Statistics: "<<std::endl;
 std::cout<<"HGrid Levels: "<<maxLevel_<<std::endl;
 for(int level=0;level<maxLevel_;level++)
 {
   std::cout<<"Size Level "<<level<<" : "<<levels_[level]->getCellSize()<<std::endl;    
 }
 std::cout<<"--------------------"<<std::endl;  
//#endif

}//EstimateCellSize

SpatialHashHierarchy::~SpatialHashHierarchy() 
{
  int j;
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    if(levels_[j]!=NULL)
      delete levels_[j];
  }
}

void SpatialHashHierarchy::clear()
{
  int j;
  //loop through the levels clear the cells
  for(j=0;j<maxLevel_;j++)
  {
    levels_[j]->clear();
  }
}

std::vector<CSpatialHashEntry>* SpatialHashHierarchy::getCellEntries(CellCoords &cell)
{
  return levels_[cell.level]->getCellEntries(cell);
}

void SpatialHashHierarchy::insert(CSpatialHashEntry &e)
{

  int level=0;
  RigidBody *body = e.m_pBody;

  Real d = body->getBoundingSphereRadius() * 2.0;
  while(d > levels_[level]->getCellSize())
  {
    level++;
  }

  if(e.m_pBody->shapeId_ == RigidBody::BOUNDARYBOX)
  {
    boundaryLevel_[level]=true;
  }

  //get the position of the rb
  VECTOR3 center = e.m_pBody->com_;

  //calculate the cell indices
  Real invCellSize = (Real)1.0/levels_[level]->getCellSize();
  
  //calculate the cell indices
  e.m_Cell.x = (int)(center.x * invCellSize);
  e.m_Cell.y = (int)(center.y * invCellSize);
  e.m_Cell.z = (int)(center.z * invCellSize);
  e.m_Cell.level = level;
  e.m_iLevel = level;

  levels_[level]->insert(e);

//DEBUG:
//   CSpatialHashEntry en = m_pCells[level][index].front();
//   CCellCoords c = en.m_Cell;
//   en.m_iLevel=en.m_iLevel;
}

void SpatialHashHierarchy::remove(const CellCoords &cell)
{

}

int SpatialHashHierarchy::hash(int x, int y, int z)
{
  int value     =     ((x * prime1_) % nCells_) 
                    + ((y * prime2_) % nCells_) 
                    + ((z * prime3_) % nCells_);

  value = value % nCells_; 

  if(value < 0)
  {
    value = nCells_ + value;
  }
  return value;
}

bool SpatialHashHierarchy::isEmpty(const CellCoords &cell)
{
  //check if empty
  return levels_[cell.level]->isEmpty(cell);
}

void SpatialHashHierarchy::convertToUnstructuredGrid(CUnstrGridr& ugrid)
{
  int NEL=0;
  int NVT=0;
  //create entries on each level
  for(int j=0;j<maxLevel_;j++)
  { 
    NEL+=levels_[j]->GetUsedCells();
  }//end for
  
  NVT=8*NEL;
  ugrid.m_iNVT          = NVT;
  ugrid.m_iNEL          = NEL;  
  ugrid.m_pVertexCoords = new VECTOR3[NVT+1];
  ugrid.m_pHexas        = new Hexa[NEL];
  ugrid.m_myTraits      = new DTraits[NVT];
  
  //needed vertexcoords,hexas,traits
  int ive=0;
  int iel=0;
  //start with the highest level
  for(int level=getMaxLevel();level >= 0;level--)
  {
    
    SimpleSpatialHash::hashiterator iter = levels_[level]->begin();
    for(;iter!=levels_[level]->end();iter++,iel++)
    {
      //Get the entries of the hash bucket
      std::vector<CSpatialHashEntry>* vec = iter.Get();
      
      Real dist=0;
      
      Real hgridsize = levels_[level]->getCellSize()/2.0;
      
      VECTOR3 center(vec->front().m_Cell.x*levels_[level]->getCellSize()+hgridsize,
                     vec->front().m_Cell.y*levels_[level]->getCellSize()+hgridsize,
                     vec->front().m_Cell.z*levels_[level]->getCellSize()+hgridsize);
     
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x-hgridsize,center.y-hgridsize,center.z-hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;
      ugrid.m_pHexas[iel].m_iVertInd[0]=ive++;
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x+hgridsize,center.y-hgridsize,center.z-hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[1]=ive++;      
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x+hgridsize,center.y+hgridsize,center.z-hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[2]=ive++;      
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x-hgridsize,center.y+hgridsize,center.z-hgridsize);                        
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[3]=ive++;            
      
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x-hgridsize,center.y-hgridsize,center.z+hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[4]=ive++;            
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x+hgridsize,center.y-hgridsize,center.z+hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[5]=ive++;            
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x+hgridsize,center.y+hgridsize,center.z+hgridsize);
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[6]=ive++;            
      ugrid.m_pVertexCoords[ive]=VECTOR3(center.x-hgridsize,center.y+hgridsize,center.z+hgridsize);                        
      ugrid.m_myTraits[ive].distance=dist;
      ugrid.m_myTraits[ive].iTag=level;      
      ugrid.m_pHexas[iel].m_iVertInd[7]=ive++;            
   
    }//end for
    
  }//end for
  
}

}
