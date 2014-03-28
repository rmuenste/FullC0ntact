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

CHSpatialHash::CHSpatialHash() 
{
  
  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    m_pLevels[j]=NULL;
  }  
  
}

CHSpatialHash::CHSpatialHash(int ncells, Real dim[6], std::vector<RigidBody*> &vRigidBodies) : m_iNCells(ncells) 
{

  //copy the grid dimension
  memcpy(m_dBoundaryExtents,dim,6*sizeof(Real));

  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    m_pLevels[j]=NULL;
  }    
  
  //analyze the rigid bodies and create
  //the spatial hash hierarchy
  EstimateCellSize(vRigidBodies);

}

void CHSpatialHash::EstimateCellSize(std::vector<RigidBody*> &vRigidBodies)
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
        lSizes.push_back(rad);
      }
    }
    else
    {
      Real rad = body->getBoundingSphereRadius();
      lSizes.push_back(rad);
    }
    //end for
  }//end for

  //we get to the target count of grid sizes by applying
  //a merge procedure, in such a way that iteratively
  //the two spheres with the minimum difference are merged.
  
  //sort the array to apply the merge procedure
  lSizes.sort();

  //successively merge the minimal pair until we reached the target level count
  std::list<Real>::iterator liter = lSizes.begin()++;
  std::list<Real>::iterator min1;
  std::list<Real>::iterator min2;
  Real minspacing=std::numeric_limits<Real>::max();

  //the list is sorted, we can now easily find and remove
  //sizes that are too close to process in a meaningful ways
  for(;liter!=lSizes.end();)
  {

    std::list<Real>::iterator temp;
    std::list<Real>::iterator temp1;
    std::list<Real>::iterator temp2;
    temp1=liter;
    temp=temp1;
    temp2=++temp;
    
    if(temp2==lSizes.end())
      break;
    
    Real t2=*temp2;
    Real t1=*temp1;    
    Real distance=fabs(t2 - t1);
    if(distance < 0.0001)
    {
      liter=lSizes.erase(liter);
    }
    else
    {
      liter++;
    }
  }//end for

  int isize=lSizes.size();
  liter = lSizes.begin()++;
  //now start merging the minimal pair
  //do this while size > target size
  while(lSizes.size() > MAX_LEVELS_HGRID)
  {
    for(;liter!=lSizes.end();liter++)
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
    lSizes.erase(min1);
  }//end while  

  int  maxIndices[MAX_LEVELS_HGRID][3];
  Real gridSize[MAX_LEVELS_HGRID];  
  
  //The list now contains the final sizes
  //of our grid levels, we now initialize the grid
  //with these values
  liter = lSizes.begin();
  for(j=0;liter!=lSizes.end();liter++,j++)
  { 
    //create a specific grid level
    m_pLevels[j] = new CSimpleSpatialHash(m_iNCells,(*liter)*2.0,m_dBoundaryExtents);
  }

  //save the sizes and initialize grid
  m_iMaxLevel=lSizes.size();
    
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    m_bIsBoundaryLevel[j]=false;
  }  

//#ifndef FEATFLOWLIB
 std::cout<<"--------------------"<<std::endl;
 std::cout<<"HGrid Statistics: "<<std::endl;
 std::cout<<"HGrid Levels: "<<m_iMaxLevel<<std::endl;
 for(int level=0;level<m_iMaxLevel;level++)
 {
   std::cout<<"Size Level "<<level<<" : "<<m_pLevels[level]->GetCellSize()<<std::endl;    
 }
 std::cout<<"--------------------"<<std::endl;  
//#endif

}//EstimateCellSize

CHSpatialHash::~CHSpatialHash() 
{
  int j;
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    if(m_pLevels[j]!=NULL)
      delete m_pLevels[j];
  }
}

void CHSpatialHash::Clear()
{
  int j;
  //loop through the levels clear the cells
  for(j=0;j<m_iMaxLevel;j++)
  {
    m_pLevels[j]->Clear();
  }
}

std::vector<CSpatialHashEntry>* CHSpatialHash::GetCellEntries(CCellCoords &cell)
{
  return m_pLevels[cell.level]->GetCellEntries(cell);
}

void CHSpatialHash::Insert(CSpatialHashEntry &e)
{

  int level=0;
  RigidBody *body = e.m_pBody;

  Real d = body->getBoundingSphereRadius() * 2.0;
  while(d > m_pLevels[level]->GetCellSize())
  {
    level++;
  }

  if(e.m_pBody->shapeId_ == RigidBody::BOUNDARYBOX)
  {
    m_bIsBoundaryLevel[level]=true;
  }

  //get the position of the rb
  VECTOR3 center = e.m_pBody->com_;

  //calculate the cell indices
  Real invCellSize = (Real)1.0/m_pLevels[level]->GetCellSize();
  
  //calculate the cell indices
  e.m_Cell.x = (int)(center.x * invCellSize);
  e.m_Cell.y = (int)(center.y * invCellSize);
  e.m_Cell.z = (int)(center.z * invCellSize);
  e.m_Cell.level = level;
  e.m_iLevel = level;

  m_pLevels[level]->Insert(e);

//DEBUG:
//   CSpatialHashEntry en = m_pCells[level][index].front();
//   CCellCoords c = en.m_Cell;
//   en.m_iLevel=en.m_iLevel;
}

void CHSpatialHash::Remove(const CCellCoords &cell)
{

}

int CHSpatialHash::hash(int x, int y, int z)
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

bool CHSpatialHash::IsEmpty(const CCellCoords &cell)
{
  //check if empty
  return m_pLevels[cell.level]->IsEmpty(cell);
}

void CHSpatialHash::ConvertToUnstructuredGrid(CUnstrGridr& ugrid)
{
  int NEL=0;
  int NVT=0;
  //create entries on each level
  for(int j=0;j<m_iMaxLevel;j++)
  { 
    NEL+=m_pLevels[j]->GetUsedCells();
  }//end for
  
  NVT=8*NEL;
  ugrid.m_iNVT          = NVT;
  ugrid.m_iNEL          = NEL;  
  ugrid.m_pVertexCoords = new VECTOR3[NVT+1];
  ugrid.m_pHexas        = new CHexa[NEL];
  ugrid.m_myTraits      = new DTraits[NVT];
  
  //needed vertexcoords,hexas,traits
  int ive=0;
  int iel=0;
  //start with the highest level
  for(int level=GetMaxLevel();level >= 0;level--)
  {
    
    CSimpleSpatialHash::hashiterator iter = m_pLevels[level]->begin();
    for(;iter!=m_pLevels[level]->end();iter++,iel++)
    {
      //Get the entries of the hash bucket
      std::vector<CSpatialHashEntry>* vec = iter.Get();
      
      Real dist=0;
      
      Real hgridsize = m_pLevels[level]->GetCellSize()/2.0;
      
      VECTOR3 center(vec->front().m_Cell.x*m_pLevels[level]->GetCellSize()+hgridsize,
                     vec->front().m_Cell.y*m_pLevels[level]->GetCellSize()+hgridsize,
                     vec->front().m_Cell.z*m_pLevels[level]->GetCellSize()+hgridsize);
     
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
