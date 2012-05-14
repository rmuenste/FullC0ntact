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
  
  //create entries on each level
  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    m_pCells[j] = NULL;
    m_bUsedCells[j] = NULL;
  }

}

CHSpatialHash::CHSpatialHash(int ncells, Real dim[6], std::vector<CRigidBody*> &vRigidBodies) : m_iNCells(ncells) 
{

  //copy the grid dimension
  memcpy(m_dDimension,dim,6*sizeof(Real));

  //analyze the rigid bodies and create
  //the spatial hash hierarchy
  EstimateCellSize(vRigidBodies);

}

void CHSpatialHash::EstimateCellSize(std::vector<CRigidBody*> &vRigidBodies)
{
  int sizes=0;
  int k,j;
  std::vector<CRigidBody*>::iterator i = vRigidBodies.begin();

  Real max=(*i)->GetBoundingSphereRadius();
  Real min=(*i)->GetBoundingSphereRadius();
  for(;i!=vRigidBodies.end();i++)
  {
    CRigidBody *body = *i;
    if(body->m_iShape == CRigidBody::COMPOUND)
    {
      CCompoundBody *pBody = dynamic_cast<CCompoundBody*>(body);
      for(int n=0;n<pBody->GetNumComponents();n++)
      {
        Real rad = pBody->GetComponent(n)->GetBoundingSphereRadius();
        lSizes.push_back(rad);
      }
    }
    else
    {
      Real rad = body->GetBoundingSphereRadius();
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

  //The list now contains the final sizes
  //of our grid levels, we now initialize the grid
  //with these values
  liter = lSizes.begin();
  for(j=0;liter!=lSizes.end();liter++,j++)
  { 
    //calculate the max indices
    m_pGridSize[j]=(*liter)*2.0;
    m_iMaxIndices[j][0] = m_dDimension[1]/m_pGridSize[j];
    m_iMaxIndices[j][1] = m_dDimension[3]/m_pGridSize[j];
    m_iMaxIndices[j][2] = m_dDimension[5]/m_pGridSize[j];
  }

  //save the sizes and initialize grid
  m_iMaxLevel=lSizes.size();
  
  //create entries on each level
  for(j=0;j<m_iMaxLevel;j++)
  { 
    //use m_iNCells on each level
    m_pCells[j] = new std::vector<CSpatialHashEntry>[m_iNCells];
    m_iUsedCells[j]=0;
    
    //This array saves the information whether
    //a specific cell is used
    m_bUsedCells[j]=new bool[m_iNCells];

    //set the bool array to false
    for(k=0;k<m_iNCells;k++)
      m_bUsedCells[j][k]=false;
  }

  //for the unused levels set the pointers to NULL
  for(j=m_iMaxLevel;j<MAX_LEVELS_HGRID;j++)
  {
    m_bUsedCells[j]=NULL;
    m_pCells[j] = NULL;
    m_iUsedCells[j]=0;
  }
  
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    m_bIsBoundaryLevel[j]=false;
  }  


//#ifndef FEATFLOWLIB
//  std::cout<<"--------------------"<<std::endl;
//  std::cout<<"HGrid Statistics: "<<std::endl;
//  std::cout<<"HGrid Levels: "<<m_iMaxLevel<<std::endl;
//  for(int level=0;level<m_iMaxLevel;level++)
//  {
//    std::cout<<"Size Level "<<level<<" : "<<m_pGridSize[level]<<std::endl;    
//  }
//  std::cout<<"--------------------"<<std::endl;  
//#endif

}//EstimateCellSize

CHSpatialHash::~CHSpatialHash() 
{
  int j;
  for(j=0;j<MAX_LEVELS_HGRID;j++)
  {
    if(m_bUsedCells[j]!=NULL)
    {
      delete[] m_bUsedCells[j];
      m_bUsedCells[j]=NULL;

      delete[] m_pCells[j];
      m_pCells[j] = NULL;
    }
  }
}

void CHSpatialHash::Clear()
{
  int j;

  //loop through the levels clear the cells
  for(j=0;j<m_iMaxLevel;j++)
  {
    std::vector<int>::iterator i = m_vUsedCells[j].begin();
    for(;i!=m_vUsedCells[j].end();i++)
    {
      m_pCells[j][(*i)].clear();
      m_bUsedCells[j][(*i)]=false;
    }
    m_vUsedCells[j].clear();
    m_iUsedCells[j]=0;    
  }

}

std::vector<CSpatialHashEntry>* CHSpatialHash::GetCellEntries(CCellCoords &cell)
{
  int index = hash(cell.x,cell.y,cell.z);
  return &m_pCells[cell.level][index];
}

void CHSpatialHash::Insert(CSpatialHashEntry &e)
{

  int level=0;
  CRigidBody *body = e.m_pBody;

  Real d = body->GetBoundingSphereRadius() * 2.0;
  while(d > m_pGridSize[level])
  {
    level++;
  }

  if(e.m_pBody->m_iShape == CRigidBody::BOUNDARYBOX)
  {
    m_bIsBoundaryLevel[level]=true;
  }

  //get the position of the rb
  VECTOR3 center = e.m_pBody->m_vCOM;

  //calculate the cell indices
  Real invCellSize = (Real)1.0/m_pGridSize[level];
  
  //calculate the cell indices
  e.m_Cell.x = (int)(center.x * invCellSize);
  e.m_Cell.y = (int)(center.y * invCellSize);
  e.m_Cell.z = (int)(center.z * invCellSize);
  e.m_Cell.level = level;
  e.m_iLevel = level;

  //compute the hash function
  int index = hash(e.m_Cell.x,e.m_Cell.y,e.m_Cell.z);

  //insert into hash
  m_pCells[level][index].push_back(e);
  
  //if the cell was empty before set the 'used' marker
  if(m_bUsedCells[level][index]==false)
  {
    m_vUsedCells[level].push_back(index);
    m_bUsedCells[level][index]=true;
    m_iUsedCells[level]++;
  }

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
  //compute the hash function
  int index = hash(cell.x,cell.y,cell.z);
  //check if empty
  return m_pCells[cell.level][index].empty();
}

void CHSpatialHash::ConvertToUnstructuredGrid(CUnstrGridr& ugrid)
{
  int NEL=0;
  int NVT=0;
  //create entries on each level
  for(int j=0;j<m_iMaxLevel;j++)
  { 
    NEL+=m_iUsedCells[j];
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
    
    CHSpatialHash::hashiterator iter = begin(level);
    for(;iter!=end(level);iter++,iel++)
    {
      //Get the entries of the hash bucket
      std::vector<CSpatialHashEntry>* vec = iter.Get();
      
      Real dist=0;
      
      Real hgridsize = m_pGridSize[level]/2.0;
      
      VECTOR3 center(vec->front().m_Cell.x*m_pGridSize[level]+hgridsize,
                     vec->front().m_Cell.y*m_pGridSize[level]+hgridsize,
                     vec->front().m_Cell.z*m_pGridSize[level]+hgridsize);
     
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
