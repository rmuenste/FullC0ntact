
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
    levels_[j]=nullptr;
  }  
  
}

SpatialHashHierarchy::SpatialHashHierarchy(int ncells, Real dim[6], std::vector<RigidBody*> &vRigidBodies) : nCells_(ncells) 
{

  //copy the grid dimension
  memcpy(boundaryExtents_,dim,6*sizeof(Real));

  for(int j=0;j<MAX_LEVELS_HGRID;j++)
  {
    levels_[j]=nullptr;
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
    Real rad = body->getBoundingSphereRadius();
    sizes_.push_back(rad);
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
    //std::cout << "size: " << *liter << std::endl;
    
    if(temp2==sizes_.end())
      break;
    
    Real t2=*temp2;
    Real t1=*temp1;    
    Real distance=std::abs(t2 - t1);
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
    //std::cout << "-------------------------------------------" << std::endl;
    //std::cout << "levels size:" << sizes_.size() << std::endl;
    minspacing=std::numeric_limits<Real>::max();
    for(liter = sizes_.begin();liter!=sizes_.end();liter++)
    {
      std::list<Real>::iterator temp;
      std::list<Real>::iterator temp1;
      std::list<Real>::iterator temp2;
      temp1=liter;
      temp=temp1;
      temp2=++temp;
      Real distance=abs((*temp2) - (*temp1));
      if(distance < minspacing)
      {
        minspacing=distance;
        min1=temp1;
        min2=temp2;
        //std::cout << "found: (" << *min1 << "," << *min2 << ")" << std::endl;
      }
    }//end for
    
    //we have found the minimum pair, we
    //replace the second value by the average and remove it
    *min2 = ((*min1) + (*min2))/2.0;
    //std::cout << "removing:" << *min1 << std::endl;
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
#ifndef FC_SILENT
// std::cout<<"--------------------"<<std::endl;
// std::cout<<"HGrid Statistics: "<<std::endl;
// std::cout<<"HGrid Levels: "<<maxLevel_<<std::endl;
// for(int level=0;level<maxLevel_;level++)
// {
//   std::cout<<"Size Level "<<level<<" : "<<levels_[level]->getCellSize()<<std::endl;    
// }
// std::cout<<"--------------------"<<std::endl;  
#endif

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
  e.m_Cell.gridIndex_.x = (int)(center.x * invCellSize);
  e.m_Cell.gridIndex_.y = (int)(center.y * invCellSize);
  e.m_Cell.gridIndex_.z = (int)(center.z * invCellSize);
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
    break;
  }//end for
  
  NVT=8*NEL;
  ugrid.nvt_          = NVT;
  ugrid.nel_          = NEL;  
  ugrid.vertexCoords_ = new VECTOR3[NVT+1];
  ugrid.hexas_        = new Hexa[NEL];
  ugrid.m_myTraits      = new DTraits[NVT];
  
  //needed vertexcoords,hexas,traits
  int ive=0;
  int iel=0;
  //start with the highest level
  for(int level=getMaxLevel();level >= 0;level--)
  {

    if (level != 0)
      continue;

    SimpleSpatialHash::hashiterator iter = levels_[level]->begin();

    for(;iter!=levels_[level]->end();iter++,iel++)
    {

      //Get the entries of the hash bucket
      std::vector<CSpatialHashEntry>* vec = iter.Get();
      
      Real dist=0;
      //Real hgridsize = levels_[level]->getCellSize()/2.0;
      Real hgridsize = 0.0;
      
      CellCoords c(vec->front().m_Cell);
      Vec3 center((vec->front().m_Cell.x())*levels_[level]->getCellSize(),
                  (vec->front().m_Cell.y())*levels_[level]->getCellSize(),
                  (vec->front().m_Cell.z())*levels_[level]->getCellSize());

      hgridsize = levels_[level]->getCellSize();
      Real signx = 1.0;
      if (c.x()!=0)
        signx = c.x() / std::abs(c.x());
      else
      {
        if (vec->front().m_pBody->com_.x < 0.0)
          signx = -1.0;
      }

      Real signy = 1.0;
      if (c.y() != 0)
        signy = c.y() / std::abs(c.y());
      else
      {
        if (vec->front().m_pBody->com_.y < 0.0)
          signy = -1.0;
      }

      Real signz = -1.0;
      if (c.z() != 0)
        signz = c.z() / std::abs(c.z());
      else
      {
        if (vec->front().m_pBody->com_.z< 0.0)
          signz = -1.0;
      }

     
      ugrid.vertexCoords_[ive] = VECTOR3(center.x, center.y, center.z);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[0] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x + signx * hgridsize, center.y, center.z);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[1] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x + signx * hgridsize, center.y + signy * hgridsize, center.z);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[2] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x, center.y + signy * hgridsize, center.z);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[3] = ive++;

      ugrid.vertexCoords_[ive] = VECTOR3(center.x, center.y, center.z + signz * hgridsize);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[4] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x + signx * hgridsize, center.y, center.z + signz * hgridsize);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[5] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x + signx * hgridsize, center.y + signy * hgridsize, center.z + signz * hgridsize);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[6] = ive++;
      ugrid.vertexCoords_[ive] = VECTOR3(center.x, center.y + signy * hgridsize, center.z + signz * hgridsize);
      ugrid.m_myTraits[ive].distance = dist;
      ugrid.m_myTraits[ive].iTag = level;
      ugrid.hexas_[iel].hexaVertexIndices_[7] = ive++;
   
    }//end for
    
  }//end for
  
}

}
