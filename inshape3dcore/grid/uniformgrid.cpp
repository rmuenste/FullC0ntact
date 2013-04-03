//===================================================
//                     INCLUDES
//===================================================
#include "uniformgrid.h"
#include <mymath.h>
#include <aabb3.h>
#include <stdio.h>
#include <stdlib.h>
#include <rigidbody.h>

namespace i3d {

template<class T, class CellType>
CUniformGrid<T,CellType>::CUniformGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element)
{
  
  m_dCellSize = 2.0 * element.GetBoundingSphereRadius();
  
  int x = (m_bxBox.m_Extends[0]+m_dCellSize)/m_dCellSize;
  int y = (m_bxBox.m_Extends[1]+m_dCellSize)/m_dCellSize;
  int z = (m_bxBox.m_Extends[2]+m_dCellSize)/m_dCellSize;

  int xy = x*y;
  int xz = x*z;
  int yz = y*z;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];

}

template<class T, class CellType>
CUniformGrid<T,CellType>::CUniformGrid()
{
  
  m_pCells = NULL;

}

template<class T, class CellType>
void CUniformGrid<T,CellType>::InitGrid(const CAABB3<T> &boundingBox, const CAABB3<T> &element)
{
   
  m_dCellSize = 2.0 * element.GetBoundingSphereRadius();
  m_bxBox = boundingBox;
    
  int x = (2.0*m_bxBox.m_Extends[0]+m_dCellSize)/m_dCellSize;
  int y = (2.0*m_bxBox.m_Extends[1]+m_dCellSize)/m_dCellSize;
  int z = (2.0*m_bxBox.m_Extends[2]+m_dCellSize)/m_dCellSize;

  // int nGhostLayerCells = (2 * xy + 2 * xz + 2 * yz) + (4 * x + 4 * y + 4 * z) + 8;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];

  // printf("cell size: %f\n",m_dCellSize);
  // printf("domain extends : %f %f %f\n",boundingBox.m_Extends[0],boundingBox.m_Extends[1],boundingBox.m_Extends[2]);
  // printf("element extends : %f %f %f\n",element.m_Extends[0],element.m_Extends[1],element.m_Extends[2]);      
  // printf("Dimensions : %d %d %d\n",x,y,z);      
  // printf("Number of cells in uniform grid : %d\n",x*y*z);    
  
  m_iDimension[0] = x;
  m_iDimension[1] = y;
  m_iDimension[2] = z;
    
}

template<class T, class CellType>
void CUniformGrid<T,CellType>::InitGrid(const CAABB3<T> &boundingBox, T cellSize)
{
   
  m_dCellSize = cellSize;
  m_bxBox = boundingBox;
    
  int x = (2.0*m_bxBox.m_Extends[0]+m_dCellSize)/m_dCellSize;
  int y = (2.0*m_bxBox.m_Extends[1]+m_dCellSize)/m_dCellSize;
  int z = (2.0*m_bxBox.m_Extends[2]+m_dCellSize)/m_dCellSize;

  // int nGhostLayerCells = (2 * xy + 2 * xz + 2 * yz) + (4 * x + 4 * y + 4 * z) + 8;

  // pass a bounding box that is m_dCellSize bigger in each dimension
  m_pCells = new CellType[x*y*z];

  // printf("cell size: %f\n",m_dCellSize);
  // printf("domain extends : %f %f %f\n",boundingBox.m_Extends[0],boundingBox.m_Extends[1],boundingBox.m_Extends[2]);
  // printf("element extends : %f %f %f\n",element.m_Extends[0],element.m_Extends[1],element.m_Extends[2]);      
  // printf("Dimensions : %d %d %d\n",x,y,z);      
  // printf("Number of cells in uniform grid : %d\n",x*y*z);    
  
  m_iDimension[0] = x;
  m_iDimension[1] = y;
  m_iDimension[2] = z;
    
}

template<class T, class CellType>
void CUniformGrid<T,CellType>::Insert(int elementID, const CVector3<T> &center)
{
  
  CVector3<T> origin(m_bxBox.m_vCenter.x-m_bxBox.m_Extends[0],
                     m_bxBox.m_vCenter.y-m_bxBox.m_Extends[1],
                     m_bxBox.m_vCenter.z-m_bxBox.m_Extends[2]);
  
  // calculate the cell index in xyz notation
  T invCellSize = 1.0/m_dCellSize;
  int indexx = (int)(fabs(origin.x-center.x) * invCellSize);
  int indexy = (int)(fabs(origin.y-center.y) * invCellSize);
  int indexz = (int)(fabs(origin.z-center.z) * invCellSize);
  
  // convert to linear array index
  int index = indexz * m_iDimension[0]*m_iDimension[1] +
              indexy * m_iDimension[0] + indexx;
  
  if(index > m_iDimension[0]*m_iDimension[1]*m_iDimension[2])
  {
    printf("Index out of bounds: %d %d %d %d\n",index,indexx,indexy,indexz);
    printf("dimensions: %d %d %d %d\n",m_iDimension[0]*m_iDimension[1]*m_iDimension[2],
	                               m_iDimension[0],m_iDimension[1],m_iDimension[2]);
    printf("origin: %f %f %f\n",origin.x,origin.y,origin.z);
    printf("center: %f %f %f\n",center.x,center.y,center.z);    
    //printf("");
    exit(0);
  }
  
  m_pCells[index].Insert(elementID);
	      
}

template<class T, class CellType>
CUniformGrid<T,CellType>::~CUniformGrid()
{

  if(m_pCells != NULL)
  {
    delete[] m_pCells;
    m_pCells = NULL;
  }

}

template<class T, class CellType>
void CUniformGrid<T,CellType>::Query(CRigidBody *body)
{

  //compute max overlap at level
  //the max overlap at a level is the maximum distance, 
  //that an object in the neighbouring cell can penetrate
  //into the cell under consideration
  T overlaplevel = 0.5 * m_dCellSize;
  T delta = body->GetBoundingSphereRadius() + overlaplevel;
  T invCellSize = 1.0/m_dCellSize;  
  
  CVector3<T> origin(m_bxBox.m_vCenter.x-m_bxBox.m_Extends[0],
                     m_bxBox.m_vCenter.y-m_bxBox.m_Extends[1],
                     m_bxBox.m_vCenter.z-m_bxBox.m_Extends[2]);
    
  int x0=std::min<int>(int(std::max<T>((body->m_vCOM.x-delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
  int y0=std::min<int>(int(std::max<T>((body->m_vCOM.y-delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);
  int z0=std::min<int>(int(std::max<T>((body->m_vCOM.z-delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);

  int x1=std::min<int>(int(std::max<T>((body->m_vCOM.x+delta-origin.x) * invCellSize,0.0)),m_iDimension[0]-1);   
  int y1=std::min<int>(int(std::max<T>((body->m_vCOM.y+delta-origin.y) * invCellSize,0.0)),m_iDimension[1]-1);   
  int z1=std::min<int>(int(std::max<T>((body->m_vCOM.z+delta-origin.z) * invCellSize,0.0)),m_iDimension[2]-1);   

  //loop over the overlapped cells
  for(int x=x0;x<=x1;x++)
    for(int y=y0;y<=y1;y++)
      for(int z=z0;z<=z1;z++)
      {
        std::list<int>::iterator i;
        int index = z*m_iDimension[1]*m_iDimension[0]+y*m_iDimension[0]+x;
        for(i=m_pCells[index].m_lElements.begin();i!=m_pCells[index].m_lElements.end();i++)
        {
	        int ielem = (*i);
          // push the potentially intersected element into the list
          body->m_iElements.push_back(ielem);
        }
      }//for z  
      
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CUniformGrid<Real,CUGCell>;

//----------------------------------------------------------------------------
}
