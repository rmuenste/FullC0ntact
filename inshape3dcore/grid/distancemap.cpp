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
#include <list>
#include <aabb3.h>
#include "distancemap.h"

namespace i3d {

template <class T>
DistanceMap<T>::DistanceMap(const AABB3<T> &aabb)
{
  
  boundingBox_ = aabb;
  
  //32x32x32
  vertexCoords_ = NULL;
  normals_ = NULL;
  contactPoints_ = NULL;
  stateFBM_ = NULL;
  
  cellSize_ = (2.0*boundingBox_.extents_[0])/32.0f;
  
  cells_[0] = (2.0*boundingBox_.extents_[0])/cellSize_;
  cells_[1] = (2.0*boundingBox_.extents_[1])/cellSize_;
  cells_[2] = (2.0*boundingBox_.extents_[2])/cellSize_;

  int vx = cells_[0]+1;

  int vxy=vx*vx;
  
  int vxyz = vxy*vx;
  
  dim_[0]=vx;
  dim_[1]=vxy;
  
  vertexCoords_ = new CVector3<T>[vxyz];
  distance_ = new T[vxyz];
  stateFBM_ = new int[vxyz];
  normals_ = new CVector3<T>[vxyz];
  contactPoints_ = new CVector3<T>[vxyz];
  
  //generate the vertex coordinates
  for(int k=0;k<vx;k++)
  {  
    for(int j=0;j<vx;j++)
    {
      for(int i=0;i<vx;i++)
      {
        vertexCoords_[k*vxy+j*vx+i].x=boundingBox_.vertices_[0].x+i*cellSize_;
        vertexCoords_[k*vxy+j*vx+i].y=boundingBox_.vertices_[0].y+j*cellSize_;
        vertexCoords_[k*vxy+j*vx+i].z=boundingBox_.vertices_[0].z+k*cellSize_;
      }
    }
  }
}

template <class T>
DistanceMap<T>::~DistanceMap()
{
  if(vertexCoords_ != NULL)
  {
    delete[] vertexCoords_;
    vertexCoords_ = NULL;
  }
  if(distance_ != NULL)
  {
    delete[] distance_;
    distance_ = NULL;
  }
  if(normals_ != NULL)
  {
    delete[] normals_;
    normals_ = NULL;
  }
  if(contactPoints_ != NULL)
  {
    delete[] contactPoints_;
    contactPoints_ = NULL;
  }
  if(stateFBM_ != NULL)
  {
    delete[] stateFBM_;
    stateFBM_ = NULL;
  }        
  
}

template <class T>
void DistanceMap<T>::vertexIndices(int icellx,int icelly, int icellz, int indices[8])
{
  int baseIndex=icellz*dim_[1]+icelly*dim_[0]+icellx; 

  indices[0]=baseIndex;         //xmin,ymin,zmin
  indices[1]=baseIndex+1;       //xmax,ymin,zmin

  indices[2]=baseIndex+dim_[0]+1; //xmax,ymax,zmin
  indices[3]=baseIndex+dim_[0];   //xmin,ymax,zmin

  
  indices[4]=baseIndex+dim_[1];  
  indices[5]=baseIndex+dim_[1]+1;  

  indices[6]=baseIndex+dim_[0]+dim_[1]+1;  
  indices[7]=baseIndex+dim_[0]+dim_[1];

}

template <class T>
T DistanceMap<T>::trilinearInterpolateDistance(const CVector3<T> &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  T x_d= (vQuery.x - vertexCoords_[indices[0]].x)/(vertexCoords_[indices[1]].x - vertexCoords_[indices[0]].x);
  T y_d= (vQuery.y - vertexCoords_[indices[0]].y)/(vertexCoords_[indices[2]].y - vertexCoords_[indices[0]].y);
  T z_d= (vQuery.z - vertexCoords_[indices[0]].z)/(vertexCoords_[indices[4]].z - vertexCoords_[indices[0]].z);  
  
  T c00 = distance_[indices[0]] * (1.0 - x_d) + distance_[indices[1]] * x_d;
  
  T c10 = distance_[indices[3]] * (1.0 - x_d) + distance_[indices[2]] * x_d;
  
  T c01 = distance_[indices[4]] * (1.0 - x_d) + distance_[indices[5]] * x_d;
  
  T c11 = distance_[indices[7]] * (1.0 - x_d) + distance_[indices[6]] * x_d;      
  
  //next step
  T c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  T c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  T c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

template <class T>
CVector3<T> DistanceMap<T>::trilinearInterpolateCP(const CVector3<T> &vQuery, int indices[8])
{
  //trilinear interpolation of distance
  T x_d= (vQuery.x - vertexCoords_[indices[0]].x)/(vertexCoords_[indices[1]].x - vertexCoords_[indices[0]].x);
  T y_d= (vQuery.y - vertexCoords_[indices[0]].y)/(vertexCoords_[indices[2]].y - vertexCoords_[indices[0]].y);
  T z_d= (vQuery.z - vertexCoords_[indices[0]].z)/(vertexCoords_[indices[4]].z - vertexCoords_[indices[0]].z);  
  
  CVector3<T> c00 = contactPoints_[indices[0]] * (1.0 - x_d) + contactPoints_[indices[1]] * x_d;
  
  CVector3<T> c10 = contactPoints_[indices[3]] * (1.0 - x_d) + contactPoints_[indices[2]] * x_d;
  
  CVector3<T> c01 = contactPoints_[indices[4]] * (1.0 - x_d) + contactPoints_[indices[5]] * x_d;
  
  CVector3<T> c11 = contactPoints_[indices[7]] * (1.0 - x_d) + contactPoints_[indices[6]] * x_d;      
  
  //next step
  CVector3<T> c0 = c00*(1.0 - y_d) + c10 * y_d;
  
  CVector3<T> c1 = c01*(1.0 - y_d) + c11 * y_d;  
  
  //final step
  CVector3<T> c = c0*(1.0 - z_d) + c1 * z_d;  
  
  return c;
}

template <class T>
std::pair<T,CVector3<T> >  DistanceMap<T>::queryMap(const CVector3<T> &vQuery)
{
  T dist = T(1000.0);
  CVector3<T> normal;
  CVector3<T> center;
  CVector3<T> origin;  
  std::pair<T,CVector3<T> > res(dist,normal);
  
  if(!boundingBox_.isPointInside(vQuery))
  {
    return res;
  }
  
  //calculate the cell indices
  T invCellSize = 1.0/cellSize_;
  
  //bring to mesh origin
  
  //calculate the cell indices
  int x = (int)(fabs(vQuery.x-boundingBox_.vertices_[0].x) * invCellSize);
  int y = (int)(fabs(vQuery.y-boundingBox_.vertices_[0].y) * invCellSize);
  int z = (int)(fabs(vQuery.z-boundingBox_.vertices_[0].z) * invCellSize);  

//   T invCellSize = 1.0/cellSize_;
//   int x = (int)(fabs(origin.x-vQuery.x) * invCellSize);
//   int y = (int)(fabs(origin.y-vQuery.y) * invCellSize);
//   int z = (int)(fabs(origin.z-vQuery.z) * invCellSize);  
  
  //vertex indices
  int indices[8];
  
  //look up distance for the cell
  vertexIndices(x,y,z,indices);
   
  int index=-1;
  T mindist = CMath<T>::MAXREAL;
  dist = 1000.0;
  
//   for(int i=0;i<8;i++)
//   {
//     center+=vertexCoords_[indices[i]];
//     //std::cout<<"DistanceMapPoint: "<<vertexCoords_[indices[i]];
//     T d = (vertexCoords_[indices[i]] - vQuery).mag();
//     if(d < dist)
//     {
//       index=i;
//       dist=d;
//       //normal = normals_[indices[i]];
//       normal = contactPoints_[indices[i]];     
//       mindist=distance_[indices[i]];        
//     }  
//   }
  center=center*0.125;
//   std::cout<<"center: "<<center*0.125;
//   std::cout<<"size: "<<cellSize_<<std::endl;  
  
  res.first  = trilinearInterpolateDistance(vQuery,indices);
  res.second = trilinearInterpolateCP(vQuery,indices);
  
  //res.first=mindist;  
  //res.second=normal;
  
  return res;
}

template <class T>
void DistanceMap<T>::convertToUnstructuredGrid(CUnstrGridr& ugrid)
{
  int NEL=0;
  int NVT=0;
    
  int vx = cells_[0]+1;

  int vxy=vx*vx;
  
  int vxyz = vxy*vx;
    
  NVT=vxyz;
  
  NEL=cells_[0]*cells_[1]*cells_[2];
  
  ugrid.nvt_          = NVT;
  ugrid.nel_          = NEL;  
  ugrid.vertexCoords_ = new VECTOR3[NVT];
  ugrid.hexas_        = new Hexa[NEL];
  ugrid.m_myTraits      = new DTraits[NVT];
  
  //needed vertexcoords,hexas,traits
  int ive=0;

  
  //start with the highest level
  for(ive=0;ive<NVT;ive++)
  {
    ugrid.vertexCoords_[ive]=vertexCoords_[ive];
    ugrid.m_myTraits[ive].distance=distance_[ive];
    ugrid.m_myTraits[ive].iTag=1;
  }//end for  

  int iel=0;
  for(int ielz=0;ielz<cells_[2];ielz++)
    for(int iely=0;iely<cells_[1];iely++)    
      for(int ielx=0;ielx<cells_[0];ielx++)
      {
        vertexIndices(ielx,iely,ielz,ugrid.hexas_[iel++].hexaVertexIndices_);    
      }//end for    

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class DistanceMap<Real>;

//----------------------------------------------------------------------------

}

