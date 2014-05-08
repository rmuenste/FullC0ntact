/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "aabb3.h"

namespace i3d {

template<class T>
Vector3<T> AABB3<T>::getVertex(int i)
{
	switch(i)
	{
		case 0:
			return getFBL();
		break;
		case 1: 
			return getFBR();
		break;
		case 2: 
			return getBBR();
		break;
		case 3: 
			return getBBL();
		break;
		case 4: 
			return getFTL();
		break;
		case 5: 
			return getFTR();
		break;
		case 6: 
			return getBTR();
		break;
		case 7: 
			return getBTL();
		break;		
	}
}

template<class T>
void AABB3<T>::update(const Vector3<T> &vQuery)
{
	vertices_[0].x += vQuery.x;
	vertices_[0].y += vQuery.y; 
	vertices_[0].z += vQuery.z;
	vertices_[1].x += vQuery.x;
	vertices_[1].y += vQuery.y; 
	vertices_[1].z += vQuery.z;
}



template<class T>
bool AABB3<T>::isPointInside(const Vector3<T> &query) const
{
  if(  (xmin() <= query.x && query.x <= xmax())
     &&(ymin() <= query.y && query.y <= ymax())
     && (zmin() <= query.z && query.z <= zmax()) )
		return true;
	else
		return false;

}

template<class T>
AABB3<T>::AABB3(const Vector3<T> &vBL, const Vector3<T> &vTR)
{

	vertices_[0] = vBL;
	vertices_[1] = vTR;
	
	extents_[0] = fabs(vTR.x-vBL.x)*0.5;
	extents_[1] = fabs(vTR.y-vBL.y)*0.5;
	extents_[2] = fabs(vTR.z-vBL.z)*0.5;
	
	center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);
	

}//end constructor

template<class T>
int AABB3<T>::longestAxis() const
{
	T rLength = -std::numeric_limits<T>::max();

	int iAxis = -1;

	T lengths[3];

	lengths[0] = fabs(vertices_[0].x - vertices_[1].x);
	lengths[1] = fabs(vertices_[0].y - vertices_[1].y);
	lengths[2] = fabs(vertices_[0].z - vertices_[1].z);

	for(int i = 0; i < 3; i++)
	{
		if(rLength < lengths[i])
		{
			iAxis = i;
			rLength = lengths[i];
		}//end if
	}//end for

	return iAxis;

}//end LongestAxis

template<class T>
AABB3<T>::AABB3(const CDynamicArray< Vector3<T> > &Vec3Array)
{
	T MaxX = std::numeric_limits<T>::min();
	T MinX = std::numeric_limits<T>::max();
	T MaxY = std::numeric_limits<T>::min();
	T MinY = std::numeric_limits<T>::max();
	T MaxZ = std::numeric_limits<T>::min();
	T MinZ = std::numeric_limits<T>::max();

	for(int i = 0; i < Vec3Array.Size(); i++)
	{

		if(Vec3Array[i].x < MinX)
		{	//assign min index
			vertices_[0].x = Vec3Array[i].x;
		}

		if(Vec3Array[i].x < MaxX)
		{	//assign max index
			vertices_[1].x = Vec3Array[i].x;
		}

		if(Vec3Array[i].y < MinY)
		{	//assign min index
			vertices_[0].y = Vec3Array[i].y;
		}

		if(Vec3Array[i].y < MaxY)
		{	//assign max index
			vertices_[1].y = Vec3Array[i].y;
		}

		if(Vec3Array[i].z < MinZ)
		{	//assign min index
			vertices_[0].z = Vec3Array[i].z;
		}

		if(Vec3Array[i].z < MaxZ)
		{	//assign max index
			vertices_[1].z = Vec3Array[i].z;
		}

	}//end for

}//end constructor

template<class T>
void AABB3<T>::initBox(const CDynamicArray< Vector3<T> > &Vec3Array)
{

	T MaxX = -std::numeric_limits<T>::max();
	T MinX = std::numeric_limits<T>::max();
	T MaxY = -std::numeric_limits<T>::max();
	T MinY = std::numeric_limits<T>::max();
	T MaxZ = -std::numeric_limits<T>::max();
	T MinZ = std::numeric_limits<T>::max();

	for(int i = 0; i < Vec3Array.Size(); i++)
	{

		if(Vec3Array[i].x < MinX)
		{	//assign min index
			MinX = Vec3Array[i].x;
		}

		if(Vec3Array[i].x > MaxX)
		{	//assign max index
			MaxX = Vec3Array[i].x;
		}

		if(Vec3Array[i].y < MinY)
		{	//assign min index
			MinY = Vec3Array[i].y;
		}

		if(Vec3Array[i].y > MaxY)
		{	//assign max index
			MaxY = Vec3Array[i].y;
		}

		if(Vec3Array[i].z < MinZ)
		{	//assign min index
			MinZ = Vec3Array[i].z;
		}

		if(Vec3Array[i].z > MaxZ)
		{	//assign max index
			MaxZ = Vec3Array[i].z;
		}

	}//end for

	vertices_[0].x = MinX;
	vertices_[0].y = MinY;
	vertices_[0].z = MinZ;

	vertices_[1].x = MaxX;
	vertices_[1].y = MaxY;
	vertices_[1].z = MaxZ;

	extents_[0] = fabs(MaxX-MinX)*0.5;
	extents_[1] = fabs(MaxY-MinY)*0.5;
	extents_[2] = fabs(MaxZ-MinZ)*0.5;
	
	center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);


}//end InitBox

template<class T>
void AABB3<T>::init(T minX,T minY,T minZ,T maxX,T maxY,T maxZ)
{
	vertices_[0] = Vector3<T>(minX,minY,minZ);

	vertices_[1] = Vector3<T>(maxX,maxY,maxZ);
	
	extents_[0] = fabs(maxX-minX)*0.5;
	extents_[1] = fabs(maxY-minY)*0.5;
	extents_[2] = fabs(maxZ-minZ)*0.5;
	
	center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);
	
	
}//end InitBox

template<class T>
void AABB3<T>::init(const Vector3<T> &minVec, const Vector3<T> &maxVec)
{
	vertices_[0] = minVec;

	vertices_[1] = maxVec;

	extents_[0] = fabs(maxVec.x-minVec.x)*0.5;
	extents_[1] = fabs(maxVec.y-minVec.y)*0.5;
	extents_[2] = fabs(maxVec.z-minVec.z)*0.5;
	
	center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);
}//end InitBox

template<class T>
void AABB3<T>::setBox(Vector3<T> minVec, Vector3<T> maxVec)
{
	vertices_[0].x = minVec.x;
	vertices_[0].y = minVec.y;
	vertices_[0].z = minVec.z;

	vertices_[1].x = maxVec.x;
	vertices_[1].y = maxVec.y;
	vertices_[1].z = maxVec.z;

	extents_[0] = fabs(maxVec.x-minVec.x)*0.5;
	extents_[1] = fabs(maxVec.y-minVec.y)*0.5;
	extents_[2] = fabs(maxVec.z-minVec.z)*0.5;
	
	center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);

}//end InitBox

template<class T>
T AABB3<T>::minDistance(const Vector3<T> &query)
{

  Vector3<T> sol;

  if(isPointInside(query))
    return T(0);

  if(query.x < xmin())
    sol.x = xmin()-query.x;
  else if(query.x > xmax())
    sol.x = query.x - xmax();
  else
    sol.x = 0.0;

  if(query.y < ymin())
    sol.y = ymin()-query.y;
  else if(query.y > ymax())
    sol.y = query.y - ymax();
  else
    sol.y = 0.0;

  if(query.z < zmin())
    sol.z = zmin()-query.z;
  else if(query.z > zmax())
    sol.z = query.z - zmax();
  else
    sol.z = 0.0;

  return sol.mag();

}//end MinDistance

template<class T>
Vector3<T> AABB3<T>::minDistanceDebug(const Vector3<T> &query)
{

  Vector3<T> sol;

  if(query.x < xmin())
    sol.x = xmin()-query.x;
  else if(query.x > xmax())
    sol.x = query.x - xmax();
  else
    sol.x = 0.0;

  if(query.y < ymin())
    sol.y = ymin()-query.y;
  else if(query.y > ymax())
    sol.y = query.y - ymax();
  else
    sol.y = 0.0;

  if(query.z > zmin())
    sol.z = zmax()-query.z;
  else if(query.z < zmax())
    sol.z = query.z - zmin();
  else
    sol.z = 0.0;
  return sol;

}//end MinDistance


template<class T>
T AABB3<T>::minDistanceSqr(const Vector3<T> &query)
{

  Vector3<T> sol;

  if(query.x < xmin())
    sol.x = xmin()-query.x;
  else if(query.x > xmax())
    sol.x = query.x - xmax();
  else
    sol.x = 0.0;

  if(query.y < ymin())
    sol.y = ymin()-query.y;
  else if(query.y > ymax())
    sol.y = query.y - ymax();
  else
    sol.y = 0.0;

  if(query.z < zmin())
    sol.z = zmin()-query.z;
  else if(query.y > ymax())
    sol.z = query.z - zmax();
  else
    sol.z = 0.0;

  return sol.norm2();

}//end MinDistanceSqr

template<class T>
void AABB3<T>::init(const std::vector<Triangle3<T> > &vTriangles)
{
  T MaxX = -std::numeric_limits<T>::max();
  T MinX = std::numeric_limits<T>::max();
  T MaxY = -std::numeric_limits<T>::max();
  T MinY = std::numeric_limits<T>::max();
  T MaxZ = -std::numeric_limits<T>::max();
  T MinZ = std::numeric_limits<T>::max();

  T MinCenter = std::numeric_limits<T>::max();

  for(int i = 0; i < vTriangles.size(); i++)
  {
    const Triangle3<T> &tri = vTriangles[i];

    for(int j = 0; j < 3; j++)
    {
      Vector3<T> Vec3 = tri.Get(j);
      if(Vec3.x < MinX)
      {	//assign min index
        MinX = Vec3.x;
      }

      if(Vec3.x > MaxX)
      {	//assign max index
        MaxX = Vec3.x;
      }

      if(Vec3.y < MinY)
      {	//assign min index
        MinY = Vec3.y;
      }

      if(Vec3.y > MaxY)
      {	//assign max index
        MaxY = Vec3.y;
      }

      if(Vec3.z < MinZ)
      {	//assign min index
        MinZ = Vec3.z;
      }

      if(Vec3.z > MaxZ)
      {	//assign max index
        MaxZ = Vec3.z;
      }

      //T d = Vector3<T>::createVector(Vec3,vCenter).mag();
      //if( d < MinCenter)
      //{
      //	m_vUpper = Vec3;
      //	MinCenter = d;
      //}

    }//end for j
  }//end for

  vertices_[0].x = MinX;
  vertices_[0].y = MinY;
  vertices_[0].z = MinZ;

  vertices_[1].x = MaxX;
  vertices_[1].y = MaxY;
  vertices_[1].z = MaxZ;

  extents_[0] = fabs(vertices_[1].x-vertices_[0].x)*0.5;
  extents_[1] = fabs(vertices_[1].y-vertices_[0].y)*0.5;
  extents_[2] = fabs(vertices_[1].z-vertices_[0].z)*0.5;

  center_ = Vector3<T>(vertices_[0].x+extents_[0],vertices_[0].y+extents_[1],vertices_[0].z+extents_[2]);

}//end InitBox

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class AABB3<Real>;
template class AABB3<float>;

//----------------------------------------------------------------------------

}
