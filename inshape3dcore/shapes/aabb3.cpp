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
CVector3<T> CAABB3<T>::GetVertex(int i)
{
	switch(i)
	{
		case 0:
			return GetFBL();
		break;
		case 1: 
			return GetFBR();
		break;
		case 2: 
			return GetBBR();
		break;
		case 3: 
			return GetBBL();
		break;
		case 4: 
			return GetFTL();
		break;
		case 5: 
			return GetFTR();
		break;
		case 6: 
			return GetBTR();
		break;
		case 7: 
			return GetBTL();
		break;		
	}
}

template<class T>
void CAABB3<T>::update(const CVector3<T> &vQuery)
{
	m_Verts[0].x += vQuery.x;
	m_Verts[0].y += vQuery.y; 
	m_Verts[0].z += vQuery.z;
	m_Verts[1].x += vQuery.x;
	m_Verts[1].y += vQuery.y; 
	m_Verts[1].z += vQuery.z;
}



template<class T>
bool CAABB3<T>::Inside(const CVector3<T> &vQuery) const
{
	if(  (Xmin() <= vQuery.x && vQuery.x <= Xmax())
	   &&(Ymin() <= vQuery.y && vQuery.y <= Ymax())
	   && (Zmin() <= vQuery.z && vQuery.z <= Zmax()) )
		return true;
	else
		return false;

}

template<class T>
CAABB3<T>::CAABB3(const CVector3<T> &vBL, const CVector3<T> &vTR)
{

	m_Verts[0] = vBL;
	m_Verts[1] = vTR;
	
	m_Extends[0] = fabs(vTR.x-vBL.x)*0.5;
	m_Extends[1] = fabs(vTR.y-vBL.y)*0.5;
	m_Extends[2] = fabs(vTR.z-vBL.z)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);
	

}//end constructor

template<class T>
int CAABB3<T>::LongestAxis() const
{
	T rLength = -std::numeric_limits<T>::max();

	int iAxis = -1;

	T lengths[3];

	lengths[0] = fabs(m_Verts[0].x - m_Verts[1].x);
	lengths[1] = fabs(m_Verts[0].y - m_Verts[1].y);
	lengths[2] = fabs(m_Verts[0].z - m_Verts[1].z);

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
CAABB3<T>::CAABB3(const CDynamicArray< CVector3<T> > &Vec3Array)
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
			m_Verts[0].x = Vec3Array[i].x;
		}

		if(Vec3Array[i].x < MaxX)
		{	//assign max index
			m_Verts[1].x = Vec3Array[i].x;
		}

		if(Vec3Array[i].y < MinY)
		{	//assign min index
			m_Verts[0].y = Vec3Array[i].y;
		}

		if(Vec3Array[i].y < MaxY)
		{	//assign max index
			m_Verts[1].y = Vec3Array[i].y;
		}

		if(Vec3Array[i].z < MinZ)
		{	//assign min index
			m_Verts[0].z = Vec3Array[i].z;
		}

		if(Vec3Array[i].z < MaxZ)
		{	//assign max index
			m_Verts[1].z = Vec3Array[i].z;
		}

	}//end for

}//end constructor

template<class T>
void CAABB3<T>::InitBox(const CDynamicArray< CVector3<T> > &Vec3Array)
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

	m_Verts[0].x = MinX;
	m_Verts[0].y = MinY;
	m_Verts[0].z = MinZ;

	m_Verts[1].x = MaxX;
	m_Verts[1].y = MaxY;
	m_Verts[1].z = MaxZ;

	m_Extends[0] = fabs(MaxX-MinX)*0.5;
	m_Extends[1] = fabs(MaxY-MinY)*0.5;
	m_Extends[2] = fabs(MaxZ-MinZ)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);


}//end InitBox

template<class T>
void CAABB3<T>::Init(T minX,T minY,T minZ,T maxX,T maxY,T maxZ)
{
	m_Verts[0] = CVector3<T>(minX,minY,minZ);

	m_Verts[1] = CVector3<T>(maxX,maxY,maxZ);
	
	m_Extends[0] = fabs(maxX-minX)*0.5;
	m_Extends[1] = fabs(maxY-minY)*0.5;
	m_Extends[2] = fabs(maxZ-minZ)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);
	
	
}//end InitBox

template<class T>
void CAABB3<T>::Init(const CVector3<T> &minVec, const CVector3<T> &maxVec)
{
	m_Verts[0] = minVec;

	m_Verts[1] = maxVec;

	m_Extends[0] = fabs(maxVec.x-minVec.x)*0.5;
	m_Extends[1] = fabs(maxVec.y-minVec.y)*0.5;
	m_Extends[2] = fabs(maxVec.z-minVec.z)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);
}//end InitBox

template<class T>
void CAABB3<T>::SetBox(CVector3<T> minVec, CVector3<T> maxVec)
{
	m_Verts[0].x = minVec.x;
	m_Verts[0].y = minVec.y;
	m_Verts[0].z = minVec.z;

	m_Verts[1].x = maxVec.x;
	m_Verts[1].y = maxVec.y;
	m_Verts[1].z = maxVec.z;

	m_Extends[0] = fabs(maxVec.x-minVec.x)*0.5;
	m_Extends[1] = fabs(maxVec.y-minVec.y)*0.5;
	m_Extends[2] = fabs(maxVec.z-minVec.z)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);

}//end InitBox

template<class T>
T CAABB3<T>::MinDistance(const CVector3<T> &vQuery)
{

	CVector3<T> vSol;

  if(Inside(vQuery))
    return T(0);

	if(vQuery.x < Xmin())
		vSol.x = Xmin()-vQuery.x;
	else if(vQuery.x > Xmax())
		vSol.x = vQuery.x - Xmax();
	else
		vSol.x = 0.0;

	if(vQuery.y < Ymin())
		vSol.y = Ymin()-vQuery.y;
	else if(vQuery.y > Ymax())
		vSol.y = vQuery.y - Ymax();
	else
		vSol.y = 0.0;

	if(vQuery.z < Zmin())
		vSol.z = Zmin()-vQuery.z;
	else if(vQuery.z > Zmax())
		vSol.z = vQuery.z - Zmax();
	else
		vSol.z = 0.0;

	return vSol.mag();

}//end MinDistance

template<class T>
CVector3<T> CAABB3<T>::MinDistanceDebug(const CVector3<T> &vQuery)
{

	CVector3<T> vSol;

	if(vQuery.x < Xmin())
		vSol.x = Xmin()-vQuery.x;
	else if(vQuery.x > Xmax())
		vSol.x = vQuery.x - Xmax();
	else
		vSol.x = 0.0;

	if(vQuery.y < Ymin())
		vSol.y = Ymin()-vQuery.y;
	else if(vQuery.y > Ymax())
		vSol.y = vQuery.y - Ymax();
	else
		vSol.y = 0.0;

	if(vQuery.z > Zmin())
		vSol.z = Zmax()-vQuery.z;
	else if(vQuery.z < Zmax())
		vSol.z = vQuery.z - Zmin();
	else
		vSol.z = 0.0;
	return vSol;

}//end MinDistance


template<class T>
T CAABB3<T>::MinDistanceSqr(const CVector3<T> &vQuery)
{

	CVector3<T> vSol;

	if(vQuery.x < Xmin())
		vSol.x = Xmin()-vQuery.x;
	else if(vQuery.x > Xmax())
		vSol.x = vQuery.x - Xmax();
	else
		vSol.x = 0.0;

	if(vQuery.y < Ymin())
		vSol.y = Ymin()-vQuery.y;
	else if(vQuery.y > Ymax())
		vSol.y = vQuery.y - Ymax();
	else
		vSol.y = 0.0;

	if(vQuery.z < Zmin())
		vSol.z = Zmin()-vQuery.z;
	else if(vQuery.y > Ymax())
		vSol.z = vQuery.z - Zmax();
	else
		vSol.z = 0.0;

	return vSol.norm2();

}//end MinDistanceSqr

template<class T>
void CAABB3<T>::Init(const std::vector<CTriangle3<T> > &vTriangles)
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
		const CTriangle3<T> &tri = vTriangles[i];

		for(int j = 0; j < 3; j++)
		{
			CVector3<T> Vec3 = tri.Get(j);
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

			//T d = CVector3<T>::createVector(Vec3,vCenter).mag();
			//if( d < MinCenter)
			//{
			//	m_vUpper = Vec3;
			//	MinCenter = d;
			//}

		}//end for j
	}//end for

	m_Verts[0].x = MinX;
	m_Verts[0].y = MinY;
	m_Verts[0].z = MinZ;

	m_Verts[1].x = MaxX;
	m_Verts[1].y = MaxY;
	m_Verts[1].z = MaxZ;

	m_Extends[0] = fabs(m_Verts[1].x-m_Verts[0].x)*0.5;
	m_Extends[1] = fabs(m_Verts[1].y-m_Verts[0].y)*0.5;
	m_Extends[2] = fabs(m_Verts[1].z-m_Verts[0].z)*0.5;
	
	m_vCenter = CVector3<T>(m_Verts[0].x+m_Extends[0],m_Verts[0].y+m_Extends[1],m_Verts[0].z+m_Extends[2]);

}//end InitBox

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CAABB3<Real>;
template class CAABB3<float>;

//----------------------------------------------------------------------------

}
