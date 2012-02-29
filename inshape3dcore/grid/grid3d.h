/****************************************************************************
**
** Copyright (C) 2005-2007 Trolltech ASA. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/

#ifdef WIN32
#pragma once
#endif

#ifndef _GRID3D_H
#define _GRID3D_H

//===================================================
//					DEFINITIONS
//===================================================

//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include "aabb3.h"

namespace i3d {

//===================================================
//					Class
//===================================================
/// @cond HIDDEN_SYBOLS
class MyTraits
{
public:
	float distance;
	bool  Inside;
	float m_LUB;
	float m_TC;
};

template<class T,class VTraits = MyTraits>
class CGrid3D
{
public:
	CGrid3D(void);
	~CGrid3D(void);

	//member functions

	//initializes a grid via a bounding box
	void InitGrid(const CVector3<T> &vBL, const CVector3<T> &vTR, int iX, int iY, int iZ);

	//computes the maximum distance value in the grid
	void GetMaxDistance();

	//return the maximum distance
	inline T GetMaximumDistance() {return m_nMaxDistance;};

	//builds the texture coords for grid visualization
	void BuildTCoords(int iSlice);

	//inline member functions
	inline int GetSizeX() {return m_iSizeX;};
	inline int GetSizeY() {return m_iSizeY;};
	inline int GetSizeZ() {return m_iSizeZ;};
	inline int Size() {return m_iSizeX * m_iSizeY * m_iSizeZ;}

	//provides a bounding box for the grid
	inline CAABB3<T> GetBoundingBox() { return CAABB3<T>(m_pVertices[0], m_pVertices[Size()-1]);};

   inline CVector3<T>& operator() (int row, int col, int slice)
   {
       return m_pVertices[m_iSizeX * row + col + slice * m_iSizeX * m_iSizeY];
   }
  
   inline CVector3<T>& operator() (int row, int col, int slice) const
   {
       return m_pVertices[m_iSizeX * row + col + slice * m_iSizeX * m_iSizeY];
   }

   inline CVector3<T>& operator() (int index) const
   {
       return m_pVertices[index];
   }

   inline CVector3<T>& GetVertex(int index) const
   {
       return m_pVertices[index];
   }   


  inline CVector3<T>* GetVertices() {return m_pVertices;};

    /*!
        \fn CGrid::SetInOut(int i, int j, int k, Real d)
     */
    inline void SetInOut(int i, int j, int k, Real d)
    {
        m_pGrid[m_iSizeX * i + j + k * m_iSizeX * m_iSizeY].distance = d;
    }

    /*!
        \fn CGrid::SetInOut(int i, bool d)
     */
    inline void SetInOut(int i, bool d)
    {
        m_pGrid[i].Inside = d;
    }

    /*!
        \fn CGrid::GetInOut(int i)
     */
    inline bool GetInOut(int i)
    {
        return m_pGrid[i].Inside;
    }

    /*!
        \fn CGrid::SetDistance(int i, int j, int k, Real d)
     */
    inline void SetDistance(int i, int j, int k, Real d)
    {
        m_pGrid[m_iSizeX * i + j + k * m_iSizeX * m_iSizeY].distance = d;
    }

    /*!
        \fn CGrid::Distance(int i, int j, int k)
     */
    inline Real Distance(int i, int j, int k) const
    {
        return m_pGrid[m_iSizeX * i + j + k * m_iSizeX * m_iSizeY].distance;
    }

	inline void SetTCoord(int i, int j, int iSlice, T t1)
	{
		m_pGrid[m_iSizeX * i + j + iSlice * m_iSizeX * m_iSizeY].m_TC = t1;
	}//end SetTCoord

	VTraits *m_pGrid;

	inline Real TCoord(int i, int j, int k){return m_pGrid[m_iSizeX * i + j + k * m_iSizeX * m_iSizeY].m_TC;};

	void ToString();

private:

	inline void SetSize(int iX, int iY, int iZ) {m_iSizeX = iX; m_iSizeY = iY; m_iSizeZ = iZ;};

	int m_iSizeX;
	int m_iSizeY;
	int m_iSizeZ;
	CVector3<T> *m_pVertices;
	T   m_nMaxDistance;

};

template<class T,class MyTraits>
void CGrid3D<T, MyTraits>::ToString()
{
  for(int i=0;i<Size();i++)
  {
	std::cout<<"Number: "<<i<<" "<<m_pVertices[i]<<std::endl;
  }//end for
}//end ToString

template<class T,class MyTraits>
CGrid3D<T, MyTraits>::CGrid3D(void) : m_pVertices(0), m_pGrid(0)
{

	//Default initialisation
	m_iSizeX = 0;
	m_iSizeY = 0;
	m_iSizeZ = 0;

}//end constructor

template<class T,class MyTraits>
CGrid3D<T, MyTraits>::~CGrid3D(void)
{
	if(m_pVertices)
	{
		delete[] m_pVertices;
		m_pVertices = NULL;
	}//end if

	if(m_pGrid)
	{
		delete[] m_pGrid;
		m_pGrid = NULL;
	}//end if

}//end deconstructor

template<class T,class MyTraits>
void CGrid3D<T, MyTraits>::BuildTCoords(int iSlice)
{

	//make texture coordinates
	for(int i = 0; i < m_iSizeX; i++)
	{
				
		for(int j = 0; j < m_iSizeY; j++)
		{
			//scale to 0.0 - 1.0
			T sc = Distance(i,j, iSlice) / m_nMaxDistance;
			T t1 = (sc < 0.0001) ? 0.0 : sc;
			SetTCoord(i,j,iSlice,t1);
		}//end for
	}//end for

}//end BuildTCoords

template<class T,class MyTraits>
void CGrid3D<T,MyTraits>::GetMaxDistance()
{

	
	T max = -std::numeric_limits<T>::max();

	int allVertices = m_iSizeX * m_iSizeY * m_iSizeZ;
	for(int i = 0; i < allVertices; i++)
	{
	
		if(m_pGrid[i].distance > max)
		{
			max = m_pGrid[i].distance;
		}//end if

	}//end for

	m_nMaxDistance =  max;

}//end GetMaximumDistance

template<class T,class MyTraits>
void CGrid3D<T, MyTraits>::InitGrid(const CVector3<T> &vBL, const CVector3<T> &vTR, int iX, int iY, int iZ)
{

	//set the size of the grid
	SetSize(iX, iY, iZ);

	//calculate the total number of vertices
	int allVertices = m_iSizeX * m_iSizeY * m_iSizeZ;

	//allocate memory
	m_pGrid = new MyTraits[allVertices];

	int iSlice = m_iSizeX * m_iSizeY; 

	m_pVertices = new CVector3<T>[allVertices];

	Real LengthX =  fabs(vTR.x - vBL.x);
	Real LengthY =  fabs(vTR.y - vBL.y);
	Real LengthZ =  fabs(vTR.z - vBL.z);

	Real xInc = LengthX / Real(m_iSizeX-1);
	Real yInc = LengthY / Real(m_iSizeY-1);
	Real zInc = LengthZ / Real(m_iSizeZ-1);

	int count = 0;
	for(int i = 0; i < m_iSizeX; i++)
	{
		for(int j = 0; j < m_iSizeY; j++)
		{
			for(int z = 0; z <m_iSizeZ; z++)
			{
				m_pVertices[i*m_iSizeX + j + z * iSlice] = CVector3<T>(vBL.x + i*xInc, vBL.y + j*yInc, vBL.z + z * zInc);
				count = i*m_iSizeX + j + z * iSlice;
			}//end for
		}//end for
	}//end for

	ToString();

}//end InitGrid
/// @cond
}

#endif