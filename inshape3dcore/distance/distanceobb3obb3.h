/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#ifndef DISTANCEOBB3OBB3_H
#define DISTANCEOBB3OBB3_H

//===================================================
//					DEFINITIONS
//===================================================


//===================================================
//					INCLUDES
//===================================================
#include <iostream>
#include <vector>
#include <limits>
#include <vector3.h>
#include <rigidbody.h>
#include <distance.h>
#include <segment3.h>
#include <rectangle3.h>
#include <distancetools.h>

namespace i3d {

/**
* @brief The class represents a transformation
*
*/ 
template <typename T>
class Transform
{
public:
	Transform(){};
	~Transform(){};

	Matrix3x3<T> m_pMatrix;
	Vector3<T> m_pOrigin;
};


/**
* @brief Computes the distance between two OBBs in 3d
*
* Computes the distance between two OBBs in 3d
*
*/ 
template <typename T>
class CDistanceOBB3OBB3 : public CDistance<T>
{
public:
	CDistanceOBB3OBB3(void);
	CDistanceOBB3OBB3(OBB3<T> &pBox0,OBB3<T> &pBox1);
	~CDistanceOBB3OBB3(void);

	T ComputeDistanceSqr();
	T ComputeDistance();

	OBB3<T> *m_pBox0;
	OBB3<T> *m_pBox1;

	OBB3<T> m_pBoxTrans0;
	OBB3<T> m_pBoxTrans1;

  CObjConfiguration<T> m_ocConf;

	enum
	{
		VERTEX=1,
		EDGE,
		FACE,
		INNER
	};

	using CDistance<T>::m_vClosestPoint0;
	using CDistance<T>::m_vClosestPoint1;

/// @cond HIDDEN_SYBOLS
	class CmpPairs
	{
	public:
		bool operator() (std::pair<T,unsigned int> p1,std::pair<T,unsigned int> p2) {return (p1.first < p2.first);};
	};
/// @cond
private:
	inline int GetRegionType(unsigned int regionCode)
	{
		int count = 0;
		while(regionCode)
		{
			//AND with 0x01 and add up
			count += regionCode & 0x1u;
			//shift the bit away
			regionCode >>= 1;
		}
		return (4-count);
	}

  inline Vector3<T> GetFaceNormal(unsigned int iRegion, int iwhich);

  void GetFace(unsigned int iRegion, int iwhich, Vector3<T> vVerts[4]);

	inline int GetBitcount(unsigned int inumber)
	{
		int count = 0;
		while(inumber)
		{
			//AND with 0x01 and add up
			count += inumber & 0x1u;
			//shift the bit away
			inumber >>= 1;
		}
		return count;
	}

	//these routines return the VERTEX,EDGE,FACE corresponding to iRegion of OBB iwhich
	//the routines assume that OBB iwhich has been transformed such that it
	//is 0-centered and axisparallel
	Vector3<T> GetRegionVertex(unsigned int iRegion, int iwhich);
	Segment3<T> GetRegionEdge(unsigned int iRegion, int iwhich);
	Rectangle3<T> GetRegionFace(unsigned int iRegion, int iwhich);

	void EdgeSplit(const Vector3<T> &vA, const Vector3<T> &vB, unsigned int ca, unsigned int cb, std::vector<unsigned int> &vRegions);

	void ComputeHalfPlaneIntersec(unsigned int s, int bcount, std::vector< std::pair<T,unsigned int> > &vCk, const Vector3<T> &vDir, const Vector3<T> &vA);

	void ClassifyVertices(unsigned int iRegions[8], Vector3<T> pVertices[8],int iwhich);

	unsigned int ClassifyVertex(const Vector3<T> &pVertex,int iwhich);

	Vector3<T> m_pVertices1[8];
	Vector3<T> m_pVertices0[8];


};

}

#endif
