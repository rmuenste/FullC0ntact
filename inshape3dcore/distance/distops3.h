
#ifdef WIN32
#pragma once
#endif

#ifndef _CDISTOPS3_H_
#define _CDISTOPS3_H_

#include "3dmodel.h"
#include <ray3.h>

namespace i3d {
  
///@cond HIDDEN_SYMBOLS
typedef struct
{
	VECTOR3 vVec;
	Real    rDistance;
}Res_t;

/*
typedef struct
{
	VECTOR3		vClosestPoint;
	Real	    rDistance;
	AABBNode3f	*pBestNode;
	std::vector<AABBNode3f*> lBFS;
}t_Collection;
*/

typedef struct
{
	bool InOut;
	int  Intersections;
}t_InnerPoints;

typedef struct
{
	bool bInOut;
	int  nIntersections;
	int  nBoxesHit;
	//std::vector<CTriangle3f*> vecTri;
}t_InnerCollection;
///@cond

/**
*  @brief Class for various distance computations and point classification methods 
*
* This class contains the algorithms for
* distance calculations and point classification.
* In this context Point classification means that
* given a 3d object and a 3d mesh we want to decide
* for all vertices of the grid if a vertex is 'inside'
* of an object or if a vertex is located on the outside
* of the object.
*
* We have al collection of distance computation algorithms
* and point classification algorithms.
* 
*/
class CDistOps3 

{
public:

	/*!
	*
	* This is the class constructor, it just handles initialisations
	*
	*/
	CDistOps3(void);

	/*!
	* The deconstructor, as usuals wqe clean up all our stuff
	* here so we dont leave a filthy, ugly mess...
	*/
	~CDistOps3(void);

	/*!
	* Distance calculation by brute force this is supposed to be the
	* slowest of all algorithms.
	*/
	Real BruteForceDistance(C3DModel &model, const Vector3f &vQuery) const;
	/*!
	* A more sophisticated method of calculating the inner points 
	*/
	//bool InnerPoints(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery);
	///*!
	//* The brute force point classification algorithm, it does make
	//* use of any special data structures so it is the slowest
	//* but a parallel version exists.
	//*/
	int BruteForceInnerPoints(C3DModel &model, const VECTOR3 &vQuery);
	
	//* The brute force point classification algorithm, it does make
	//* use of any special data structures so it is the slowest
	//* but a parallel version exists.This routine should be used with 
	//* objects that do not move, i.e. static objects
	//*/
	int BruteForceInnerPointsStatic(const C3DModel &model, const VECTOR3 &vQuery);
	/*!
	* The first version of the BAB type algorithms, this is just
	* the basic version without any special accerelation techniques
	*/
	//Real SimpleLUB(AABBTree3f &tree, const VECTOR3 &vQuery);
	//
	///*!
	//* An accelerated version of the bab algorithm, here we use coherency
	//* information, this version should give the best results with
	//* regard to computational speed
	//*/
	//Res_t CoherencyLUB(const AABBTree3f &tree, const VECTOR3 &vQuery, Real rLUB);
	///*!
	//* A helper routine that calculated the distance between a triangle and a point
	//* in 3d space.
	//*/
	//Real DistTriPoint(vector<VECTOR3*>& vVerts, const VECTOR3 &vQuery, VECTOR3& vNearest);
	///*!
	//* In this debug version of the simple BAB algorithm we calculate additional
	//* output information, to provide better algorithm analysis
	//*/
	//t_Collection SimpleLUBDebug(AABBTree3f &tree, const VECTOR3 &vQuery);
	///*!
	//* This is the debug version of the simple InnerPoints algorithm
	//* We get additional output parameters to analyze the algorithm
	//*/
	//t_InnerPoints DebugBruteForceInnerPoints(CModel3D &model, VECTOR3& vQuery);
	///*!
	//* The BVH version of the inner points algorithm
	//*/
	//bool InnerPointsBVH(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery);

	///*!
	//* The BVH version of the inner points algorithm
	//*/
	//bool InnerPointsList(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery);
	///*!
	//* The BVH version of the inner points algorithm
	//*/
	//t_InnerCollection InnerPointsListDebug(AABBTree3f &tree, CModel3D & model, VECTOR3& vQuery);

	///*!
	//* The BVH
	//*/
	//bool InnerPointsHPC(CModel3D &model, VECTOR3 &vQuery);

	int BruteForcefbm(const C3DModel &model, const VECTOR3 &vQuery, Ray3<Real> ray3);
	
	inline Real SphereSphere(VECTOR3 vC1,VECTOR3 vC2,Real r1, Real r2)
	{
	  return VECTOR3::createVector(vC1,vC2).mag();
	}
	
};

}

#endif
