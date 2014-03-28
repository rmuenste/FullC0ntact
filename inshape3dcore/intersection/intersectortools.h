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

#ifndef INTERSECTORTOOLS_H
#define INTERSECTORTOOLS_H


//===================================================
//                     INCLUDES
//===================================================
#include <obb3.h>
#include <string>
#include <sstream>
#include <cylinder.h>
#include <distanceconvexconvexgjk.h>

namespace i3d {

/**
* @brief A class for storing relative orientation for two geometric primitives
*
*
*/
template <class T>
class CProjCfg
{
public:

  enum
  {
    LEFT,
    RIGHT,
    NONE,
    OVERLAPMAX,
    OVERLAPMIN
  };

  enum
  {
    BOX_VERTEX,
    BOX_EDGE,
    BOX_FACE
  };

  int getFeature() const {return m_iFeature;};

  int getMinFeature() const {return m_iMinFeature;};

  void SetFeature(int feature) {m_iFeature = feature;};

  void SetMinFace(int a,int b, int c, int d)
  {
    m_iFeatureIndex[0] = a;
    m_iFeatureIndex[1] = b;
    m_iFeatureIndex[2] = c;
    m_iFeatureIndex[3] = d;
  };

  void SetMaxFace(int a,int b, int c, int d)
  {
    m_iFeatureIndex[4] = a;
    m_iFeatureIndex[5] = b;
    m_iFeatureIndex[6] = c;
    m_iFeatureIndex[7] = d;
  };

  void SetMinEdge(int a,int b)
  {
    m_iFeatureIndex[0] = a;
    m_iFeatureIndex[1] = b;
    m_iFeatureIndex[2] = T(0);
    m_iFeatureIndex[3] = T(0);
  };

  void SetMaxEdge(int a,int b)
  {
    m_iFeatureIndex[4] = a;
    m_iFeatureIndex[5] = b;
    m_iFeatureIndex[6] = T(0);
    m_iFeatureIndex[7] = T(0);
  };

  void SetMinVertex(int a)
  {
    m_iFeatureIndex[0] = a;
    m_iFeatureIndex[1] = T(0);
    m_iFeatureIndex[2] = T(0);
    m_iFeatureIndex[3] = T(0);
  };

  void SetMaxVertex(int a)
  {
    m_iFeatureIndex[4] = a;
    m_iFeatureIndex[5] = T(0);
    m_iFeatureIndex[6] = T(0);
    m_iFeatureIndex[7] = T(0);
  };

  std::string ToString() {

	std::string out;
	std::stringstream ss;
		
	ss<<"------------------------------------"<<std::endl;
  switch(m_iFeature) {
  case BOX_VERTEX:
	  ss<<"Feature: Vertex"<<std::endl;
    break;
  case BOX_EDGE:
	  ss<<"Feature: Edge"<<std::endl;
    break;
  case BOX_FACE:
	  ss<<"Feature: Face"<<std::endl;
    break;
  }
	ss<<"Min-Max projection: "<<m_dMin<<m_dMax<<std::endl;
	ss<<"------------------------------------"<<std::endl;

	out = ss.str();
	return out;

  };

  bool m_bOverlap;
  bool m_bPenetration;
  
  int m_iMinOrientation;
  int m_iMinSepOrientation;

  int m_iFeature;
  int m_iMinFeature;
  int m_iMinSepFeature;
  int m_iFeatureIndex[8];
  int m_iMinFeatureIndex[8];
  int m_iMinSepIndex[8];


  T m_dMin, m_dMax;
  T m_dMinOverlap;
  T m_dMinSeparation;

  CVector3<T> m_vNormal;
  CVector3<T> m_vMinNormal;
  CVector3<T> m_vMinSepNormal;

};

/**
* @brief Provides common algorithms for intersection operations
*
*
*/
template <class T>
class CIntersectorTools
{
public:
  CIntersectorTools(void);
  ~CIntersectorTools(void);

  void Find(const CVector3<T> &testAxis);

/**
* @brief Generates the projection info of a box for a given axis
*
* Generates the projection info of a box relative a given axis for
* a SAT intersection-find operation
*
* @param testAxis The axis that the info is generated for
* @param box The box for that the min-max projection is calculated
* @param cfg The projection configuration that we want to generate
*/
  static void getProjCfg(const CVector3<T> &testAxis, const OBB3<T> &box, CProjCfg<T> &cfg);

/**
* @brief Wrapper for the "find-orientation of two boxes relative to an axis" function
*
*
* @param testAxis The axis that the info is generated for
* @param box0 The first of the two boxes
* @param box1 The second boxes
* @param relVelocity The relative velocity of the boxes (v1-v0)
* @param tmax The maximum time for an intersection
* @param tfirst The first time of intersection for any axis
* @param tlast  The last time of intersection for any axis
* @param side The orientation of box1 to box0 relative to testAxis
* @param cfgFinal0 The final projection info for box0
* @param cfgFinal1 The final projection info for box1
*/
  static bool Find(const CVector3<T> &testAxis, const OBB3<T> &box0, const OBB3<T> &box1, const CVector3<T> relVelocity,
                   T &tmax, T &tfirst, T &tlast,int &side, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1);


/**
* @brief Computes the "find-intersection time of two features relative to an axis" query
*
*
* @param testAxis The axis that the info is generated for
* @param relVelocity The relative velocity of the boxes (v1-v0)
* @param tmax The length of the time interval, that is checked
* @param tfirst The first time of intersection for any axis
* @param tlast  The last time of intersection for any axis
* @param side The orientation of box1 to box0 relative to testAxis
* @param cfgCurr0 The current projection info for box0 relative to testAxis
* @param cfgCurr1 The current projection info for box1 relative to testAxis
* @param cfgFinal0 The final projection info for box0
* @param cfgFinal1 The final projection info for box1
*/
  static bool Find(const CVector3<T> &testAxis, const CVector3<T> relVelocity, T &tmax, T &tfirst, T &tlast,int &side,
                         CProjCfg<T> &cfgCurr0, CProjCfg<T> &cfgCurr1, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1);


/**
* @brief Computes the contact set between two intersecting boxes
*
*
* @param box0 The first of the two boxes
* @param iside The orientation of box1 to box0
* @param cfg0 The projection info for box0 with respect to the overlapping axis
* @param cfg1 The projection info for box1 with respect to the overlapping axis
* @param vel0 The velocity of box0
* @param vel1 The velocity of box1
* @param tfirst The first time of impact
* @param nContacts The number of contact points in the set
* @param vContacts The vector that will contain the points of the contact set
*/
  static void ComputeContactSet(const OBB3<T> &box0, const OBB3<T> &box1, int iside, const CProjCfg<T> &cfg0,
                           const CProjCfg<T> &cfg1, const CVector3<T> &vel0, const CVector3<T> &vel1,
                           T tfirst, int &nContacts, std::vector<CVector3<T> > &vContacts);


  static void ComputeContactSet(const OBB3<T> &box0, const OBB3<T> &box1, int iside, const CProjCfg<T> &cfg0,
                           const CProjCfg<T> &cfg1, const CVector3<T> &vel0, const CVector3<T> &vel1,
                           int &nContacts, std::vector<CVector3<T> > &vContacts);


  static void ComputeContactSet(const Cylinder<T> &cylinder, const OBB3<T> &box,
                                CSimplexDescriptorGjk<T> &simplex,
                                const Transformation<T> &transform0, const Transformation<T> &transform1,
                                const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void ComputeContactSet(const Cylinder<T> &cylinder0, const Cylinder<T> &cylinder1,
                                CSimplexDescriptorGjk<T> &simplex,
                                const Transformation<T> &transform0, const Transformation<T> &transform1,
                                const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void ComputeContactSetGjk(const ConvexShape<T> &shape0, const ConvexShape<T> &shape1, int shapeId0, int shapeId1, 
                                CSimplexDescriptorGjk<T> &simplex,
                                const Transformation<T> &transform0, const Transformation<T> &transform1,
                                const CVector3<T> &closestPoint0, const CVector3<T> &closestPoint1,
                                int &nContacts, std::vector<CVector3<T> > &vContacts);

  static bool Find(const CVector3<T> &testAxis, const OBB3<T> &box0, const OBB3<T> &box1,
                                  CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1);

  static bool Find(const CVector3<T> &testAxis, CProjCfg<T> &cfgCurr0,
            CProjCfg<T> &cfgCurr1, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1);

  static CVector3<T> getPoint(int index, const OBB3<T> &box)
  {
    return box.getVertex(index);
  };

  static void FindSegmentSegment(CVector3<T> seg0[2],CVector3<T> seg1[2],int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void SegmentRectanglePlanar(CVector3<T> seg[2],CVector3<T> rec[4],int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void RectangleRectanglePlanar(CVector3<T> rec0[4],CVector3<T> rec1[4],int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void ColinearSegments(CVector3<T> seg0[2],CVector3<T> seg1[2],int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void SegmentSegmentPlanar(CVector3<T> seg[2],const CVector3<T> &origin, const CVector3<T> &normal, int &nContacts, std::vector<CVector3<T> > &vContacts);

  static void ClipConvexPolygonAgainstPlane(const CVector3<T>& normal, T constant, int& quantity, CVector3<T> *P);

  static void ProjectLineOnBoxPlane(const OBB3<T> &box, unsigned int plane, const CVector3<T> &v0, const CVector3<T> &v1, CVector3<T> seg[2]);

  static void ProjectPointOnBoxPlane(const OBB3<T> &box, unsigned int plane, const CVector3<T> &v0, CVector3<T> &point);

  static void ClipCircleRectangle(const CRectangle3<T> &rec, const CVector3<T> &circleCenter, T radius, std::vector<CVector3<T> > &vContacts);

  static int map(int index)
  {

    if(index==0)return 0;
    else if(index==1)return 1;
    else if(index==2)return 3;
    else if(index==3)return 2;
    else if(index==4)return 4;
    else if(index==5)return 5;
    else if(index==6)return 7;
    else if(index==7)return 6;
  }
    

};

}

#endif
