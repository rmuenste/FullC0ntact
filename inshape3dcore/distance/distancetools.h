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
#ifndef DISTANCETOOLS_H
#define DISTANCETOOLS_H



//===================================================
//                     INCLUDES
//===================================================
#include <obb3.h>
#include <rectangle3.h>
#include <segment3.h>

namespace i3d {

 /**
* @brief CObjConfiguration
*
*/ 
template <typename T>
class CObjConfiguration
{
public:
  CObjConfiguration(){};
  ~CObjConfiguration(){};

  CVector3<T> m_vNormal;
  CVector3<T> m_vContactPoints[8];
  int         m_iContacts;
  int         m_iConf;
  int         m_iFeature[2];
  int         m_iFeatureIndex[2];
  T           m_dSeparation;

};
  
/**
* @brief A class that has several methods shared by various distance algorithms
*
*
*/
template <class T>
class CDistanceTools {

public: 

CDistanceTools(); 

~CDistanceTools(); 

  static unsigned int ClassifyVertex(const CVector3<T> &vertex, const OBB3<T> &box);

  //these routines return the VERTEX,EDGE,FACE corresponding to iRegion of OBB iwhich
  //the routines assume that OBB iwhich has been transformed such that it
  //is 0-centered and axisparallel
  static CVector3<T> GetRegionVertex(unsigned int iRegion,   const OBB3<T> &box);
  static CSegment3<T> GetRegionEdge(unsigned int iRegion,    const OBB3<T> &box);
  static CRectangle3<T> GetRegionFace(unsigned int iRegion,  const OBB3<T> &box);
  
  inline static CVector3<T>   GetFaceNormal(unsigned int iRegion, const OBB3<T> &box);

  inline static int GetRegionType(unsigned int regionCode)
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

  enum
  {
    VERTEX=1,
    EDGE,
    FACE,
    INNER
  };

};

}
#endif
