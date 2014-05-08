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

#ifndef MESHOBJECT_H
#define MESHOBJECT_H

#include <shape.h>
#include <3dmodel.h>
#include <distancefuncgridmodel.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <string>

namespace i3d {

/** @brief A shape object that is described by a triangular mesh
 *
 * A shape object that is described by a triangular mesh
 */  
template<class T>
class CMeshObject : public Shape<T>
{

public:
  CMeshObject(void);
  CMeshObject(const char* strFilename);
  ~CMeshObject(void);
  
  T getVolume() const;

  AABB3<T> getAABB() 
  {
    return m_Model.GetBox();
  }

  bool isPointInside(const Vector3<T> &vQuery) const
  {
    CDistanceFuncGridModel<T> distFunc;
    //if(distFunc.BruteForceInnerPointsStatic(m_Model,vQuery)==1)
    const CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pRoot = m_BVH.GetChild(0);
    if(distFunc.PointInside(pRoot,vQuery)==1)
      return true;
    else
      return false;
  }      
    
  C3DModel m_Model;
  CBoundingVolumeTree3<AABB3r,Real,CTraits,CSubdivisionCreator> m_BVH;

  void   SetFileName(const char* strFileName){m_sFileName=std::string(strFileName);};
  std::string GetFileName(){return m_sFileName;}

  /**
    * Returns the geometric center of the shape
    *
    */
  Vector3<T> getCenter() const {return m_Model.m_bdBox.center_;};

  private:
  void Traverse(CBoundingVolumeNode3<AABB3<T>,T,CTraits> *pNode) const;

  std::string m_sFileName;

};

typedef CMeshObject<Real> CMeshObjectr;

}

#endif // MESHOBJECT_H
