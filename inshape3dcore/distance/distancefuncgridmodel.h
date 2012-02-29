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

#ifndef DISTANCEFUNCGRIDMODEL_H
#define DISTANCEFUNCGRIDMODEL_H

#include <distancefunctiongrid.h>
#include <3dmodel.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <list>
#include <ray3.h>

namespace i3d {

/**
* @brief Computes a distance function on a grid
*
* Computes a distance function on a grid, that means the distance between
* all vertices of the grid and the object under consideration
*
*/
template<class T>
class CDistanceFuncGridModel : public CDistanceFuncGrid<T>
{

public:
	
	 CDistanceFuncGridModel();
	 CDistanceFuncGridModel(CUnstructuredGrid<T,DTraits> *pGrid,const C3DModel &model); 
   ~CDistanceFuncGridModel(void);
   void ComputeDistance();

   int BruteForceInnerPointsStatic(const C3DModel &model, const CVector3<T> &vQuery);

   int PointInside(const CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode, const CVector3<T> &vQuery);

   const C3DModel *m_pModel;

private:

  void Traverse(const CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *pNode, const CRay3<T> &rRay);
  std::list<const CBoundingVolumeNode3<CAABB3<T>,T,CTraits> *> m_pNodes;

};

}

#endif // DISTANCEFUNCGRIDMODEL_H
