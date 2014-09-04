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
#ifndef CCOLLRESPONSESI_H_
#define CCOLLRESPONSESI_H_
#include <collresponse.h>
#include <vectorn.h>

namespace i3d {

/**
 * @brief An SI(sequential impulses)-based collision response solver
 */  
class CollResponseSI : public CollResponse
{
public:
  CollResponseSI(void);

  CollResponseSI(std::list<CollisionInfo> *CollInfo, World *pWorld);

  ~CollResponseSI(void);

  void Solve();
  
private:

  void ApplyImpulse(CollisionInfo &ContactInfo);

  Real computeDefect(Real &maxNorm);

  Real computeDefectImpulse();

  void PreComputeConstants(CollisionInfo &ContactInfo);

  void ComputeTangentSpace(const VECTOR3 &normal, VECTOR3 &t1, VECTOR3 &t2);

  void sortByStackHeight(std::vector<CollisionInfo*> &contactInfo);

  int GetNumIterations() { return iterations_; };

  void setMaxIterations(int i) { iterations_ = i; };

  void setDefEps(Real e) { eps_ = e; };

  Real getDefEps() { return eps_; };

  VectorNr defect_;

  Real oldDefect_;

  Real m_dBiasFactor;

  int  nContactInfos;

  int iterations_;

  Real eps_;

};

}

#endif
