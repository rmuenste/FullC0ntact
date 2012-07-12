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
class CCollResponseSI : public CCollResponse
{
public:
	CCollResponseSI(void);
	CCollResponseSI(std::list<CCollisionInfo> *CollInfo, CWorld *pWorld);
	~CCollResponseSI(void);
	void Solve();
private:
  void ApplyImpulse(CCollisionInfo &ContactInfo);
  Real ComputeDefect();
  void PreComputeConstants(CCollisionInfo &ContactInfo);
  void ComputeTangentSpace(const VECTOR3 &normal, VECTOR3 &t1, VECTOR3 &t2);
  CVectorNr m_vDef;

};

}

#endif
