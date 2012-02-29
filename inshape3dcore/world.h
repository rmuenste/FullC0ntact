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

#ifndef PARTICLEDOMAIN_H
#define PARTICLEDOMAIN_H
#include<vector>
#include<3dmodel.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <rigidbody.h>
#include <boundarybox.h>
#include <timecontrol.h>


namespace i3d {

/**
* @brief The physics world
*
* A class that holds all the parameters defining the physics world that we intend
* simulate.
*
*/
class CWorld
{
public:

  CWorld();
  ~CWorld();
  CWorld(const CWorld &copy);

  void Init();

	inline void SetGravity(const VECTOR3 &vGravity)
	{
		m_vGrav = vGravity;
	}

	inline VECTOR3 GetGravity() 
	{
		return m_vGrav;
	}
	
  inline VECTOR3 GetGravityEffect(CRigidBody *pBody) 
  {
    return m_vGrav;
    //return pBody->m_dVolume*(pBody->m_dDensity - m_dDensityMedium) * pBody->m_dInvMass * m_vGrav;
  }	

	inline void SetBoundary(CBoundaryBoxr *pBoundary)
	{
		m_pBoundary = pBoundary;
	}

	inline void SetTimeControl(CTimeControl *pTimeControl)
	{
		m_pTimeControl = pTimeControl;
	}

	Real GetTotalEnergy();

	friend std::ostream& operator<<(std::ostream& out, CWorld &world); 
	
/**
* @brief Prints the world configuration to a string
*
* @return Returns a string output of the world configuration
*/
	std::string ToString();
  
  std::vector<C3DModel> m_vParticles;
  std::vector<C3DModel> m_vSolids;
	std::vector<CRigidBody*> m_vRigidBodies;
  std::vector<VECTOR3> m_vExternalForces;
	CBoundaryBoxr *m_pBoundary;
	CTimeControl *m_pTimeControl;
	int m_iParticles;
	int m_iRigidBodiesDynamic;
	int m_iRigidBodiesStatic;
	int m_iOutput;

  Real    m_dDensityMedium;
  
  private:  
    VECTOR3 m_vGrav;  
  
};

}

#endif // PARTICLEDOMAIN_H
