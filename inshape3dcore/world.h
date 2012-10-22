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
#include <parinfo.h>
#include <subdomainboundary.h>
#include <particleSystem.h>

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
    if(m_bLiquidSolid)
      return pBody->m_dVolume*(pBody->m_dDensity - m_dDensityMedium) * pBody->m_dInvMass * m_vGrav;      
    else
      return m_vGrav;
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
  
  /**
   * The vector of rigid bodies used in the simulation
   **/
  std::vector<CRigidBody*> m_vRigidBodies;
  
  std::vector<VECTOR3> m_vExternalForces;

  std::list<std::pair<int,int> > m_lSendList;
  
  /**
   * A describtion of the boundary of the simulation domain
   **/
	CBoundaryBoxr *m_pBoundary;
  
  /**
   * An object that controls the timestep settings
   **/
	CTimeControl *m_pTimeControl;
  
	int m_iParticles;
  
	int m_iRigidBodiesDynamic;
  
	int m_iRigidBodiesStatic;
  
	int m_iOutput;

  /**
   * On/Off switch for extended contact graph algorithms
   **/
  bool m_bExtGraph;

  /**
   * In case the rigid bodies are immersed into a surrounding medium
   * we have to define the density of the surrounding medium
   **/
  Real    m_dDensityMedium;
  
  /**
   * When the simulation is run in parallel, we store
   * the parallel info in this structure
   **/
  CParInfo m_myParInfo;
  
  /**
   * True in case of a Liquid-Solid simulation and false
   * for a pure solid simulation.
   **/
  bool m_bLiquidSolid;
  
  /**
   * Description of the subdomainboundary
   * */
  CSubdomainBoundary *m_pSubBoundary;

  ParticleSystem *psystem;

  private:  
  
  /**
   * The gravity vector (assumed unit is m/s**2)
   **/  
  VECTOR3 m_vGrav;  
  


};

}

#endif // PARTICLEDOMAIN_H
