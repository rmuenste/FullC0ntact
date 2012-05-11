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

#ifndef PARTICLEFACTORY_H
#define PARTICLEFACTORY_H
#include <vector>
#include <rigidbody.h>
#include <worldparameters.h>
#include <deformparameters.h>

namespace i3d {

class C3DModel;
class CWorld;
class CTimeControl;

/**
* @brief A factory class that produces rigid bodies for the physics world
*
* A factory class that produces rigid bodies for the physics world
*
*/
class CParticleFactory
{

public:
  CParticleFactory(){};
  ~CParticleFactory(){};
  
  CWorld ProduceSpheres(int iCount, Real rad);

  void AddSpheres(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real rad);

  CWorld ProduceSphericalPure(int iCount);


	CWorld ProduceTubes(const char* strFileName);

	CWorld Produce2RigidBodies(void);

	CWorld Produce2RigidBodies3(void);

	CWorld ProduceMesh(const char* strFileName);
  
  CWorld ProduceFromParameters(CWorldParameters &param);
  
  CWorld ProduceFromDeformParameters(CDeformParameters &param);   

  CWorld ProduceBoxes(int iCount, Real extends[3]);

  void AddBoxes(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real extends[3]);
  
  CWorld ProduceCylinders(int iCount, Real extends[3]);  

	CWorld ProduceFromFile(const char* strFileName, CTimeControl &timeControl);

	CWorld ProduceMixer();

  void AddCylinders(std::vector<CRigidBody*> &vRigidBodies, int iCount, Real extends[3]);
	
	void BuildSpheres(std::vector<CRigidBody*> &vBodies, Real dRad);

};

}

#endif // PARTICLEFACTORY_H
