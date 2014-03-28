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
class World;
class TimeControl;

/**
* @brief A factory class that produces rigid bodies for the physics world
*
* A factory class that produces rigid bodies for the physics world
*
*/
class ParticleFactory
{

public:
  ParticleFactory(){};
  ~ParticleFactory(){};
  
  World ProduceSpheres(int iCount, Real rad);

  void addSpheres(std::vector<RigidBody*> &vRigidBodies, int iCount, Real rad);
  
  void addMeshObjects(std::vector<RigidBody*> &vRigidBodies, int iCount, const char *strFileName);  

  World produceSphericalPure(int iCount);

  World produceTubes(const char* strFileName);

  World produce2RigidBodies(void);

  World produce2RigidBodies3(void);

  World produceMesh(const char* strFileName);
  
  World produceFromParameters(WorldParameters &param);
  
  World produceFromDeformParameters(DeformParameters &param);   

  World produceBoxes(int iCount, Real extends[3]);

  void addBoxes(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3]);
  
  World produceCylinders(int iCount, Real extends[3]);  

  World produceFromFile(const char* strFileName, TimeControl &timeControl);

  World produceMixer();

  void addCylinders(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3]);
  
  void buildSpheres(std::vector<RigidBody*> &vBodies, Real dRad);

  void addFromDataFile(WorldParameters &param, World *pWorld);

};

}

#endif // PARTICLEFACTORY_H
