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
#include <world.h>

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

  World *world_;

  WorldParameters *params_;

  ParticleFactory(){};

  ParticleFactory(World &world, WorldParameters &params);

  ~ParticleFactory(){};
  
  World produceSpheres(int nSpheres, Real rad);

  void addSpheres(std::vector<RigidBody*> &vRigidBodies, int iCount, Real rad);
  
  void addMeshObjects(std::vector<RigidBody*> &vRigidBodies, int iCount, const char *strFileName);  

  World produceSphericalPure(int iCount);

  World produceTubes(const char* strFileName);

  World produceMesh(const char* strFileName);
  
  World produceFromParameters(WorldParameters &param);
  
  World produceFromDeformParameters(DeformParameters &param);   

  World produceBoxes(int iCount, Real extends[3]);

  void addBoxes(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3]);
  
  World produceCylinders(int iCount, Real extends[3]);  

  World produceFromFile(const char* strFileName, TimeControl &timeControl);

  void addCylinders(std::vector<RigidBody*> &vRigidBodies, int iCount, Real extends[3]);
  
  void buildSpheres(std::vector<RigidBody*> &vBodies, Real dRad);

  void addFromDataFile(WorldParameters &param, World *pWorld);

  void meshCowStack();

private:
  
  void buildSphereOfSpheres();
  
  void initFromParticleFile();

  void initRigidBodyParameters();
  
  void initSplitBottom();

  void addSpheres2(std::vector<RigidBody*> &vRigidBodies, int iCount, Real rad);

  VECTOR3 getPointOnCircle(VECTOR3 t0, VECTOR3 t1, Real l, int j, Real nu)
  {
    VECTOR3 dhk(cos(2.0 * (Real(j)-1.0)*CMath<Real>::SYS_PI/Real(nu))*t0 + sin(2.0*(Real(j)-1.0)*CMath<Real>::SYS_PI/Real(nu))*t1);
    dhk.Normalize();
    dhk *= l;
    return dhk;
  }

  inline float frand()
  {
    return rand() / (float)RAND_MAX;
  }

};

}

#endif // PARTICLEFACTORY_H
