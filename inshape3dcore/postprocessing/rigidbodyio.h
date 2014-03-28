/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#ifndef RIGIDBODYWRITER_H
#define RIGIDBODYWRITER_H
#include <vector>
#include <vector3.h>
#include <matrix3x3.h>
#include <string.h>
#include <quaternion.h>

namespace i3d {

class RigidBody;
class World;

/**
*  @brief A class that stores the file header for a rigid body output file
*
*  A class that stores the file header for a rigid body output file.
*  The member variables represent the content of the header.
*/
class sRigidBodyHeader
{
public:
/**
  * Number of rigid bodies stored in the file
  */
  int    iNumParticles;
  
/**
 * The variable stores the integer rank of the time step
 * in a ordered sequence of time steps
 */  
  int    iTimeStep;
  
  int    iOutput;
  
/**
 * The simulation time of the output file
 */  
  float  dSimTime;
  
/**
 * The current time step used in the simulation at the time
 * of the outpu
 */  
  float  dDeltaT;
};

/**
*  @brief A class that stores contruction info for a rigid body
*
*  A class that stores contruction info for a rigid body
*/
class sRigidBody
{
public:
  sRigidBody() {m_bMatrixAvailable=false;};
  ~sRigidBody() {};  
  
  sRigidBody(const sRigidBody &copy)
  {
    m_vCOM         =    copy.m_vCOM;       
    m_vVelocity    =    copy.m_vVelocity;
    m_dDensity     =    copy.m_dDensity;
    m_dVolume      =    copy.m_dVolume;
    m_dInvMass     =    copy.m_dInvMass;
    m_Restitution  =    copy.m_Restitution;
    m_vAngVel      =    copy.m_vAngVel;
    m_vAngle       =    copy.m_vAngle;
    m_iShape       =    copy.m_iShape;
    m_iID          =    copy.m_iID;
    m_vForce       =    copy.m_vForce;
    m_vTorque      =    copy.m_vTorque;
    m_vQ           =    copy.m_vQ;
    m_Extends[0]   =    copy.m_Extends[0];
    m_Extends[1]   =    copy.m_Extends[1];
    m_Extends[2]   =    copy.m_Extends[2];   
    m_vUVW[0]      =    copy.m_vUVW[0];
    m_vUVW[1]      =    copy.m_vUVW[1];
    m_vUVW[2]      =    copy.m_vUVW[2];
    m_iAffectedByGravity = copy.m_iAffectedByGravity;
    m_bMatrixAvailable = copy.m_bMatrixAvailable;
    memcpy(m_dTensor,copy.m_dTensor,9*sizeof(Real));
    memcpy(m_strFileName,copy.m_strFileName,255);
  };
  
  VECTOR3      m_vCOM;
  VECTOR3      m_vVelocity;
  VECTOR3      m_vUVW[3];
  Real         m_dDensity;
  Real         m_dVolume;
  Real         m_dInvMass;
  Real         m_Restitution;
  VECTOR3      m_vAngVel;
  VECTOR3      m_vAngle;
  int          m_iShape;
  VECTOR3      m_vForce;
  VECTOR3      m_vTorque;
  CQuaternionr m_vQ;
  Real         m_Extends[3];
  Real         m_dTensor[9];
  char         m_strFileName[256];
  int          m_iAffectedByGravity;
  int          m_iID;
  bool         m_bMatrixAvailable;
};

/**
*  @brief A reader/writer for rigid bodies
*
*  A reader/writer for rigid bodies
*/
class CRigidBodyIO
{
	public:
		CRigidBodyIO();
		~CRigidBodyIO();
		
    /**
    * Writes the bodies in the world to a file
    *
    * @param world The world containing the bodies
    * @param strFileName The name of the output file
    * @param outputBoundary On/Off switch for outputting the boundary
    */
		void Write(World &world, const char *strFileName, bool outputBoundary=true);

    /**
    * Writes a selection of bodies from the world to a file
    *
    * @param world The world containing the bodies
    * @param world vIndices An index vector containing the indices of the bodies to write
    * @param strFileName The name of the output file
    */
		void Write(World &world, std::vector<int> &vIndices, const char *strFileName);

    /**
    * Reads bodies from a file into the world
    *
    * @param world The world that will contain the bodies
    * @param strFileName The name of the input file
    */
		void Read(World &world, const char *strFileName);
		
};

}

#endif // RIGIDBODYWRITER_H
