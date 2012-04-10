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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector3.h>
#include <obb3.h>
#include <vector>
#include <matrix3x3.h>
#include <shape.h>
#include <rigidbodyio.h>
#include <quaternion.h>
#include <transform.h>
#include <list>

namespace i3d {

  class CCollisionInfo;

/**
*  @brief A rigid body in the physics world
*
*  A rigid body in the physics world
*/
class CRigidBody
{
public:

  /**
  *
  * Creates an empty rigid body
  *
  */
  CRigidBody();

  ~CRigidBody();

  CRigidBody(int iShape)
  {
    m_iShape = iShape;
  }
  
  /** 
  *
  * Initializes a rigid body with parameters
  * @param vVelocity initial velocity
  * @param dDensity density
  * @param dVolume volume
  * @param dMass mass
  * @param vAngle initial angle
  */
  CRigidBody(VECTOR3 vVelocity,Real dDensity,Real dVolume,Real dMass,VECTOR3 vAngle,int iShape);

  /** 
  *
  * Initializes a rigid body with a shape 
  * @param pSphere pointer to the shape
  * @param iShape id of the shape
  */
  CRigidBody(CShaper *pShape, int iShape);

  /** 
  *
  * Initializes a rigid body
  * @param pBody Information about the rigid body we want to create
  */
  CRigidBody(sRigidBody *pBody);

  /** 
  * Copy a rigid body
  */
  CRigidBody(const CRigidBody& copy);

  void TranslateTo(const VECTOR3 &vPos);
  
  /** 
  *
  * Computes the inverse of the inertia tensor and stores it
  * in the member variable m_InvInertiaTensor
  *
  */
  void GenerateInvInertiaTensor();
  
  /** 
  * Computes the kinetic energy of the body due to its motion
  * @return The kinetic energy of the body
  */
  Real GetEnergy();

  /** 
  * Returns the inverse of the world-transformed inertia tensor
  * @return The inverse of the world-transformed inertia tensor
  */
  MATRIX3X3 GetWorldTransformedInvTensor();

  /** 
  * Returns the orientation of the body as a matrix
  * @return Returns the orientation of the body as a matrix
  */
  CTransformr GetTransformation() const;
  
  /** 
  * Returns the transformation matrix
  * @return Returns the orientation of the body as a matrix
  */
  MATRIX3X3 GetTransformationMatrix() const;

  /** 
  * Set the transformation matrix
  * @param mat The transformation matrix
  */
  void SetTransformationMatrix(const MATRIX3X3 &mat);
  
  /**
  * Returns the world-transformed shape
  * @return The world-transformed shape
  */
  CShaper* GetWorldTransformedShape();

  /**
  * Returns the world-transformed shape after stepping dT
  * @return The world-transformed shape after one time step of size dT
  */
  CShaper* GetWorldTransformedShapeNext(Real dT);

  /**
  * Computes the rotation of the body in AxisAngle format
  * @return The axis of rotation scaled by the angle
  */
  VECTOR3 GetAxisAngle();

  /**
  * Returns the untransformed shape
  * @return The untransformed shape
  */  
  const CShaper& GetOriginalShape() const;

  /**
  * Returns whether the body is affected by gravity
  * @return Returns whether the body is affected by gravity
  */    
  bool IsAffectedByGravity() const {return m_bAffectedByGravity;};
  
  /**
  * Returns whether the body is resting
  * @return Returns whether the body is resting
  */      
  bool Resting() const {return m_bResting;};
  
  /**
  * Returns whether the body is touching the ground
  * @return Returns whether the body is touching the ground
  */        
  bool TouchesGround() const {return m_bTouchesGround;};    

  /**
  * Returns the angular velocity in world coordinates
  * @return Returns the angular velocity
  */        
  VECTOR3 GetAngVel() const {return m_vAngVel;};

  /**
  * Sets the angular velocity
  * @param angVel The angular velocity
  */        
  void SetAngVel(const VECTOR3 &angVel) {m_vAngVel=angVel;};
  
  /**
  * Sets the orientation of the body
  * @param vXYZ The orientation in euler angles
  */        
  void SetOrientation(const VECTOR3 &vXYZ)
  {
    m_vQ.CreateFromEulerAngles(vXYZ.y,vXYZ.z,vXYZ.x);
    m_matTransform=m_vQ.GetMatrix();
  };
  
  /**
  * Updates the angular velocity by delta
  */        
  void UpdateAngVel(const VECTOR3 &delta);

  /**
  * Applies an angular impulse and a linear impulse
  */          
  void ApplyImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate);

  /**
  * Tests if a point is inside the rigid body
  */          
  bool IsInBody(const VECTOR3 &vQuery) const;
  
  /**
  * Returns the orientation as a quaternion
  * @return Returns the quaternion
  */        
  CQuaternionr GetQuaternion() const {return m_vQ;};

  /**
  * Sets the orientation quaternion
  * @param q The quaternion
  */        
  void SetQuaternion(const CQuaternionr &q) {m_vQ=q;};
  
  /**
   * The body gets a edge that represents a contact connection
   * to another body
   * @param pInfo The edge that represents a contact connection
   **/  
  void AddEdge(CCollisionInfo *pInfo)
  {
    m_pEdges.push_back(pInfo);
  }

  /**
   * Removes an edge (contact connection) from the list of contacts
   * @param pInfo The edge that should be removed
   **/
  void RemoveEdge(CCollisionInfo *pInfo);
  
  /**
   * Returns the radius of a bounding sphere for the body
   **/
  Real GetBoundingSphereRadius()
  {
	  if(m_iShape == CRigidBody::SPHERE)
	  {
      return m_pShape->GetAABB().m_Extends[0];
	  }
    else
      return m_pShape->GetAABB().GetBoundingSphereRadius();
  }

  /**
   * Returns a shape identifier for the body
   **/
  inline int GetShape() const {return m_iShape;};

  /**
   * Returns the coefficient of friction
   **/
  inline Real GetFriction() const {return m_dFriction;};
  
  
  int NDofsHexa(VECTOR3 vertices[8])
  {
    int count = 0;
    int i,j;
    int ifaces[6][4] = {{0,1,2,3},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7},{4,5,6,7}};
    int iedges[12][2] = {{0,1},{1,2},{2,3},{3,0},{0,4},{1,5},{2,6},{3,7},{4,5},{5,6},{6,7},{7,4}};

    VECTOR3 dofs[27];
    for(i=0;i<8;i++)
    {
      dofs[i]=vertices[i];
    }
    for(i=0,j=8;i<12;i++,j++)
    {
      dofs[j]= 0.5 * (vertices[iedges[i][0]] + vertices[iedges[i][1]]);
    }
    for(i=0,j=20;i<6;i++,j++)
    {
      dofs[j]= 0.25 * (vertices[ifaces[i][0]] + vertices[ifaces[i][1]] + vertices[ifaces[i][2]] + vertices[ifaces[i][3]]);
    }

    dofs[26] =  0.125 * (vertices[0] + vertices[1] + vertices[2] + vertices[3] + vertices[4] + vertices[5] + vertices[6] + vertices[7]);

    for(i=0;i<27;i++)
    {
      if(IsInBody(dofs[i]))
      {
        count++;
      }
    }

    return count;
  }

	enum
	{
		SPHERE,
		BOX,
		ELLIPSOID,
		CAPSULE,
		BVH,
		WALL,
		BOUNDARYBOX,
    CYLINDER,
    PLANE,
    MESH
	};
  
  VECTOR3   m_vVelocity;
  Real      m_dDensity;
  Real      m_dVolume;
  Real      m_dInvMass;
  CShaper*  m_pShape;
  
  MATRIX3X3 m_InvInertiaTensor;


  int       m_iShape;
  int       m_iCollisionState;
  int       m_iID;
  int       m_iGroup;
  int       m_iHeight;
  int       m_iElement;
  int       m_iProcess;
  bool      m_bVisited;
  
  /**
   * The coefficient of restitution
   */
  Real      m_Restitution;
  
  /**
   * The coefficient of friction
   */
  Real      m_dFriction;
  
  Real      m_dDampening;
  VECTOR3   m_vForceResting;
  VECTOR3   m_vTorque;
  VECTOR3   m_vForce;

  bool m_bResting;
  bool m_bTouchesGround;
  bool m_bAffectedByGravity;

  VECTOR3   m_vAngle;
  VECTOR3   m_vCOM;

  const static int MAX_HEIGHT=10001;
  
  std::list<CCollisionInfo *> m_pEdges;
  std::list<int> m_iElements;
  std::list< std::pair<int,int>  > m_iBoundaryElements;
  int m_iElementsPrev;

private:
	VECTOR3   m_vAngVel;
  MATRIX3X3 m_matTransform;  
  CQuaternionr m_vQ;  

};

}

#endif // RIGIDBODY_H
