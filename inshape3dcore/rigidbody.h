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
#include <set>
#include <distancemap.h>
#include <sphere.h>

namespace i3d {

  class CollisionInfo;

  
  typedef struct {
    float x,y,z;
    float vx,vy,vz;
    float ax,ay,az;
    float avx,avy,avz;
    float mx,my,mz;
    float tx,ty,tz;
    float exx,exy,exz;
    float qx,qy,qz,qw;
    float density;
    float invmass;
    float volume;
    float restitution; 
    
    float a1,a2,a3,a4,a5,a6,a7,a8,a9; //38 floats
    
    int ishape;
    int igrav;
    int origid; //3 ints
    
  }Particle;  
  
template<typename T>
  class DistanceMap<T,gpu>;
  
/**
*  @brief A rigid body in the physics world
*
*  A rigid body in the physics world
*/
class RigidBody
{
  
private:

  /**
   * Angular velocity in radians per time unit
   * I.e. rad/s if the time unit is seconds
   */
  VECTOR3   angVel_;

  VECTOR3   biasAngVel_;

  MATRIX3X3 matTransform_;

  
  std::list<CollisionInfo *> edges_;
  
public:
	Quaternionr quat_;
	Transformationr transform_;
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
    MESH,
    COMPOUND,
    SUBDOMAIN,
    PLINE,
    CYLINDERBDRY,
    HOLLOWCYLINDER
  };
  
  VECTOR3   velocity_;
  VECTOR3   oldVel_;
  VECTOR3   oldAngVel_;
  VECTOR3   biasVelocity_;
  Real      density_;
  Real      volume_;
  Real      invMass_;
  Shaper*  shape_;
  
  MATRIX3X3 invInertiaTensor_;


  int       shapeId_;
  int       collisionState_;
  int       iID_;
  int       remoteID_;
  int       group_;
  int       height_;
  int       element_;
  int       process_;
  bool      visited_;
  bool      remote_;

  /**
   * The coefficient of restitution
   */
  Real      restitution_;
  
  /**
   * The coefficient of friction
   */
  Real      friction_;
  
  DistanceMap<float,gpu> *map_gpu_;
  DistanceMap<Real> *map_;
  std::vector<Spherer> spheres;
  
  Real      dampening_;
  VECTOR3   forceResting_;
  VECTOR3   torque_;
  VECTOR3   force_;

  VECTOR3   torque_local_;
  VECTOR3   force_local_;

  Real      color_;

  bool affectedByGravity_;

  VECTOR3   angle_;
  VECTOR3   com_;

  const static int MAX_HEIGHT=10001;
  
  std::list<int> elements_;
  std::list< std::pair<int,int>  > boundaryElements_;
  int elementsPrev_;
  std::set<int> remoteDomains_;
    
  /**
  *
  * Creates an empty rigid body
  *
  */
  RigidBody();

  virtual ~RigidBody();

  RigidBody(int iShape)
  {
    shapeId_ = iShape;
  }
  
  /** 
  *
  * Initializes a rigid body with parameters
  * @param velocity initial velocity
  * @param dDensity density
  * @param dVolume volume
  * @param dMass mass
  * @param vAngle initial angle
  */
  RigidBody(VECTOR3 velocity,Real dDensity,Real dVolume,Real dMass,VECTOR3 vAngle,int iShape);

  /** 
  *
  * Initializes a rigid body with a shape 
  * @param pSphere pointer to the shape
  * @param iShape id of the shape
  */
  RigidBody(Shaper *pShape, int iShape);

  /** 
  *
  * Initializes a rigid body
  * @param pBody Information about the rigid body we want to create
  */
  RigidBody(BodyStorage *pBody, bool sub=false);
  
  /** 
  *
  * Initializes a rigid body
  * @param p Parallel rigid body data format
  */
  RigidBody(Particle &p);

  /** 
  * Copy a rigid body
  */
  RigidBody(const RigidBody& copy);

  virtual void translateTo(const VECTOR3 &vPos);
  
  /** 
  *
  * Computes the inverse of the inertia tensor and stores it
  * in the member variable m_InvInertiaTensor
  *
  */
  virtual void generateInvInertiaTensor();
  
  /** 
  * Computes the kinetic energy of the body due to its motion
  * @return The kinetic energy of the body
  */
  Real getEnergy();

  /** 
  * Returns the inverse of the world-transformed inertia tensor
  * @return The inverse of the world-transformed inertia tensor
  */
  virtual MATRIX3X3 getWorldTransformedInvTensor();

  /** 
  * Returns the orientation of the body as a matrix
  * @return Returns the orientation of the body as a matrix
  */
  Transformationr getTransformation() const;
  
  /** 
  * Returns the transformation matrix
  * @return Returns the orientation of the body as a matrix
  */
  MATRIX3X3 getTransformationMatrix() const;

  /** 
  * Set the transformation matrix
  * @param mat The transformation matrix
  */
  void setTransformationMatrix(const MATRIX3X3 &mat);
  
  /**
  * Returns the world-transformed shape
  * @return The world-transformed shape
  */
  Shaper* getWorldTransformedShape();

  /**
  * Returns the world-transformed shape after stepping dT
  * @return The world-transformed shape after one time step of size dT
  */
  Shaper* getWorldTransformedShapeNext(Real dT);

  /**
  * Computes the rotation of the body in AxisAngle format
  * @return The axis of rotation scaled by the angle
  */
  VECTOR3 getAxisAngle();

  /**
  * Returns the untransformed shape
  * @return The untransformed shape
  */  
  const Shaper& getOriginalShape() const;

  /**
  * Returns whether the body is affected by gravity
  * @return Returns whether the body is affected by gravity
  */    
  bool isAffectedByGravity() const {return affectedByGravity_;};
  
  /**
  * Returns the angular velocity in world coordinates
  * @return Returns the angular velocity
  */        
  VECTOR3 getAngVel() const {return angVel_;};

  /**
  * Sets the angular velocity
  * @param angVel The angular velocity
  */        
  void setAngVel(const VECTOR3 &angVel) {angVel_=angVel;};
  
  /**
  * Sets the orientation of the body
  * @param vXYZ The orientation in euler angles
  */        
  void setOrientation(const VECTOR3 &vXYZ)
  {
    quat_.CreateFromEulerAngles(vXYZ.y,vXYZ.z,vXYZ.x);
    matTransform_=quat_.GetMatrix();
  };
  
  /**
  * Updates the angular velocity by delta
  */        
  void updateAngVel(const VECTOR3 &delta);

  /**
  * Applies an angular impulse and a linear impulse
  */          
  void applyImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate);

  /**
  * Applies a bias angular and linear impulse
  */
  void applyBiasImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate);

 
  /**
  * Tests if a point is inside the rigid body
  */          
  bool isInBody(const VECTOR3 &vQuery) const;
  
  /**
  * Returns the orientation as a quaternion
  * @return Returns the quaternion
  */        
  Quaternionr getQuaternion() const {return quat_;};

  /**
  * Sets the orientation quaternion
  * @param q The quaternion
  */        
  void setQuaternion(const Quaternionr &q) {quat_=q;};
  
  /**
   * Get the edges vector of the rigid body
   **/  
  std::list<CollisionInfo *>& getEdges()
  {
    return edges_;
  }
  
  /**
   * The body gets a edge that represents a contact connection
   * to another body
   * @param pInfo The edge that represents a contact connection
   **/  
  void addEdge(CollisionInfo *pInfo)
  {
    edges_.push_back(pInfo);
  }
  

  /**
   * Removes an edge (contact connection) from the list of contacts
   * @param pInfo The edge that should be removed
   **/
  void removeEdge(CollisionInfo *pInfo);
  
  /**
   * Returns the radius of a bounding sphere for the body
   **/
  virtual Real getBoundingSphereRadius()
  {
    //Polymorphism :)
    if(shapeId_ == RigidBody::SPHERE)
    {
      return shape_->getAABB().extents_[0];
    }
    else if(shapeId_ == RigidBody::BOUNDARYBOX)
    {
      AABB3r aabb = shape_->getAABB();
      int iAxis = aabb.longestAxis();
      return aabb.extents_[iAxis];
    }
    else if(shapeId_ == RigidBody::PLANE)
    {
      AABB3r aabb = shape_->getAABB();
      int iAxis = aabb.longestAxis();
      return aabb.extents_[iAxis];
    }
    else
      return shape_->getAABB().getBoundingSphereRadius();
  }

  AABB3r getAABB()
  {
    Shaper *transformedShape = getWorldTransformedShape();
    
    AABB3r aabb   = transformedShape->getAABB();
    aabb.center_ = com_;
    aabb.vertices_[0] = aabb.center_ - VECTOR3(aabb.extents_[0],aabb.extents_[1],aabb.extents_[2]);
    aabb.vertices_[1] = aabb.center_ + VECTOR3(aabb.extents_[0],aabb.extents_[1],aabb.extents_[2]);
    
    delete transformedShape;
    return aabb;
  }

  /**
   * Returns a shape identifier for the body
   **/
  inline int getShape() const {return shapeId_;};

  /**
   * Returns the coefficient of friction
   **/
  inline Real getFriction() const {return friction_;};
  
  /**
   * Returns the unique ID of the body
   **/
  virtual int getID() {return iID_;};

  /**
   * Set the ID of the body
   **/
  virtual void setID(int id) {iID_=id;};

  /**
  * Returns the bias angular velocity
  * @return Returns the bias angular velocity
  */
  VECTOR3 getBiasAngVel() const {return biasAngVel_;};

  /**
  * Sets the bias angular velocity
  * @param angVel The bias angular velocity
  */
  void setBiasAngVel(const VECTOR3 &angVel) {biasAngVel_=angVel;};

  /**
  * Returns the bias velocity
  * @return Returns the bias velocity
  */
  VECTOR3 getBiasVelocity() const {return biasVelocity_;};
  
  /**
  * Constructs a distance map for the rigid body
  */
  void buildDistanceMap();

  /**
  * Sets the bias velocity
  * @param biasVelocity The bias velocity
  */
  void setBiasVelocity(const VECTOR3 &biasVelocity) {biasVelocity_=biasVelocity;};

  bool isLocal() {return !remote_;};

  bool isRemote() {return remote_;};

  void setRemote(bool remote) {remote_=remote;};
  
  bool isKnownInDomain(int domain)
  {
    bool found = (remoteDomains_.find(domain) != remoteDomains_.end());
    return found;
  }

  void addRemoteDomain(int domain)
  {
    remoteDomains_.insert(domain);
  }

  int nDofsHexa(VECTOR3 vertices[8])
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
      dofs[j]= Real(0.5) * (vertices[iedges[i][0]] + vertices[iedges[i][1]]);
    }
    for(i=0,j=20;i<6;i++,j++)
    {
      dofs[j]= Real(0.25) * (vertices[ifaces[i][0]] + vertices[ifaces[i][1]] + vertices[ifaces[i][2]] + vertices[ifaces[i][3]]);
    }

    dofs[26] =  Real(0.125) * (vertices[0] + vertices[1] + vertices[2] + vertices[3] + vertices[4] + vertices[5] + vertices[6] + vertices[7]);

    for(i=0;i<27;i++)
    {
      if(isInBody(dofs[i]))
      {
        count++;
      }
    }

    return count;
  }
  
  void model_out();

  VECTOR3 getTransformedPosition()
  {

	  VECTOR3 trans = transform_.getMatrix() * com_;
	  trans += transform_.getOrigin();
	  return trans;
  }

  /**
  * applies a force and a torque to the body
  */
  void applyForces(const VECTOR3 &force, const VECTOR3 &torque);

};

}

#endif // RIGIDBODY_H
