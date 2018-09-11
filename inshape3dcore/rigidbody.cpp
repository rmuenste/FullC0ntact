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

#include "rigidbody.h"
#include <sphere.h>
#include <boundarybox.h>
#include <rigidbodyio.h>
#include <stdlib.h>
#include <cylinder.h>
#include <meshobject.h>
#include <genericloader.h>
#include <3dmodel.h>
#include <subdivisioncreator.h>
#include <collisioninfo.h>
#include <quaternion.h>
#include <distancemeshpoint.h>
#include <cstring>
#ifdef FC_CUDA_SUPPORT
#include <cuda_runtime.h>
#include <difi.cuh>
#include <vtkwriter.h>
#endif


namespace i3d {

#ifdef WITH_CGAL
  bool cgal_avail = true;
#else
  bool cgal_avail = false;
#endif

  RigidBody::RigidBody() : collisionState_(0)
  {
    shape_             = nullptr;
    dampening_         = 1.0;
    affectedByGravity_ = true;
    elementsPrev_      = 0;
    friction_          = 0.0;
    remote_            = false;
    map_               = nullptr;
    volume_            = 0;
    iID_               = -1;
    invMass_           = 0.0;
    remoteID_          = -1;
    shapeId_           = -1;
    density_           = 0.0;
    height_            = -1;
    restitution_       = 0.0;
    visited_           = false;
    group_             = 0;
    element_           = -1;
    process_           = -1;
    color_             = 0.0;
    odeIndex_          = -1;
  }

  RigidBody::~RigidBody()
  {
    if(shape_ != nullptr)
    {
      delete shape_;
      shape_ = nullptr;
    }
  }

  RigidBody::RigidBody(Particle& p)
  {

    invMass_           = 0.0;
    height_            = -1;
    restitution_       = 0.0;
    visited_           = false;
    group_             = 0;
    element_           = -1;
    process_           = -1;
    color_             = 0.0;
    collisionState_    = 0;
    remote_            = false;

    map_ = nullptr;
    velocity_      = VECTOR3(p.vx,p.vy,p.vz);
    density_       = p.density;
    restitution_   = p.restitution;
    angle_         = VECTOR3(p.ax,p.ay,p.az);
    angVel_        = VECTOR3(p.avx,p.avy,p.avz);
    shapeId_       = p.ishape;
    iID_           = p.origid;
    remoteID_      = p.origid;
    com_           = VECTOR3(p.x,p.y,p.z);
    force_         = VECTOR3(p.mx,p.my,p.mz);
    torque_        = VECTOR3(p.tx,p.ty,p.tz);
    quat_          = Quaternionr(p.qx,p.qy,p.qz,p.qw);
    elementsPrev_  = 0;
    friction_      = 0.0;

    matTransform_  = quat_.GetMatrix();

    Real entries[9] = {p.a1,p.a2,p.a3,p.a4,p.a5,p.a6,p.a7,p.a8,p.a9};

    std::memcpy(invInertiaTensor_.m_dEntries,entries, sizeof entries);

    dampening_ = 1.0;

    if(p.igrav == 0)
    {
      affectedByGravity_ = false;
      if(shapeId_ == RigidBody::SPHERE)
      {
        shape_ = new Spherer(VECTOR3(0,0,0),p.exx);
        volume_   = p.volume;
        invMass_  = 0.0;
      }
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shape_<<". Please enter a valid shape identifier."<<std::endl;
        std::exit(EXIT_FAILURE);
      }
      invInertiaTensor_.SetZero();
    }
    else if(p.igrav == 1)
    {
      affectedByGravity_ = true;
      if(shapeId_ == RigidBody::SPHERE)
      {
        shape_ = new Spherer(VECTOR3(0,0,0),p.exx);
        volume_   = p.volume;
        invMass_  = p.invmass;
      }
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shape_<<". Please enter a valid shape identifier."<<std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
    else
    {
      std::cerr<<"Invalid value: isAffectedByGravity: "<<shape_<<std::endl;
      std::exit(EXIT_FAILURE);
    }

  }

  RigidBody::RigidBody(BodyStorage *pBody, bool sub)
  {
    velocity_    = pBody->velocity_;
    density_     = pBody->density_;
    restitution_ = pBody->restitution_;
    angle_       = pBody->angle_;
    angVel_      = pBody->angVel_;
    shapeId_     = pBody->shapeId_;
    iID_         = pBody->id_;
    com_         = pBody->com_;
    force_       = pBody->force_;
    torque_      = pBody->torque_;
    quat_        = pBody->quat_;
    
    configureDynamicsType(pBody->dynamicsType_);
    
    elementsPrev_= 0;  
    remote_      = false;
    friction_    = 0.0;
    shape_       = nullptr;
    map_         = nullptr;
    odeIndex_    = -1;

    if(pBody->matrixAvailable_)
    {
      matTransform_ = quat_.GetMatrix();
      transform_.setMatrix(matTransform_);
      transform_.setOrigin(com_);
    }
    else
    {
      setOrientation(angle_);
      matTransform_ = quat_.GetMatrix();
      transform_.setMatrix(matTransform_);
      transform_.setOrigin(com_);
    }

    std::memcpy(invInertiaTensor_.m_dEntries, pBody->tensor_, sizeof pBody->tensor_);

    dampening_ = 1.0;

    if(pBody->affectedByGravity_ == 0)
    {

      configureDynamicsType(pBody->dynamicsType_);

      if(shapeId_ == RigidBody::SPHERE)
      {
        affectedByGravity_ = true;
        shape_ = new Spherer(VECTOR3(0,0,0),pBody->extents_[0]);
        volume_   = shape_->getVolume();
        invMass_ = 1.0 / (density_ * volume_);
      }
      else if(shapeId_ == RigidBody::BOUNDARYBOX)
      {
        //Implement the adding of a boundary box
        shape_ = new BoundaryBoxr(com_,pBody->extents_);
        volume_   = shape_->getVolume();
        invMass_  = 0.0;
      }
      else if(shapeId_ == RigidBody::BOX)
      {
        shape_ = new OBB3r(VECTOR3(0,0,0), pBody->uvw_, pBody->extents_);
        volume_   = shape_->getVolume();
        invMass_  = 0.0;
      }
      else if(shapeId_ == RigidBody::CYLINDER)
      {
        shape_ = new Cylinderr(VECTOR3(0,0,0),VECTOR3(0,0,1),pBody->extents_[0],pBody->extents_[2]);
        volume_   = shape_->getVolume();
        invMass_  = 0.0;
      }
      else if(shapeId_ == RigidBody::MESH)
      {

        affectedByGravity_ = false;

        // Generate a mesh object
        shape_ = new MeshObject<Real>(pBody->useMeshFiles_, pBody->meshFiles_);

        MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(shape_);

        pMeshObject->setFileName(pBody->fileName_);

        volume_   = shape_->getVolume();
        invMass_  = 0.0;

        GenericLoader Loader;
        Loader.readModelFromFile(&pMeshObject->getModel(),pMeshObject->getFileName().c_str());

        pMeshObject->getModel().generateBoundingBox();

        Transformationr tf = getTransformation();
        pMeshObject->initTree(tf);

      }
      else if (shapeId_ == RigidBody::COMPOUND)
      {
        if(sub)
        {
          shape_ = new Spherer(VECTOR3(0, 0, 0), pBody->extents_[0]);
          volume_ = shape_->getVolume();
          invMass_ = 0.0;
        }
        else
        {
          volume_ = pBody->volume_;
          invMass_ = pBody->invMass_;
        }
      }
      else if (shapeId_ == RigidBody::CGALMESH)
      {
        if (!cgal_avail)
        {
          std::cout << "Requesting a CGAL-meshobject, but CGAL is not available." << std::endl;
          std::exit(EXIT_FAILURE);
        }

#ifdef WITH_CGAL
        affectedByGravity_ = false;

        if (pBody->useMeshFiles_)
        {
          shape_ = new MeshObject<Real, cgalKernel>(pBody->useMeshFiles_, pBody->meshFiles_);
        }
        else
        {
          pBody->useMeshFiles_ = true;
          pBody->meshFiles_.push_back(std::string(pBody->fileName_));

          shape_ = new MeshObject<Real, cgalKernel>(pBody->useMeshFiles_, pBody->meshFiles_);
        }


        MeshObject<Real, cgalKernel> *pMeshObject = dynamic_cast<MeshObject<Real, cgalKernel> *>(shape_);

        pMeshObject->setFileName(pBody->fileName_);

        volume_ = shape_->getVolume();

        invMass_ = 0.0;

        pMeshObject->initCgalMeshes();

        Transformationr tf = getTransformation();

        pMeshObject->initTree(tf);
#endif

      }
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shapeId_<<". Please enter a valid shape identifier."<<std::endl;
        std::exit(EXIT_FAILURE);
      }
      invInertiaTensor_.SetZero();
    }
    else if(pBody->affectedByGravity_ == 1)
    {
      affectedByGravity_ = true;

      configureDynamicsType(pBody->dynamicsType_);

      if(shapeId_ == RigidBody::SPHERE)
      {
        shape_ = new Spherer(VECTOR3(0,0,0),pBody->extents_[0]);
        volume_   = shape_->getVolume();
        invMass_  = 1.0/(density_ * volume_);
      }
      else if(shapeId_ == RigidBody::BOUNDARYBOX)
      {
        //Implement the adding of a boundary box
        shape_ = new BoundaryBoxr(com_,pBody->extents_);
        volume_   = shape_->getVolume();
        invMass_  = 0.0;
      }
      else if(shapeId_ == RigidBody::BOX)
      {
        shape_ = new OBB3r(VECTOR3(0,0,0), pBody->uvw_, pBody->extents_);
        volume_   = shape_->getVolume();
        invMass_  = 1.0/(density_ * volume_);
      }
      else if(shapeId_ == RigidBody::CYLINDER)
      {
        shape_ = new Cylinderr(VECTOR3(0,0,0),VECTOR3(0,0,1),pBody->extents_[0],pBody->extents_[2]);
        volume_   = shape_->getVolume();
        invMass_  = 1.0/(density_ * volume_);
      }
      else if(shapeId_ == RigidBody::MESH)
      {

        // Generate a mesh object
        shape_ = new MeshObject<Real>(pBody->useMeshFiles_, pBody->meshFiles_);

        MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(shape_);

        pMeshObject->setFileName(pBody->fileName_);

        setInvInertiaTensorMesh(pBody->fileName_);

        GenericLoader Loader;
        Loader.readModelFromFile(&pMeshObject->getModel(),pMeshObject->getFileName().c_str());

        pMeshObject->getModel().generateBoundingBox();

        Transformationr tf = getTransformation();
        pMeshObject->initTree(tf);

      }
      else if (shapeId_ == RigidBody::CGALMESH)
      {
        if (!cgal_avail)
        {
          std::cout << "Requesting a CGAL-meshobject, but CGAL is not available." << std::endl;
          std::exit(EXIT_FAILURE);
        }

#ifdef WITH_CGAL
        // Generate a mesh object
        shape_ = new MeshObject<Real, cgalKernel>(pBody->useMeshFiles_, pBody->meshFiles_);

        MeshObject<Real, cgalKernel> *pMeshObject = dynamic_cast<MeshObject<Real, cgalKernel> *>(shape_);

        pMeshObject->setFileName(pBody->fileName_);

        setInvInertiaTensorMesh(pBody->fileName_);

        pMeshObject->initCgalMeshes();

        Transformationr tf = getTransformation();
        pMeshObject->initTree(tf);
#endif 

      }
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shape_<<". Please enter a valid shape identifier."<<std::endl;
        std::exit(EXIT_FAILURE);
      }
      //generate the inverted inertia tensor
      generateInvInertiaTensor();
    }
    else
    {
      std::cerr<<"Invalid value: isAffectedByGravity: "<<shape_<<std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  RigidBody::RigidBody(const RigidBody& copy)
  {

    com_               = copy.com_;
    velocity_          = copy.velocity_;
    density_           = copy.density_;
    volume_            = copy.volume_;
    invMass_           = copy.invMass_;
    shape_             = copy.shape_;
    quat_              = copy.quat_;
    invInertiaTensor_  = copy.invInertiaTensor_;
    angVel_            = copy.angVel_;
    angle_             = copy.angle_;
    shape_             = copy.shape_;
    iID_               = copy.iID_;
    collisionState_    = copy.collisionState_;
    restitution_       = copy.restitution_;
    forceResting_      = copy.forceResting_;
    force_             = copy.force_;
    torque_            = copy.torque_;
    dampening_         = copy.dampening_;
    affectedByGravity_ = copy.affectedByGravity_;
    matTransform_      = copy.matTransform_;
    group_             = copy.group_;
    elementsPrev_      = 0;
    remote_            = copy.remote_; 
    friction_          = copy.friction_;
    map_               = copy.map_;
    shapeId_           = copy.shapeId_;
    remoteID_          = copy.remoteID_;

    height_            = copy.height_;
    visited_           = copy.visited_;
    group_             = copy.group_;
    element_           = copy.element_;
    process_           = copy.process_;
    color_             = copy.color_;
    odeIndex_          = copy.color_;

  }
  
  void RigidBody::setInvInertiaTensorMesh(std::string fileName)
  {

    if (fileName == std::string("meshes/swimmer_export.obj"))
    {
      volume_ = 8.22e-3;
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/blood_cell.obj"))
    {
      volume_ = 3.5e-7; //94 micro meter^3
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/dog_small.obj"))
    {
      volume_ = 1.5e-7; // 94.0; //94 micro meter^3
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/capsule.obj"))
    {
      volume_ = 1.3e-6; // 94.0; //94 micro meter^3
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/cone.obj"))
    {
      volume_ = 2.3e-7; // 94.0; //94 micro meter^3
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/cylinder.obj"))
    {
      volume_ = 2.3e-7; // 94.0; //94 micro meter^3
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/torus.obj"))
    {
      volume_ = 3.55e-7;
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/ellipsoid.obj"))
    {
      volume_ = 4.0e-6;
      invMass_ = 1.0 / (density_ * volume_);
    }
    else if (fileName == std::string("meshes/two_particles_zero.obj"))
    {
      volume_ = 6.89478e-8;
      invMass_ = 1.0 / (0.000137896);
      invInertiaTensor_.m_dEntries[7] = 6.3e-11;
      invInertiaTensor_.m_dEntries[5] = 6.3e-11;
    }
    else if (fileName == std::string("meshes/two_particles_zero.off"))
    {
      volume_ = 6.89478e-8;
      invMass_ = 1.0 / (0.000137896);
      invInertiaTensor_.m_dEntries[7] = 6.3e-11;
      invInertiaTensor_.m_dEntries[5] = 6.3e-11;
    }
    else
    {
      volume_ = 0.01303;
      invMass_ = 1.0 / (density_ * volume_);
    }

  }

  void RigidBody::translateTo(const VECTOR3 &vPos)
  {
    com_ = vPos;
  }

  void RigidBody::generateInvInertiaTensor()
  {
    AABB3r box;
    if(shapeId_ == RigidBody::SPHERE)
    {
      //calculate the inertia tensor
      //Get the inertia tensor
      box = shape_->getAABB();
      Real rad2 = box.extents_[0]*box.extents_[0];
      Real dnum  = 5.0f*invMass_; 
      Real denom = 2.f*rad2;
      invInertiaTensor_ = MATRIX3X3(dnum/denom, 0, 0, 0, dnum/denom, 0, 0, 0, dnum/denom);
    }
    else if(shapeId_ == RigidBody::BOX)
    {
      box = shape_->getAABB();
      Real dwmass = 12.0 * invMass_; 
      Real w2 = 4.0 * box.extents_[0]*box.extents_[0];
      Real h2 = 4.0 * box.extents_[1]*box.extents_[1];
      Real d2 = 4.0 * box.extents_[2]*box.extents_[2];
      invInertiaTensor_ = MATRIX3X3(dwmass/(h2+d2), 0, 0, 0, dwmass/(w2+d2), 0, 0, 0, dwmass/(w2+h2));
    }
    else if(shapeId_ == RigidBody::CYLINDER)
    {
      box = shape_->getAABB();
      Real sqrrad = box.extents_[0]*box.extents_[0];
      Real sqrh   = box.extents_[2]*box.extents_[2] * 4.0;
      Real dmass = 12*invMass_;
      Real m3rh = dmass * (1.0/(3.0*sqrrad+sqrh));
      invInertiaTensor_ = MATRIX3X3(m3rh, 0, 0, 0, m3rh, 0, 0, 0, 2.0*invMass_*(1.0/sqrrad));
    }
    else if(shapeId_ == RigidBody::CGALMESH)
    {
      invInertiaTensor_.m_dEntries[7] = 6.3e-11;
      invInertiaTensor_.m_dEntries[5] = 6.3e-11;
    }
    else if(shapeId_ == RigidBody::BOUNDARYBOX)
    {
      invInertiaTensor_.SetZero();
    }
    else if(shapeId_ == RigidBody::MESH)
    {
      //default torus:
      //I_zz=(rad_xz^2*3/4 + rad_xy^2)*m
      //I_xx=I_yy=(5*rad_xz^2+4*rad_xy^2)*m
      //Volume = 2*pi^2*rad_xz^2*rad_xy

      MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(shape_);

      if(pMeshObject->getFileName()=="meshes/swimmer_export.obj")
      {     
        Real xx =1.82e-4;
        Real yy =1.82e-4;
        Real zz =9.21e-5;
        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);      
      }
      else if(pMeshObject->getFileName()=="meshes/cow.obj")
      {
        Real xx =8.67142e-004;
        Real yy =3.68183e-003;
        Real zz =3.33655e-003;
        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);
      }
      else if (pMeshObject->getFileName() == "meshes/blood_cell.obj")
      {
//        Real xx =1.82e-4;
//        Real yy =1.82e-4;
//        Real zz =9.21e-5;

        Real m = 1.0/invMass_;
        Real xx = m * (0.00391 * 0.00391 + 0.00128 * 0.00128) * 0.2;
        Real yy = xx;
        Real zz = m * (0.00391 * 0.00391 + 0.00391 * 0.00391) * 0.2;

        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);      
      }
      else if (pMeshObject->getFileName() == "meshes/dog_small.obj")
      {
        Real xx = 3.0e-12;
        Real yy = 5.0e-12;
        Real zz = 2.5e-12;
        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/cone.obj")
      {

	    MeshObject<Real> *pMeshObject = dynamic_cast<MeshObject<Real>*>(shape_);
    	Real h = 2.0 * pMeshObject->getAABB().extents_[2];
    	Real r = 1.0 * pMeshObject->getAABB().extents_[1];
    	Real m = volume_ * density_;

        Real xx = (1.0/10.0) * m * h * h + (3.0/20.0) * m * r * r;

        Real yy = (1.0/10.0) * m * h * h + (3.0/20.0) * m * r * r;

        Real zz = (3.0/10.0) * m * r * r;

        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/ellipsoid.obj")
      {

        Real myPi = 4.0 * std::atan(1);
	    MeshObject<Real> *pMeshObject = dynamic_cast<MeshObject<Real>*>(shape_);
    	Real a = 1.0 * pMeshObject->getAABB().extents_[0];
    	Real b = 1.0 * pMeshObject->getAABB().extents_[1];
    	Real c = 1.0 * pMeshObject->getAABB().extents_[2];

    	volume_ = 4.0/3.0 * myPi * a * b * c;
    	Real m = volume_ * density_;
    	invMass_ = 1.0/m;

        Real xx = (1.0/5.0) * m * (b * b + c * c);

        Real yy = (1.0/5.0) * m * (a * a + c * c);

        Real zz = (1.0/5.0) * m * (b * b + a * a);

        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/cylinder.obj")
      {
      	Real myPi = 4.0 * std::atan(1);
	    MeshObject<Real> *pMeshObject = dynamic_cast<MeshObject<Real>*>(shape_);
    	Real h = 2.0 * pMeshObject->getAABB().extents_[2];
    	Real r = 1.0 * pMeshObject->getAABB().extents_[1];
    	volume_ = myPi * r * r * h;
    	Real m = volume_ * density_;

    	invMass_ = 1.0 / m;

        Real xx = (1.0/12.0) * m * (3.0 * r * r + h *h);

        Real yy = (1.0/12.0) * m * (3.0 * r * r + h *h);

        Real zz = 0.5 * m * r * r;

        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/torus.obj")
      {

    	Real a = 0.002;
    	Real c = 0.005;

    	volume_ = 3.55e-7;
    	Real m = volume_ * density_;

    	invMass_ = 1.0 / m;

        Real xx = ((5.0/8.0) * a * a + 0.5 * c * c) * m;

        Real yy = ((5.0/8.0) * a * a + 0.5 * c * c) * m;

        Real zz = ((3.0/4.0) * a * a + 0.5 * c * c) * m;

        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/capsule.obj")
      {

        MeshObject<Real> *pMeshObject = dynamic_cast<MeshObject<Real>*>(shape_);
        Real h  = 2.0 * pMeshObject->getAABB().extents_[2];
        Real r  = 1.0 * pMeshObject->getAABB().extents_[1];
        Real r2 = r * r;

        Real myPi = 4.0 * std::atan(1);

        Real cM = myPi * h * r2 * density_;

        Real hsM = 2.0 * myPi * (1.0/3.0) * r2 * r * density_;

        Real zz = 0.0;

        Real yy = r2 * cM * 0.5;

        Real xx = zz = yy * 0.5 + cM * h * h * (1.0/12.0);

        Real temp0 = hsM * 2.0 * r2 / 5.0;

    	  yy += temp0 * 2.0;

    	  Real temp1 = h*0.5;

    	  Real temp2 = temp0 + hsM*(temp1*temp1 + 3.0* (1.0/8.0) * h * r);

    	  xx += temp2 * 2.0;

    	  zz += temp2 * 2.0;

    	  Real mass = cM + hsM * 2.0;
    	  this->invMass_ = 1.0/mass;

        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }
      else if (pMeshObject->getFileName() == "meshes/box.obj")
      {

        MeshObject<Real> *pMeshObject = dynamic_cast<MeshObject<Real>*>(shape_);

    	  Real w  = 2.0 * pMeshObject->getAABB().extents_[0];
    	  Real h  = 2.0 * pMeshObject->getAABB().extents_[1];
    	  Real d  = 2.0 * pMeshObject->getAABB().extents_[2];

        volume_ = w * h * d;

        Real mass = density_ * volume_;

        invMass_ = mass;

        Real w2  = w * w;
        Real h2  = h * h;
        Real d2  = d * d;

        Real dwmass = 12.0 * invMass_; 

        invInertiaTensor_ = MATRIX3X3(dwmass/(h2+d2), 0, 0, 0, dwmass/(w2+d2), 0, 0, 0, dwmass/(w2+h2));

      }
      else
      {
        Real xx = 1.82e-4;
        Real yy = 1.82e-4;
        Real zz = 9.21e-5;
        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }

    }

  }

  Real RigidBody::getEnergy()
  {
    Real velEnergy = 0.0;
    if(affectedByGravity_)
    {
      velEnergy = 1.0/invMass_ * (velocity_ * velocity_);
      //Real angEnergy = GetInertiaTensor * (m_vAngVel * m_vAngVel);
    }
    return velEnergy;
  }

  MATRIX3X3 RigidBody::getWorldTransformedInvTensor()
  {
    //transform the inertia tensor to world space
    MATRIX3X3 mMatWorldTensor = matTransform_ * invInertiaTensor_;// * matTransform_.GetTransposedMatrix();
    return mMatWorldTensor;
  }


  const Shaper& RigidBody::getOriginalShape() const
  {
    if(shapeId_ == RigidBody::BOX)
    {
      const OBB3r &pBox = dynamic_cast<const OBB3r&>(*shape_);
      return pBox;
    }
    if(shapeId_ == RigidBody::SPHERE)
    {
      const Spherer &sphere = dynamic_cast<const Spherer&>(*shape_);
      return sphere;
    }
    if(shapeId_ == RigidBody::CYLINDER)
    {
      const Cylinderr &pCylinder = dynamic_cast<const Cylinderr&>(*shape_);
      return pCylinder;
    }
    else
    {
      const BoundaryBoxr &pBoundary = dynamic_cast<const BoundaryBoxr&>(*shape_);
      return pBoundary;
    }
  }

  Shaper* RigidBody::getWorldTransformedShape()
  {

    Shaper *pShape = nullptr;

    if(shapeId_ == RigidBody::SPHERE)
    {
      pShape = new Spherer(com_,shape_->getAABB().extents_[0]);
    }
    else if(shapeId_ == RigidBody::BOX)
    {
      //pBox = CoordTransform().transform(dynamic_cast<OBB3r *>(pBody0->m_pShape),pBody->CoM,pBody->GetOrientMatrix());
      OBB3r *pBox = dynamic_cast<OBB3r *>(shape_);

      //get the transformed vectors
      VECTOR3 vUVW[3];
      VECTOR3 vWorld;

      //transform
      for(int i=0;i<3;i++)
      {
        vWorld = pBox->uvw_[i];
        vWorld = matTransform_*vWorld;
        vUVW[i] = vWorld;
      }

      //the world tranformed box
      pShape = new OBB3r(com_,vUVW,pBox->extents_);
    }
    else if(shapeId_ == RigidBody::CYLINDER)
    {
      Cylinderr *pCylinder = dynamic_cast<Cylinderr *>(shape_);

      VECTOR3 u = matTransform_*pCylinder->getU();

      //the world tranformed cylinder
      pShape = new Cylinderr(com_,u,pCylinder->getRadius(),pCylinder->getHalfLength());
    }
    else if(shapeId_ == RigidBody::MESH)
    {
      std::cout << "Function getWorldTransformedShape should not be executed for meshes." << std::endl;
      std::exit(EXIT_FAILURE);
      //pMesh->m_Model=pMeshObject->m_Model;
      //pMesh->m_Model.meshes_[0].transform_ =matTransform_;
      //pMesh->m_Model.meshes_[0].com_ =com_;
      //pMesh->m_Model.meshes_[0].TransformModelWorld();
      //pMesh->m_Model.meshes_[0].generateBoundingBox();
      //pMesh->m_Model.box_ = pMesh->m_Model.meshes_[0].box_;
      ////the world tranformed mesh
      return pShape;
    }
    else if(shapeId_ == RigidBody::BOUNDARYBOX)
    {
      pShape =  new BoundaryBoxr();
    }
    return pShape;
  }

  Shaper* RigidBody::getWorldTransformedShapeNext(Real dT)
  {

    Shaper *pShape = nullptr;

    if(shapeId_ == RigidBody::SPHERE)
    {
      VECTOR3 newCoM = com_ + velocity_ * dT;
      pShape = new Spherer(newCoM,shape_->getAABB().extents_[0]);
    }
    else if(shapeId_ == RigidBody::BOX)
    {
      OBB3r *pBox = dynamic_cast<OBB3r *>(shape_);
      MATRIX3X3 mrotMat;
      VECTOR3 vWorld;

      //update the position
      VECTOR3 newpos = com_ + velocity_ * dT;

      //update angle
      VECTOR3 newangle = angle_ + angVel_ * dT;

      //get the rotation matrix
      mrotMat.MatrixFromAngles(newangle);
      VECTOR3 vUVW[3];

      //transform
      for(int i=0;i<3;i++)
      {
        vWorld = pBox->uvw_[i];
        vWorld = mrotMat*vWorld;
        vUVW[i] = vWorld;
      }

      //the world tranformed box
      pShape = new OBB3r(newpos,vUVW,pBox->extents_);
    }
    else if(shapeId_ == RigidBody::BOUNDARYBOX)
    {
      pShape =  new BoundaryBoxr();
    }

    return pShape;

  }

  Transformationr RigidBody::getTransformation() const
  {
    return Transformationr(matTransform_,com_);
  }

  VECTOR3 RigidBody::getAxisAngle()
  {
    VECTOR3 vAxis;
    //compute sine and cosine values
    double c1 = cos(angle_.x/2);//heading
    double s1 = sin(angle_.x/2);//heading
    double c2 = cos(angle_.y/2);//attitude
    double s2 = sin(angle_.y/2);//attitude
    double c3 = cos(angle_.z/2);//bank
    double s3 = sin(angle_.z/2);//bank
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    double w =c1c2*c3 - s1s2*s3;
    double x =c1c2*s3 + s1s2*c3;
    double y =s1*c2*c3 + c1*s2*s3;
    double z =c1*s2*c3 - s1*c2*s3;
    double angle;
    if(fabs(w)-1.0 < CMath<double>::TOLERANCEZERO)
    {
      if(w < 0)
        w=-1.0;
      else
        w=1.0;
      angle = 2 * acos(w);
    }
    else
    {
      angle = 2 * acos(w);
    }
    double norm = x*x+y*y+z*z;
    if (norm < 0.001)
    {
      // take care of zero division
      // set axis to arbitrary unit vector
      x=1;
      y=0;
      z=0;
    }
    else
    {
      norm = sqrt(norm);
      x /= norm;
      y /= norm;
      z /= norm;
    }

    //adjust to the xyz-convention
    vAxis.x = y;
    vAxis.y = z;
    vAxis.z = x;

    vAxis*=angle;

    return vAxis;

  }//end GetAxisAngle

  void RigidBody::updateAngVel(const VECTOR3 &delta) 
  {
    angVel_ += delta;
  }

  void RigidBody::applyImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
  {
    MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

    velocity_ += linearUpdate;

    angVel_   += mInvInertiaTensor * (VECTOR3::Cross(relPos,impulse));

  }

  void RigidBody::applyBiasImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
  {

    MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

    biasVelocity_ += linearUpdate;

    biasAngVel_  += mInvInertiaTensor * (VECTOR3::Cross(relPos,impulse));

  }

  bool RigidBody::isInBody(const VECTOR3 &vQuery) const
  {
    if(shapeId_ == RigidBody::MESH)
    {
      // for meshes we calculate in world coordinates
      // sometimes... 
      Vec3 mid1(0.0000076, 0.00616911, 0.0011843);	  
      Vec3 mid2(0.0000076, 0.009777, 0.00214776);	  
      Real r1(0.0024);
      Real r2(0.0014);
      VECTOR3 vLocal = vQuery - com_;
      MATRIX3X3 trans = matTransform_;
      trans.TransposeMatrix();
      vLocal = trans * vLocal ;
      if( ((mid1 - vLocal).mag() <= r1) || ((mid2 - vLocal).mag() <= r2))
        return true;
      else
        return false;
    }
    else if(shapeId_ == RigidBody::CYLINDER)
    {
      // for other shapes we transform to local coordinates
      VECTOR3 vLocal = vQuery - com_;
      MATRIX3X3 trans = matTransform_;
      trans.TransposeMatrix();
      vLocal = trans * vLocal ;
      return (shape_->isPointInside(vLocal));
    }
    else
    {
      // for other shapes we transform to local coordinates
      VECTOR3 vLocal = vQuery - com_;
      MATRIX3X3 trans = matTransform_;
      trans.TransposeMatrix();
      vLocal = trans * vLocal ;
      return (shape_->isPointInside(vLocal));
    }
  }

  MATRIX3X3 RigidBody::getTransformationMatrix() const
  {
    return matTransform_;
  }

  void RigidBody::setTransformationMatrix(const MATRIX3X3& mat)
  {
    matTransform_ = mat;
  }

  bool RigidBody::isInBody(const Vec3 & vQuery, const Vec3 & vDir) const
  {

    if (shapeId_ == RigidBody::CGALMESH)
    {
#ifdef WITH_CGAL
      MeshObject<Real, cgalKernel> *pMeshObject = dynamic_cast<MeshObject<Real, cgalKernel> *>(shape_);

      Vec3 vLocal = vQuery - com_;
      MATRIX3X3 trans = matTransform_;
      vLocal = trans * vLocal ;
      return pMeshObject->isPointInside(vLocal, vDir);
#else 
    return true;
#endif 
    }
    else
      return false;

  }

  Real RigidBody::getMinimumDistance(const Vec3 & vQuery) const
  {

    if (shapeId_ == RigidBody::CGALMESH)
    {

#ifdef WITH_CGAL
      MeshObject<Real, cgalKernel> *pMeshObject = dynamic_cast<MeshObject<Real, cgalKernel> *>(shape_);

      return pMeshObject->getMinimumDistance(vQuery);
#else
      return Real();
#endif
    }
    else
      return Real();
  }

  std::pair<Real, Vec3> RigidBody::getClosestPoint(const Vec3 & vQuery) const
  {
    if (shapeId_ == RigidBody::CGALMESH)
    {
#ifdef WITH_CGAL
      MeshObject<Real, cgalKernel> *pMeshObject = dynamic_cast<MeshObject<Real, cgalKernel> *>(shape_);
      return pMeshObject->getClosestPoint(vQuery);
#else
      return std::make_pair<Real, Vec3>(Real(), Vec3());
#endif
    }
    else
      return std::make_pair<Real, Vec3>(Real(), Vec3());
  }

  void RigidBody::removeEdge(CollisionInfo *pInfo)
  {
    std::list<CollisionInfo*>::iterator i = edges_.begin();
    while(i!=edges_.end())
    {
      CollisionInfo *collinfo = (*i);
      if(collinfo->iID1 == pInfo->iID1 && collinfo->iID2 ==pInfo->iID2)
      {
        i=edges_.erase(i);
        break;
      }//end if
      i++;
    }//end while

  }//end Remove edge

  void RigidBody::buildDistanceMap()
  {

    Real size = getBoundingSphereRadius();
    Real size2 = shape_->getAABB().extents_[shape_->getAABB().longestAxis()] + 0.1f * size;
    //Real cellSize = 2.0 * size2 / 64.0f;
    Real cellSize = 2.0 * size2 / 128.0f;

    Real _x = 2.0 * (shape_->getAABB().extents_[0] + 0.1f * size);
    Real _y = 2.0 * (shape_->getAABB().extents_[1] + 0.1f * size);
    Real _z = 2.0 * (shape_->getAABB().extents_[2] + 0.1f * size);

    Real lx = (shape_->getAABB().extents_[0]);
    Real ly = (shape_->getAABB().extents_[1]);
    Real lz = (shape_->getAABB().extents_[2]);

    std::cout << "> Bounding box volume: " << 2.0 * lx * 2.0 * ly * 2.0 * lz << std::endl;
    std::cout << "> Size box: " << Vec3(lx,ly,lz) << std::endl;

    //shape_->getAABB().Output();
    VECTOR3 boxCenter = shape_->getAABB().center_;

    int nCells[3] = {int(_x/cellSize), int(_y/cellSize), int(_z/cellSize)};
    std::cout << "> Cells [" << nCells[0] << "," << nCells[1] << "," << nCells[2] << "]" << std::endl;

//    int cells[3]={(myBox.extents_[0]/cellSize),
//                  (myBox.extents_[1]/cellSize),
//                  (myBox.extents_[2]/cellSize)};

    Real extents[3] = {0.5 * _x, 0.5 * _y, 0.5 * _z};

    //AABB3r myBox(boxCenter,size2); 
    AABB3r myBox(boxCenter, extents); 

    int cells[3]={64, 64, 64};

    map_ = new DistanceMap<Real>(myBox,nCells);
   
    MeshObject<Real> *object = dynamic_cast< MeshObject<Real> *>(shape_);

    Model3D &model = object->getModel();  

    Model3D model_out_0(model);
    model_out_0.meshes_[0].transform_.SetIdentity();
    model_out_0.meshes_[0].com_ = VECTOR3(0,0,0);
    model_out_0.generateBoundingBox();

    std::vector<Triangle3r> pTriangles = model_out_0.genTriangleVector();
    CSubDivRessources myRessources_dm(1,7,0,model_out_0.getBox(),&pTriangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);

    CBoundingVolumeTree3<AABB3r,Real,CTraits,CSubdivisionCreator> bvh;
    bvh.InitTree(&subdivider_dm);
    int total = (map_->cells_[0]+1)*(map_->cells_[1]+1)*(map_->cells_[2]+1);

    for(int i=0;i<total;i++)
    {
      VECTOR3 vQuery=map_->vertexCoords_[i];

      CDistanceFuncGridModel<Real> distFunc;

      bool inside = false;
//      if (distFunc.BruteForceInnerPointsStatic(model_out_0, vQuery) == 1)
//        inside = true;

//      if(isInBody(vQuery))
//      {
//        map_->stateFBM_[i]=1;    
//      }
//      else
//      {
//        map_->stateFBM_[i]=0;          
//      }

      CDistanceMeshPoint<Real> distMeshPoint(&bvh,vQuery);
      //map_->distance_[i] =  distMeshPoint.ComputeDistance();
//      map_->distance_[i] = distMeshPoint.ComputeDistanceBruteForce();
//      

//      if(map_->stateFBM_[i])
//        map_->distance_[i]*=-1.0;

//      map_->normals_[i] = vQuery - distMeshPoint.m_Res.m_vClosestPoint;

//      map_->contactPoints_[i] = distMeshPoint.m_Res.m_vClosestPoint;    

      if(i%1000==0)
      {
        double percent = (double(i) / total) * 100.0;
        std::cout << "> Progress: " << static_cast<int>(percent) << "%" << std::flush;
        std::cout << "\r";
      }

    }
    std::cout << "> Progress: " << 100 << "%" << std::flush;
    std::cout << std::endl;

#ifdef FC_CUDA_SUPPORT
//    transfer_distancemap(this, map_);
#endif
    
    //for (int z = 0; z < map_->cells_[2]; ++z)
    //{
    //  for (int y = 0; y < map_->cells_[1]; ++y)
    //  {
    //    for (int x = 0; x < map_->cells_[0]; ++x)
    //    {
    //      int indices[8];
    //      map_->vertexIndices(x, y, z, indices);
    //      Vec3 center(0, 0, 0);
    //      int in = 1;
    //      for (int k = 0; k < 8; ++k)
    //      {
    //        center += map_->vertexCoords_[indices[k]];
    //        in = in & map_->stateFBM_[indices[k]];
    //      }
    //      if (in)
    //      {
    //        center *= 0.125;
    //        Real rad = (map_->vertexCoords_[indices[0]] - center).mag();
    //        Spherer sphere(center, rad);
    //        spheres.push_back(sphere);
    //      }

    //    }
    //  }
    //}

    //CVtkWriter writer;
    //writer.WriteSphereFile(spheres, "output/st.vtk");

  }


  void RigidBody::buildDistanceMapFromFile(std::string fileName)
  {

    using namespace std;
    ifstream myfile(fileName);
    if(!myfile.is_open())
    {
      cout<<"Error opening file: "<<fileName<<endl;
      std::exit(1);
    }//end if


    char line[1024];
    int dim[2];
    int cells[3];

    myfile >> dim[0] >> dim[1];
    myfile.getline(line,1024);
    myfile >> cells[0] >> cells[1] >> cells[2];
    myfile.getline(line,1024);

    int _x = cells[0]+1;
    int _y = cells[1]+1;
    int _z = cells[2]+1;
    int _size = _x * _y * _z; 

    Real size = getBoundingSphereRadius();
    Real x =  (shape_->getAABB().extents_[0] + 0.1f * size);
    Real y =  (shape_->getAABB().extents_[1] + 0.1f * size);
    Real z =  (shape_->getAABB().extents_[2] + 0.1f * size);

    //shape_->getAABB().Output();
    VECTOR3 boxCenter = shape_->getAABB().center_;

    Real extents[3] = {x, y, z};

//    Real extends[3];
//    extends[0]=size2;
//    extends[1]=shape_->getAABB().extents_[1] + 0.1f * size;
//    extends[2]=shape_->getAABB().extents_[1] + 0.1f * size;
    AABB3r myBox(boxCenter,extents); 

    printf("first: cells.x=%i cells.y=%i cells.z=%i\n",cells[0],cells[1],cells[2]);

    int ncells[3]{cells[0],cells[1],cells[2]};

    map_ = new DistanceMap<Real>(myBox,ncells);

    map_->boundingBox_ = myBox;
    map_->dim_[0] = dim[0];
    map_->dim_[1] = dim[1];

    map_->cells_[0] = cells[0];
    map_->cells_[1] = cells[1];
    map_->cells_[2] = cells[2];

    myfile >> map_->cellSize_;
    myfile.getline(line,1024);

    int s = _size;
    for(int i(0); i < s; ++i)
    { 
      myfile >> map_->vertexCoords_[i].x
             >> map_->vertexCoords_[i].y 
             >> map_->vertexCoords_[i].z;
      myfile.getline(line,1024);
    }

    for(int i(0); i < s; ++i)
    { 
      myfile >> map_->contactPoints_[i].x
             >> map_->contactPoints_[i].y 
             >> map_->contactPoints_[i].z;
      myfile.getline(line,1024);
    }

    for(int i(0); i < s; ++i)
    { 
      myfile >> map_->normals_[i].x
             >> map_->normals_[i].y 
             >> map_->normals_[i].z;
      myfile.getline(line,1024);
    }

    for(int i(0); i < s; ++i)
    { 
      myfile >> map_->distance_[i];
      myfile.getline(line,1024);
    }

    for(int i(0); i < s; ++i)
    { 
      myfile >> map_->stateFBM_[i];
      myfile.getline(line,1024);
    }

    myfile.close();
    std::cout << "> Finished reading distance map" << std::endl;
    std::cout << "> Size: " << map_->cells_[0] << " " 
                            << map_->cells_[1] << " " 
                            << map_->cells_[2] << std::endl;

  }

  void RigidBody::storeDistanceMapToFile(std::string fileName)
  {

    using namespace std;
    ofstream myfile(fileName);

    if(!myfile.is_open())
    {
      cout<<"Error opening file: "<<fileName<<endl;
      std::exit(EXIT_FAILURE);
    }//end if


    myfile << map_->dim_[0] << " " << map_->dim_[1] << endl;

    myfile << map_->cells_[0] << " " << map_->cells_[1] << " " << map_->cells_[2] << endl;

    myfile << map_->cellSize_ << endl;

    int _x = map_->cells_[0]+1;
    int _y = map_->cells_[1]+1;
    int _z = map_->cells_[2]+1;
    int _size = _x * _y * _z; 

    int s = _size;
    for(int i(0); i < s; ++i)
    { 
      myfile << map_->vertexCoords_[i].x << " "
             << map_->vertexCoords_[i].y << " "  
             << map_->vertexCoords_[i].z << endl;
    }

    for(int i(0); i < s; ++i)
    { 
      myfile << map_->contactPoints_[i].x << " "
             << map_->contactPoints_[i].y << " " 
             << map_->contactPoints_[i].z << endl;
    }

    for(int i(0); i < s; ++i)
    { 
      myfile << map_->normals_[i].x << " "
             << map_->normals_[i].y << " " 
             << map_->normals_[i].z << endl;
    }

    for(int i(0); i < s; ++i)
    { 
      myfile << map_->distance_[i] << endl;
    }

    for(int i(0); i < s; ++i)
    { 
      myfile << map_->stateFBM_[i] << endl;
    }

    myfile.close();

  }

  void RigidBody::applyForces(const VECTOR3 &force, const VECTOR3 &torque){

    MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

    //update velocity of the compound
    velocity_ += force;

    //and the angular velocity
    setAngVel(getAngVel() + mInvInertiaTensor * torque);

  }

}
