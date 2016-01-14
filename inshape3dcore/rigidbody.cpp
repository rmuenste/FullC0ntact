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
#ifdef FC_CUDA_SUPPORT
#include <cuda_runtime.h>
#include <difi.cuh>
#include <vtkwriter.h>
#endif

namespace i3d {

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
  }

  RigidBody::~RigidBody()
  {
    if(shape_ != NULL)
    {
      delete shape_;
      shape_ = NULL;
    }
  }

  RigidBody::RigidBody(VECTOR3 velocity, Real density, Real volume, Real mass, VECTOR3 angle, int shapeId) : collisionState_(0)
  {
    this->velocity_=velocity;
    this->density_ = density;
    this->volume_  = volume;
    this->invMass_ = mass;
    this->angle_   = angle;
    this->shapeId_   = shapeId;

    shape_             = nullptr;
    friction_ = 0.0;
    affectedByGravity_ = true;
    map_ = nullptr;
    dampening_     = 1.0;
    remote_            = false;
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
    elementsPrev_      = 0;

  }

  RigidBody::RigidBody(Shaper *shape, int shapeId)
  {
    shapeId_           = shapeId;
    shape_             = shape;
    affectedByGravity_ = true;
    remote_            = false;    
    friction_          = 0.0;
    dampening_ = 1.0;
    map_ = nullptr;
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

    memcpy(invInertiaTensor_.m_dEntries,entries,9*sizeof(Real));

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
        exit(0);
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
        exit(0);
      }
    }
    else
    {
      std::cerr<<"Invalid value: isAffectedByGravity: "<<shape_<<std::endl;
      exit(0);
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
    elementsPrev_= 0;  
    remote_      = false;
    friction_    = 0.0;
    shape_ = NULL;
    map_ = nullptr;

    if(pBody->matrixAvailable_)
    {
      matTransform_ = quat_.GetMatrix();
    }
    else
    {
      setOrientation(angle_);
      matTransform_ = quat_.GetMatrix();
    }

    memcpy(invInertiaTensor_.m_dEntries,pBody->tensor_,9*sizeof(Real));

    dampening_ = 1.0;

    if(pBody->affectedByGravity_ == 0)
    {
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
        shape_ = new CMeshObject<Real>();
        CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(shape_);
        pMeshObject->SetFileName(pBody->fileName_);
        volume_   = shape_->getVolume();
        invMass_  = 0.0;

        GenericLoader Loader;
        Loader.readModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

        pMeshObject->m_Model.GenerateBoundingBox();
        for(unsigned i=0;i< pMeshObject->m_Model.meshes_.size();i++)
        {
          pMeshObject->m_Model.meshes_[i].generateBoundingBox();
        }

        Model3D model_out_0(pMeshObject->m_Model);
        model_out_0.meshes_[0].transform_ = getTransformationMatrix();
        model_out_0.meshes_[0].com_ = com_;
        model_out_0.meshes_[0].TransformModelWorld();
        model_out_0.GenerateBoundingBox();
        model_out_0.meshes_[0].generateBoundingBox();
        std::vector<Triangle3r> pTriangles = model_out_0.GenTriangleVector();
        CSubDivRessources myRessources_dm(1,4,0,model_out_0.GetBox(),&pTriangles);
        CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
        pMeshObject->m_BVH.InitTree(&subdivider_dm);      

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
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shapeId_<<". Please enter a valid shape identifier."<<std::endl;
        exit(0);
      }
      invInertiaTensor_.SetZero();
    }
    else if(pBody->affectedByGravity_ == 1)
    {
      affectedByGravity_ = true;
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
        shape_ = new CMeshObject<Real>();
        CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(shape_);
        pMeshObject->SetFileName(pBody->fileName_);

        if(pBody->fileName_==std::string("meshes/swimmer_export.obj"))
        {     
          volume_   = 8.22e-3;
          invMass_  = 1.0/(density_ * volume_);
        }
        else if(pBody->fileName_==std::string("meshes/blood_cell.obj"))
        {
          volume_   = 94.0; //94 micro meter^3
          invMass_  = 1.0/(density_ * volume_);
        }
        else
        {
          volume_   = 0.01303;        
          invMass_  = 1.0/(density_ * volume_);
        }

        GenericLoader Loader;
        Loader.readModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

        pMeshObject->m_Model.GenerateBoundingBox();
        for(unsigned i=0;i< pMeshObject->m_Model.meshes_.size();i++)
        {
          pMeshObject->m_Model.meshes_[i].generateBoundingBox();
        }

        Model3D model_out_0(pMeshObject->m_Model);
        model_out_0.meshes_[0].transform_ = getTransformationMatrix();
        model_out_0.meshes_[0].com_ = com_;
        model_out_0.meshes_[0].TransformModelWorld();
        model_out_0.GenerateBoundingBox();
        model_out_0.meshes_[0].generateBoundingBox();
        std::vector<Triangle3r> pTriangles = model_out_0.GenTriangleVector();
        CSubDivRessources myRessources_dm(1,4,0,model_out_0.GetBox(),&pTriangles);
        CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
        pMeshObject->m_BVH.InitTree(&subdivider_dm);     

      }
      else
      {
        std::cerr<<"Unknown shape identifier: "<<shape_<<". Please enter a valid shape identifier."<<std::endl;
        exit(0);
      }
      //generate the inverted inertia tensor
      generateInvInertiaTensor();
    }
    else
    {
      std::cerr<<"Invalid value: isAffectedByGravity: "<<shape_<<std::endl;
      exit(0);
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

      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(shape_);

      if(pMeshObject->GetFileName()=="meshes/swimmer_export.obj")
      {     
        Real xx =1.82e-4;
        Real yy =1.82e-4;
        Real zz =9.21e-5;
        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);      
      }
      else if(pMeshObject->GetFileName()=="meshes/cow.obj")
      {
        Real xx =8.67142e-004;
        Real yy =3.68183e-003;
        Real zz =3.33655e-003;
        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);
      }
      else if (pMeshObject->GetFileName() == "meshes/blood_cell.obj")
      {
        Real xx =1.82e-4;
        Real yy =1.82e-4;
        Real zz =9.21e-5;
        invInertiaTensor_ = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);      
      }
      else
      {
        Real xx = 1.82e-4;
        Real yy = 1.82e-4;
        Real zz = 9.21e-5;
        invInertiaTensor_ = MATRIX3X3(1.0 / xx, 0, 0, 0, 1.0 / yy, 0, 0, 0, 1.0 / zz);
      }

      //Real xx =8.67142e-004;
      //Real yy =3.68183e-003;
      //Real zz =3.33655e-003;
      //m_InvInertiaTensor = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);

      //Real rad_xy = 0.1;
      //Real rad_xz = 0.01;
      //Real rad_xy2 = 0.1*0.1;
      //Real rad_xz2 = 0.01*0.01;
      //Real dmass = 1.0/m_dInvMass;
      //m_InvInertiaTensor = MATRIX3X3((5.0*rad_xz2+4.0*rad_xy2)*dmass, 0, 0, 0, (5.0*rad_xz2+4.0*rad_xy2)*dmass, 0, 0, 0, (rad_xz2*3.0/4.0 + rad_xy2)*dmass);

      //Volume_approx:   3.66973876953125000E-002
      //Volume_ref:   6.54498469497873520E-002
      //calculating characteristic function...
      //Calculating Moment of Inertia...
      //MOI_X:   8.67142652471614606E-004
      //MOI_Y:   3.68188073237738205E-003
      //MOI_Z:   3.33655563493564242E-003
      //MOI_ref:   3.27249234748936768E-003
      //COG:  -7.63318607068621535E-002  4.71023908523768146E-005  8.67008835758858662E-003

      //cow volume 0.01303

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
    MATRIX3X3 mMatWorldTensor = matTransform_ * invInertiaTensor_ * matTransform_.GetTransposedMatrix();
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
      CMeshObject<Real> *pMeshObject = dynamic_cast<CMeshObject<Real>*>(shape_);
      CMeshObject<Real> *pMesh= new CMeshObject<Real>();
      pMesh->m_Model=pMeshObject->m_Model;
      pMesh->m_Model.meshes_[0].transform_ =matTransform_;
      pMesh->m_Model.meshes_[0].com_ =com_;
      pMesh->m_Model.meshes_[0].TransformModelWorld();
      pMesh->m_Model.meshes_[0].generateBoundingBox();
      pMesh->m_Model.box_ = pMesh->m_Model.meshes_[0].box_;
      //the world tranformed mesh
      pShape = pMesh;
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
    //for meshes we calculate in world coordinates
    if(shapeId_ == RigidBody::MESH)
      return (shape_->isPointInside(vQuery));
    else
    {
      //for other shapes we transform to local coordinates
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
    shape_->getAABB().Output();
    VECTOR3 boxCenter = shape_->getAABB().center_;

    std::cout << "size2: " << size2 << std::endl;

    Real extends[3];
    extends[0]=size;
    extends[1]=size;
    extends[2]=size;
    AABB3r myBox(boxCenter,size2); 
    map_ = new DistanceMap<Real>(myBox,32);
   
    CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(shape_);
    //if (object->m_BVH.GetChild(0) == NULL)
    //{

    //}
    Model3D &model = object->m_Model;  

    Model3D model_out_0(model);
    model_out_0.meshes_[0].transform_.SetIdentity();
    model_out_0.meshes_[0].com_ = VECTOR3(0,0,0);
    model_out_0.GenerateBoundingBox();
    model_out_0.meshes_[0].generateBoundingBox();
    std::vector<Triangle3r> pTriangles = model_out_0.GenTriangleVector();
    CSubDivRessources myRessources_dm(1,4,0,model_out_0.GetBox(),&pTriangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);

    CBoundingVolumeTree3<AABB3r,Real,CTraits,CSubdivisionCreator> bvh;
    bvh.InitTree(&subdivider_dm);

    for(int i=0;i<map_->dim_[0]*map_->dim_[0]*map_->dim_[0];i++)
    {
      VECTOR3 vQuery=map_->vertexCoords_[i];

      CDistanceFuncGridModel<Real> distFunc;
      //if(distFunc.BruteForceInnerPointsStatic(m_Model,vQuery)==1)
      bool inside = false;
      if (distFunc.PointInside(bvh.GetChild(0), vQuery) == 1)
        inside = true;

      if(isInBody(vQuery))
      {
        map_->stateFBM_[i]=1;    
      }
      else
      {
        map_->stateFBM_[i]=0;          
      }

      CDistanceMeshPoint<Real> distMeshPoint(&bvh,vQuery);
      map_->distance_[i] =  distMeshPoint.ComputeDistance();

      if(map_->stateFBM_[i])
        map_->distance_[i]*=-1.0;

      map_->normals_[i] = vQuery - distMeshPoint.m_Res.m_vClosestPoint;

      map_->contactPoints_[i] = distMeshPoint.m_Res.m_vClosestPoint;    

      if(i%1000==0)
      {
        std::cout<<"Progress: "<<i<<"/"<<map_->dim_[0]*map_->dim_[0]*map_->dim_[0]<<std::endl;        
      }

    }

    //transfer_distancemap(this, map_);
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

  void RigidBody::applyForces(const VECTOR3 &force, const VECTOR3 &torque){

    MATRIX3X3 mInvInertiaTensor = getWorldTransformedInvTensor();

    //update velocity of the compound
    velocity_ += force;

    //and the angular velocity
    setAngVel(getAngVel() + mInvInertiaTensor * torque);
  }

}
