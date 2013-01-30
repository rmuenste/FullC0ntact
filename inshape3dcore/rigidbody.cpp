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

namespace i3d {

CRigidBody::CRigidBody() : m_iCollisionState(0)
{
  m_pShape = NULL;
  m_dDampening = 1.0;
  m_bAffectedByGravity = true;
  m_iElementsPrev      = 0;
  m_dFriction          = 0.0;
  m_bRemote            = false;
}

CRigidBody::~CRigidBody()
{
	if(m_pShape != NULL)
	{
		delete m_pShape;
		m_pShape = NULL;
	}
}

CRigidBody::CRigidBody(VECTOR3 vVelocity, Real dDensity, Real dVolume, Real dMass, VECTOR3 vAngle, int iShape) : m_iCollisionState(0)
{
	this->m_vVelocity=vVelocity;
	this->m_dDensity = dDensity;
	this->m_dVolume  = dVolume;
	this->m_dInvMass = dMass;
	this->m_vAngle   = vAngle;
	this->m_iShape   = iShape;
  m_bAffectedByGravity = true;

  m_dDampening     = 1.0;
  m_bRemote            = false;
}

CRigidBody::CRigidBody(CShaper *pShape, int iShape)
{
	m_iShape     = iShape;
	m_pShape     = pShape;
  m_bAffectedByGravity = true;
  m_bRemote            = false;  
  
  m_dDampening = 1.0;
}

CRigidBody::CRigidBody(Particle& p)
{
  m_vVelocity   = VECTOR3(p.vx,p.vy,p.vz);
  m_dDensity    = p.density;
  m_Restitution = p.restitution;
  m_vAngle      = VECTOR3(p.ax,p.ay,p.az);
  m_vAngVel     = VECTOR3(p.avx,p.avy,p.avz);
  m_iShape      = p.ishape;
  m_iID         = p.origid;
  m_iRemoteID   = p.origid;
  m_vCOM        = VECTOR3(p.x,p.y,p.z);
  m_vForce      = VECTOR3(p.mx,p.my,p.mz);
  m_vTorque     = VECTOR3(p.tx,p.ty,p.tz);
  m_vQ          = CQuaternionr(p.qx,p.qy,p.qz,p.qw);
  m_iElementsPrev   = 0;

  m_matTransform = m_vQ.GetMatrix();
  
  Real entries[9] = {p.a1,p.a2,p.a3,p.a4,p.a5,p.a6,p.a7,p.a8,p.a9};

  memcpy(m_InvInertiaTensor.m_dEntries,entries,9*sizeof(Real));

  m_dDampening = 1.0;

  if(p.igrav == 0)
  {
    m_bAffectedByGravity = false;
    if(m_iShape == CRigidBody::SPHERE)
    {
      m_pShape = new CSpherer(VECTOR3(0,0,0),p.exx);
      m_dVolume   = p.volume;
      m_dInvMass  = 0.0;
    }
    else
    {
      std::cerr<<"Unknown shape identifier: "<<m_iShape<<". Please enter a valid shape identifier."<<std::endl;
      exit(0);
    }
    m_InvInertiaTensor.SetZero();
  }
  else if(p.igrav == 1)
  {
    m_bAffectedByGravity = true;
    if(m_iShape == CRigidBody::SPHERE)
    {
      m_pShape = new CSpherer(VECTOR3(0,0,0),p.exx);
      m_dVolume   = p.volume;
      m_dInvMass  = p.invmass;
    }
    else
    {
      std::cerr<<"Unknown shape identifier: "<<m_iShape<<". Please enter a valid shape identifier."<<std::endl;
      exit(0);
    }
  }
  else
  {
    std::cerr<<"Invalid value: isAffectedByGravity: "<<m_iShape<<std::endl;
    exit(0);
  }

}

CRigidBody::CRigidBody(sRigidBody *pBody)
{
  m_vVelocity = pBody->m_vVelocity;
  m_dDensity  = pBody->m_dDensity;
  m_Restitution = pBody->m_Restitution;
  m_vAngle    = pBody->m_vAngle;
  m_vAngVel   = pBody->m_vAngVel;
  m_iShape    = pBody->m_iShape;
  m_iID       = pBody->m_iID;
  m_vCOM      = pBody->m_vCOM;
  m_vForce    = pBody->m_vForce;
  m_vTorque   = pBody->m_vTorque;
  m_vQ        = pBody->m_vQ;
  m_iElementsPrev   = 0;
  m_bRemote            = false;
  if(pBody->m_bMatrixAvailable)
  {
    m_matTransform = m_vQ.GetMatrix();
  }
  else
  {
    SetOrientation(m_vAngle);
    m_matTransform = m_vQ.GetMatrix();
  }

  memcpy(m_InvInertiaTensor.m_dEntries,pBody->m_dTensor,9*sizeof(Real));

  m_dDampening = 1.0;

  if(pBody->m_iAffectedByGravity == 0)
  {
    m_bAffectedByGravity = false;
    if(m_iShape == CRigidBody::SPHERE)
    {
      m_pShape = new CSpherer(VECTOR3(0,0,0),pBody->m_Extends[0]);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;
    }
    else if(m_iShape == CRigidBody::BOUNDARYBOX)
    {
      //Implement the adding of a boundary box
      m_pShape = new CBoundaryBoxr(m_vCOM,pBody->m_Extends);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;
    }
    else if(m_iShape == CRigidBody::BOX)
    {
      m_pShape = new COBB3r(VECTOR3(0,0,0), pBody->m_vUVW, pBody->m_Extends);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;
    }
    else if(m_iShape == CRigidBody::CYLINDER)
    {
      m_pShape = new CCylinderr(VECTOR3(0,0,0),VECTOR3(0,0,1),pBody->m_Extends[0],pBody->m_Extends[2]);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;
    }
    else if(m_iShape == CRigidBody::MESH)
    {
      m_pShape = new CMeshObject<Real>();
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(m_pShape);
      pMeshObject->SetFileName(pBody->m_strFileName);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;

      CGenericLoader Loader;
      Loader.ReadModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

      pMeshObject->m_Model.GenerateBoundingBox();
      for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
      {
        pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
      }

      C3DModel model_out(pMeshObject->m_Model);
      model_out.GenerateBoundingBox();
      for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
      {
        model_out.m_vMeshes[i].m_matTransform = m_matTransform;
        model_out.m_vMeshes[i].m_vOrigin = m_vCOM;
        model_out.m_vMeshes[i].TransformModelWorld();
        model_out.m_vMeshes[i].GenerateBoundingBox();
      }

      std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
      CSubDivRessources myRessources(1,7,0,model_out.GetBox(),&pTriangles);
      CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
      pMeshObject->m_BVH.InitTree(&subdivider);
      
    }	  
    else
    {
      std::cerr<<"Unknown shape identifier: "<<m_iShape<<". Please enter a valid shape identifier."<<std::endl;
      exit(0);
    }
    m_InvInertiaTensor.SetZero();
  }
  else if(pBody->m_iAffectedByGravity == 1)
  {
    m_bAffectedByGravity = true;
    if(m_iShape == CRigidBody::SPHERE)
    {
      m_pShape = new CSpherer(VECTOR3(0,0,0),pBody->m_Extends[0]);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 1.0/(m_dDensity * m_dVolume);
    }
    else if(m_iShape == CRigidBody::BOUNDARYBOX)
    {
      //Implement the adding of a boundary box
      m_pShape = new CBoundaryBoxr(m_vCOM,pBody->m_Extends);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 0.0;
    }
    else if(m_iShape == CRigidBody::BOX)
    {
      m_pShape = new COBB3r(VECTOR3(0,0,0), pBody->m_vUVW, pBody->m_Extends);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 1.0/(m_dDensity * m_dVolume);
    }
    else if(m_iShape == CRigidBody::CYLINDER)
    {
      m_pShape = new CCylinderr(VECTOR3(0,0,0),VECTOR3(0,0,1),pBody->m_Extends[0],pBody->m_Extends[2]);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 1.0/(m_dDensity * m_dVolume);
    }
    else if(m_iShape == CRigidBody::MESH)
    {
      m_pShape = new CMeshObject<Real>();
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(m_pShape);
      pMeshObject->SetFileName(pBody->m_strFileName);
      m_dVolume   = m_pShape->Volume();
      m_dInvMass  = 1.0/(m_dDensity * m_dVolume);

      CGenericLoader Loader;
      Loader.ReadModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());

      pMeshObject->m_Model.GenerateBoundingBox();
      for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
      {
        pMeshObject->m_Model.m_vMeshes[i].GenerateBoundingBox();
      }

      C3DModel model_out(pMeshObject->m_Model);
      model_out.GenerateBoundingBox();
      for(int i=0;i< pMeshObject->m_Model.m_vMeshes.size();i++)
      {
        model_out.m_vMeshes[i].m_matTransform = m_matTransform;
        model_out.m_vMeshes[i].m_vOrigin = m_vCOM;
        model_out.m_vMeshes[i].TransformModelWorld();
        model_out.m_vMeshes[i].GenerateBoundingBox();
      }

      std::vector<CTriangle3r> pTriangles = model_out.GenTriangleVector();
      CSubDivRessources myRessources(1,5,0,model_out.GetBox(),&pTriangles);
      CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
      pMeshObject->m_BVH.InitTree(&subdivider);
      
    }
    else
    {
      std::cerr<<"Unknown shape identifier: "<<m_iShape<<". Please enter a valid shape identifier."<<std::endl;
      exit(0);
    }
    //generate the inverted inertia tensor
    GenerateInvInertiaTensor();
  }
  else
  {
    std::cerr<<"Invalid value: isAffectedByGravity: "<<m_iShape<<std::endl;
    exit(0);
  }
}

CRigidBody::CRigidBody(const CRigidBody& copy)
{

	m_vCOM             = copy.m_vCOM;
	m_vVelocity        = copy.m_vVelocity;
	m_dDensity         = copy.m_dDensity;
	m_dVolume          = copy.m_dVolume;
	m_dInvMass         = copy.m_dInvMass;
	m_pShape           = copy.m_pShape;
	m_vQ               = copy.m_vQ;
	m_InvInertiaTensor = copy.m_InvInertiaTensor;
	m_vAngVel          = copy.m_vAngVel;
	m_vAngle           = copy.m_vAngle;
	m_iShape           = copy.m_iShape;
	m_iID              = copy.m_iID;
	m_iCollisionState  = copy.m_iCollisionState;
	m_Restitution      = copy.m_Restitution;
	m_vForceResting    = copy.m_vForceResting;
	m_vForce           = copy.m_vForce;
	m_vTorque          = copy.m_vTorque;
  m_dDampening       = copy.m_dDampening;
  m_bAffectedByGravity = copy.m_bAffectedByGravity;
  m_matTransform     = copy.m_matTransform;
  m_iGroup           = copy.m_iGroup;
  m_iElementsPrev    = 0;
  m_bRemote            = copy.m_bRemote; 

}

void CRigidBody::TranslateTo(const VECTOR3 &vPos)
{
	m_vCOM = vPos;
}

void CRigidBody::GenerateInvInertiaTensor()
{
  CAABB3r box;
	if(m_iShape == CRigidBody::SPHERE)
	{
		//calculate the inertia tensor
		//Get the inertia tensor
		box = m_pShape->GetAABB();
		Real rad2 = box.m_Extends[0]*box.m_Extends[0];
		Real dnum  = 5.0f*m_dInvMass; 
		Real denom = 2.f*rad2;
		m_InvInertiaTensor = MATRIX3X3(dnum/denom, 0, 0, 0, dnum/denom, 0, 0, 0, dnum/denom);
	}
	else if(m_iShape == CRigidBody::BOX)
	{
		box = m_pShape->GetAABB();
		Real dwmass = 12.0 * m_dInvMass; 
		Real w2 = 4.0 * box.m_Extends[0]*box.m_Extends[0];
		Real h2 = 4.0 * box.m_Extends[1]*box.m_Extends[1];
		Real d2 = 4.0 * box.m_Extends[2]*box.m_Extends[2];
		m_InvInertiaTensor = MATRIX3X3(dwmass/(h2+d2), 0, 0, 0, dwmass/(w2+d2), 0, 0, 0, dwmass/(w2+h2));
	}
  else if(m_iShape == CRigidBody::CYLINDER)
	{
		box = m_pShape->GetAABB();
    Real sqrrad = box.m_Extends[0]*box.m_Extends[0];
    Real sqrh   = box.m_Extends[2]*box.m_Extends[2] * 4.0;
    Real dmass = 12*m_dInvMass;
    Real m3rh = dmass * (1.0/(3.0*sqrrad+sqrh));
		m_InvInertiaTensor = MATRIX3X3(m3rh, 0, 0, 0, m3rh, 0, 0, 0, 2.0*m_dInvMass*(1.0/sqrrad));
	}
	else if(m_iShape == CRigidBody::BOUNDARYBOX)
	{
		m_InvInertiaTensor.SetZero();
	}
  else if(m_iShape == CRigidBody::MESH)
	{
    //default torus:
    //I_zz=(rad_xz^2*3/4 + rad_xy^2)*m
    //I_xx=I_yy=(5*rad_xz^2+4*rad_xy^2)*m
    //Volume = 2*pi^2*rad_xz^2*rad_xy
    Real xx = 3.872174e-6;
    Real yy = 4.502750e-6;
    Real zz = 5.111549e-6;
    m_InvInertiaTensor = MATRIX3X3(1.0/xx, 0, 0, 0, 1.0/yy, 0, 0, 0, 1.0/zz);
    //Real rad_xy = 0.1;
    //Real rad_xz = 0.01;
    //Real rad_xy2 = 0.1*0.1;
    //Real rad_xz2 = 0.01*0.01;
    //Real dmass = 1.0/m_dInvMass;
    //m_InvInertiaTensor = MATRIX3X3((5.0*rad_xz2+4.0*rad_xy2)*dmass, 0, 0, 0, (5.0*rad_xz2+4.0*rad_xy2)*dmass, 0, 0, 0, (rad_xz2*3.0/4.0 + rad_xy2)*dmass);
	}
	
}

Real CRigidBody::GetEnergy()
{
  Real velEnergy = 0.0;
  if(m_bAffectedByGravity)
  {
	  velEnergy = 1.0/m_dInvMass * (m_vVelocity * m_vVelocity);
	  //Real angEnergy = GetInertiaTensor * (m_vAngVel * m_vAngVel);
  }
	return velEnergy;
}

MATRIX3X3 CRigidBody::GetWorldTransformedInvTensor()
{
  //transform the inertia tensor to world space
  MATRIX3X3 mMatWorldTensor = m_matTransform * m_InvInertiaTensor * m_matTransform.GetTransposedMatrix();
  return mMatWorldTensor;
}

const CShaper& CRigidBody::GetOriginalShape() const
{
  if(m_iShape == CRigidBody::BOX)
  {
    const COBB3r &pBox = dynamic_cast<const COBB3r&>(*m_pShape);
    return pBox;
  }
  if(m_iShape == CRigidBody::SPHERE)
  {
    const CSpherer &pSphere = dynamic_cast<const CSpherer&>(*m_pShape);
    return pSphere;
  }
  if(m_iShape == CRigidBody::CYLINDER)
  {
    const CCylinderr &pCylinder = dynamic_cast<const CCylinderr&>(*m_pShape);
    return pCylinder;
  }
  else
  {
    const CBoundaryBoxr &pBoundary = dynamic_cast<const CBoundaryBoxr&>(*m_pShape);
    return pBoundary;
  }
}

CShaper* CRigidBody::GetWorldTransformedShape()
{
  
  CShaper *pShape;
  
  if(m_iShape == CRigidBody::SPHERE)
  {
    pShape = new CSpherer(m_vCOM,m_pShape->GetAABB().m_Extends[0]);
  }
  else if(m_iShape == CRigidBody::BOX)
  {
    //pBox = CoordTransform().transform(dynamic_cast<COBB3r *>(pBody0->m_pShape),pBody->CoM,pBody->GetOrientMatrix());
    COBB3r *pBox = dynamic_cast<COBB3r *>(m_pShape);
    
    //get the transformed vectors
    VECTOR3 vUVW[3];
    VECTOR3 vWorld;
    
    //transform
    for(int i=0;i<3;i++)
    {
     vWorld = pBox->m_vUVW[i];
     vWorld = m_matTransform*vWorld;
     vUVW[i] = vWorld;
    }

    //the world tranformed box
    pShape = new COBB3r(m_vCOM,vUVW,pBox->m_Extents);
  }
  else if(m_iShape == CRigidBody::CYLINDER)
  {
    CCylinderr *pCylinder = dynamic_cast<CCylinderr *>(m_pShape);
    
    VECTOR3 u = m_matTransform*pCylinder->GetU();

    //the world tranformed cylinder
    pShape = new CCylinderr(m_vCOM,u,pCylinder->GetRadius(),pCylinder->GetHalfLength());
  }
  else if(m_iShape == CRigidBody::MESH)
  {

		CMeshObject<Real> *pMeshObject = dynamic_cast<CMeshObject<Real>*>(m_pShape);

	  CMeshObject<Real> *pMesh= new CMeshObject<Real>();
	  pMesh->m_Model=pMeshObject->m_Model;
    pMesh->m_Model.m_vMeshes[0].m_matTransform =m_matTransform;
		pMesh->m_Model.m_vMeshes[0].m_vOrigin =m_vCOM;
		pMesh->m_Model.m_vMeshes[0].TransformModelWorld();
    pMesh->m_Model.m_vMeshes[0].GenerateBoundingBox();
    pMesh->m_Model.m_bdBox = pMesh->m_Model.m_vMeshes[0].m_bdBox;
    //the world tranformed mesh
    pShape = pMesh;
    return pShape;
  }
  else if(m_iShape == CRigidBody::BOUNDARYBOX)
  {
    pShape =  new CBoundaryBoxr();
  }
  
  return pShape;
  
}

CShaper* CRigidBody::GetWorldTransformedShapeNext(Real dT)
{
  
  CShaper *pShape;
  
  if(m_iShape == CRigidBody::SPHERE)
  {
    VECTOR3 newCoM = m_vCOM + m_vVelocity * dT;
    pShape = new CSpherer(newCoM,m_pShape->GetAABB().m_Extends[0]);
  }
  else if(m_iShape == CRigidBody::BOX)
  {
    COBB3r *pBox = dynamic_cast<COBB3r *>(m_pShape);
    MATRIX3X3 mrotMat;
    VECTOR3 vWorld;

    //update the position
    VECTOR3 newpos = m_vCOM + m_vVelocity * dT;

    //update angle
    VECTOR3 newangle = m_vAngle + m_vAngVel * dT;

    //get the rotation matrix
    mrotMat.MatrixFromAngles(newangle);
    VECTOR3 vUVW[3];
    
    //transform
    for(int i=0;i<3;i++)
    {
     vWorld = pBox->m_vUVW[i];
     vWorld = mrotMat*vWorld;
     vUVW[i] = vWorld;
    }

    //the world tranformed box
    pShape = new COBB3r(newpos,vUVW,pBox->m_Extents);
  }
  else if(m_iShape == CRigidBody::BOUNDARYBOX)
  {
    pShape =  new CBoundaryBoxr();
  }
  
  return pShape;
  
}

CTransformr CRigidBody::GetTransformation() const
{
  return CTransformr(m_matTransform,m_vCOM);
}

VECTOR3 CRigidBody::GetAxisAngle()
{
  VECTOR3 vAxis;
  //compute sine and cosine values
  double c1 = cos(m_vAngle.x/2);//heading
  double s1 = sin(m_vAngle.x/2);//heading
  double c2 = cos(m_vAngle.y/2);//attitude
  double s2 = sin(m_vAngle.y/2);//attitude
  double c3 = cos(m_vAngle.z/2);//bank
  double s3 = sin(m_vAngle.z/2);//bank
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

void CRigidBody::UpdateAngVel(const VECTOR3 &delta) 
{
    m_vAngVel += delta;
}

void CRigidBody::ApplyImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
{
    MATRIX3X3 mInvInertiaTensor = GetWorldTransformedInvTensor();

    m_vVelocity += linearUpdate;

    m_vAngVel   += mInvInertiaTensor * (VECTOR3::Cross(relPos,impulse));

}

void CRigidBody::ApplyBiasImpulse(const VECTOR3 &relPos, const VECTOR3 &impulse, const VECTOR3 &linearUpdate)
{

  MATRIX3X3 mInvInertiaTensor = GetWorldTransformedInvTensor();

  m_vBiasVelocity += linearUpdate;

  m_vBiasAngVel  += mInvInertiaTensor * (VECTOR3::Cross(relPos,impulse));

}

bool CRigidBody::IsInBody(const VECTOR3 &vQuery) const
{
	  //for meshes we calculate in world coordinates
    if(m_iShape == CRigidBody::MESH)
      return (m_pShape->PointInside(vQuery));
		else
		{
			//for other shapes we transform to local coordinates
			VECTOR3 vLocal = vQuery - m_vCOM;
      MATRIX3X3 trans = m_matTransform;
      trans.TransposeMatrix();
			vLocal = trans * vLocal ;
			return (m_pShape->PointInside(vLocal));
		}
}

MATRIX3X3 CRigidBody::GetTransformationMatrix() const
{
  return m_matTransform;
}

void CRigidBody::SetTransformationMatrix(const MATRIX3X3& mat)
{
  m_matTransform = mat;
}

void CRigidBody::RemoveEdge(CCollisionInfo *pInfo)
{
  std::list<CCollisionInfo*>::iterator i = m_pEdges.begin();
  while(i!=m_pEdges.end())
  {
    CCollisionInfo *collinfo = (*i);
    if(collinfo->iID1 == pInfo->iID1 && collinfo->iID2 ==pInfo->iID2)
    {
      i=m_pEdges.erase(i);
      break;
    }//end if
    i++;
  }//end while

}//end Remove edge


}


