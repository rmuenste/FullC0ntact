#ifndef COLLIDERMESHMESH_CUH_A8YRKLEG
#define COLLIDERMESHMESH_CUH_A8YRKLEG

#include <contact.h>
#include <rigidbody.h>
#include <fcdefines.hpp>
#include <collidermeshmesh.h>
#include <meshobject.h>
#include <distancemeshmesh.h>
#include <distanceaabbaabb.h>
#include <cuda_runtime.h>
#include <difi.cuh>

namespace i3d {

  template <>
  class ColliderMeshMesh<gpu> : public Collider
  {
  public:

    Vector3<float> *vertexCoords_;
    Vector3<float> *contactPoints_;

    float *distance_;

    int size_;

    ColliderMeshMesh(){};

    virtual ~ColliderMeshMesh()
    {
      //cudaFree(vertexCoords_);
    };

    void transferData()
    {

      CMeshObject<Real> *pObject0 = dynamic_cast<CMeshObject<Real>* >(body0_->shape_);
      CMeshObject<Real> *pObject1 = dynamic_cast<CMeshObject<Real>* >(body1_->shape_);

      DistanceMap<Real> *map0 = body0_->map_;
      CBoundingVolumeTree3<AABB3<Real>, Real, CTraits, CSubdivisionCreator> *pBVH = &pObject1->m_BVH;
      CBoundingVolumeTree3<AABB3<Real>, Real, CTraits, CSubdivisionCreator> *pBVH0 = &pObject0->m_BVH;

      //get all the triangles contained in the root node
      Real mindist = CMath<Real>::MAXREAL;
      Vec3 cp0(0, 0, 0);
      Vec3 cp_pre(0, 0, 0);
      Vec3 cp_dm(0, 0, 0);

      //Check distance between root nodes
      CBoundingVolumeNode3<AABB3<Real>, Real, CTraits> *pNode = pBVH->GetChild(0);
      CBoundingVolumeNode3<AABB3<Real>, Real, CTraits> *pNode0 = pBVH0->GetChild(0);

      AABB3r a0 = pNode0->m_BV;
      AABB3r a1 = pNode->m_BV;

      a0.center_ = body0_->com_;
      a1.center_ = body1_->com_;

      std::vector<vector3> vertices;
      for (int k(0); k < pObject1->m_Model.meshes_[0].vertices_.Size(); ++k)
      {
        Vec3 &v0 = pObject1->m_Model.meshes_[0].vertices_[k];
        vector3 v(v0.x, v0.y, v0.z);
        vertices.push_back(v);
      }

      //vector3 *dev_vertexCoords;
      //vector3 *dev_normals;
      //vector3 *dev_contactPoints;
      //float   *dev_distance;

      size_ = vertices.size();
      int size = size_;

      cudaMalloc((void**)&vertexCoords_, size * sizeof(vector3));
      cudaCheckErrors("Allocate vertices distancemap");

      cudaMemcpy(vertexCoords_, vertices.data(), size * sizeof(vector3), cudaMemcpyHostToDevice);
      cudaCheckErrors("copy vertices distance");

      cudaMalloc((void**)&distance_, size * sizeof(float));
      cudaCheckErrors("Allocate distance array");

      cudaMalloc((void**)&contactPoints_, size * sizeof(vector3));
      cudaCheckErrors("Allocate contact points");

    }

    /**
    * @see CCollider::Collide
    *
    */
    void collide(std::vector<Contact> &vContacts)
    {

      if (body0_->map_gpu_ == nullptr || body1_->map_gpu_ == nullptr)
      {
        std::cerr << " Distance map uninitialized... exiting... " << std::endl;
        std::cerr << "> Aborting simulation: " << __FILE__ << "line: " << __LINE__ << std::endl;
        std::exit(EXIT_FAILURE);
      }

      //Get the transformation of world space into the local space of body0
      Transformationr World2Model = body0_->getTransformation();
      MATRIX3X3 Model2World = World2Model.getMatrix();
      World2Model.Transpose();

      //Get the transformation from local space to world space for body1
      Transformationr t1 = body1_->getTransformation();
      MATRIX3X3 m2w1 = t1.getMatrix();


      vector3 v0(World2Model.getOrigin().x, World2Model.getOrigin().y, World2Model.getOrigin().z);
      vector3 v1(t1.getOrigin().x, t1.getOrigin().y, t1.getOrigin().z);

      Mat3f m0;
      Mat3f m2w0;
      Mat3f m1;
      for (int j(0); j < 9; ++j)
      {
        m0.m_dEntries[j] = static_cast<float>(World2Model.getMatrix().m_dEntries[j]);
        m2w0.m_dEntries[j] = static_cast<float>(Model2World.m_dEntries[j]);
        m1.m_dEntries[j] = static_cast<float>(t1.getMatrix().m_dEntries[j]);
      }

      TransInfo info;
      info.w2m0 = m0;
      info.m2w0 = m2w0;
      info.m2w1 = m1;
      info.origin1 = v1;
      info.origin0 = v0;

      eval_distmap(body0_->map_gpu_, vertexCoords_, contactPoints_, distance_, size_, info);

      for (int i = 0; i < size_; ++i)
      {
        if (distance_[i] < 0.0015)
        {
          Vec3 c0 = (Model2World * cp_dm) + World2Model.getOrigin();
          Vec3 c1 = (Model2World * cp0) + World2Model.getOrigin();

          //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
          Contact contact;
          contact.m_dDistance = mindist;
          contact.m_vPosition0 = contactPoints_[i];
          contact.m_vPosition1 = contact.m_vPosition0;

//          contact.m_vNormal = c0 - c1;
//          contact.m_vNormal.Normalize();

          contact.m_pBody0 = body0_;
          contact.m_pBody1 = body1_;
          contact.id0 = contact.m_pBody0->iID_;
          contact.id1 = contact.m_pBody1->iID_;

          contact.m_iState = CollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }//end if(relVel < 0.0)                   
      } 

      std::cerr << "> MeshMesh GPU collider " << std::endl;
      std::cerr << "> Aborting simulation: " << __FILE__ << " line: " << __LINE__ << std::endl;
      std::exit(EXIT_FAILURE);

    }

  private:
    /* data */
  };

}


#endif /* end of include guard: COLLIDERMESHMESH_CUH_A8YRKLEG */
