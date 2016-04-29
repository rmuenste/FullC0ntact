#ifndef COLLIDERMESHMESH_CUH_A8YRKLEG
#define COLLIDERMESHMESH_CUH_A8YRKLEG

#include <contact.h>
#include <rigidbody.h>
#include <fcdefines.hpp>
#include <collidermeshmesh.h>
#include <collisioninfo.h>
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
    Vector3<float> *normals_;
    

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

      //std::cout <<"> cps_gpu: " << vertices[0] << std::endl;

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

      cudaMalloc((void**)&normals_, size * sizeof(vector3));
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

//      std::cout <<"> GPU world2model matrix: " << m0 << std::endl;
//      std::cout <<"> GPU model2world matrix: " << m2w0 << std::endl;
//      std::cout <<"> GPU origin0: " << v0 << std::endl;
//      std::cout <<"> GPU origin1: " << v1 << std::endl;

      eval_distmap(body0_->map_gpu_, vertexCoords_, contactPoints_, normals_, distance_, size_, info);

      std::vector<float> distance(size_);
      std::vector<vector3> normals(size_);
      std::vector<vector3> cps(size_);

      cudaMemcpy(distance.data(), distance_, sizeof(float)*size_, cudaMemcpyDeviceToHost);
      cudaCheckErrors("Copying distance to host");

      cudaMemcpy(cps.data(), contactPoints_,  sizeof(vector3)*size_, cudaMemcpyDeviceToHost);
      cudaCheckErrors("Copying cps to host");

      cudaMemcpy(normals.data(), normals_,  sizeof(vector3)*size_, cudaMemcpyDeviceToHost);
      cudaCheckErrors("Copying normals to host");
     
      int contacts(0);
      for (int i = 0; i < size_; ++i)
      {
        if (distance[i] < 0.0015)
        {
          contacts++;
          Vec3 c0(cps[i].x, cps[i].y, cps[i].z);

          Contact contact;
          contact.m_dDistance = 0.0;
          contact.m_vPosition0 = c0;
          contact.m_vPosition1 = contact.m_vPosition0;

          Vec3 n(normals[i].x, normals[i].y, normals[i].z);
          contact.m_vNormal = n;

          contact.m_pBody0 = body0_;
          contact.m_pBody1 = body1_;
          contact.id0 = contact.m_pBody0->iID_;
          contact.id1 = contact.m_pBody1->iID_;

          contact.m_iState = CollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }//end if(relVel < 0.0)                   
      } 
      std::cout << "> Number of contacts point for gpu: " << vContacts.size() << std::endl;
    }

  private:
    /* data */
  };

}


#endif /* end of include guard: COLLIDERMESHMESH_CUH_A8YRKLEG */
