#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <common.h>
#include <3dmodel.h>
#include <intersectorray3tri3.h>
#include <perftimer.h>

namespace i3d {

  class GPUTest : public Application {

  public:

    GPUTest() : Application() {

    }

    void init(std::string fileName)
    {

      xmin_ = -2.5f;
      ymin_ = -2.5f;
      zmin_ = -4.5f;
      xmax_ = 2.5f;
      ymax_ = 2.5f;
      zmax_ = 1.5f;

      size_t pos = fileName.find(".");

      std::string ending = fileName.substr(pos);

      std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
      if (ending == ".txt")
      {

        Reader myReader;
        //Get the name of the mesh file from the
        //configuration data file.
        myReader.readParameters(fileName, this->dataFileParams_);

      }//end if
      else if (ending == ".xml")
      {

        FileParserXML myReader;

        //Get the name of the mesh file from the
        //configuration data file.
        myReader.parseDataXML(this->dataFileParams_, fileName);

      }//end if
      else
      {
        std::cerr << "Invalid data file ending: " << ending << std::endl;
        exit(1);
      }//end else

      std::string meshFile("meshes/sb.tri");
      hasMeshFile_ = 0;

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(meshFile.c_str());
      }
      else
      {
        if (dataFileParams_.hasExtents_)
        {
          grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
            dataFileParams_.extents_[4], dataFileParams_.extents_[1],
            dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
        }
        else
          grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
      }

      //initialize rigid body parameters and
      //placement in the domain
      configureRigidBodies();

      configureBoundary();

      //assign the rigid body ids
      for (int j = 0; j<myWorld_.rigidBodies_.size(); j++)
        myWorld_.rigidBodies_[j]->iID_ = j;

      configureTimeDiscretization();

      //link the boundary to the world
      myWorld_.setBoundary(&myBoundary_);

      //set the time control
      myWorld_.setTimeControl(&myTimeControl_);

      //set the gravity
      myWorld_.setGravity(dataFileParams_.gravity_);

      //Set the collision epsilon
      myPipeline_.setEPS(0.02);

      //initialize the collision pipeline 
      myPipeline_.init(&myWorld_, dataFileParams_.solverType_, dataFileParams_.maxIterations_, dataFileParams_.pipelineIterations_);

      //set the broad phase to simple spatialhashing
      myPipeline_.setBroadPhaseHSpatialHash();

      if (dataFileParams_.solverType_ == 2)
      {
        //set which type of rigid motion we are dealing with
        myMotion_ = new MotionIntegratorSI(&myWorld_);
      }
      else
      {
        //set which type of rigid motion we are dealing with
        myMotion_ = new RigidBodyMotion(&myWorld_);
      }

      //set the integrator in the pipeline
      myPipeline_.integrator_ = myMotion_;

      myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

      myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

      myPipeline_.response_->m_pGraph = myPipeline_.graph_;

    }

    void run() {
    
      RigidBody *body;
      if (!myWorld_.rigidBodies_.empty())
      {
        body = myWorld_.rigidBodies_.front();
      }
      else
      {
        exit(1);
      }

      CMeshObject<Real> *meshObject = dynamic_cast< CMeshObject<Real> *>(body->shape_);
      C3DModel *model = &meshObject->m_Model;

      grid_.initStdMesh();

      for (int i = 0; i<dataFileParams_.nTimesteps_; i++)
      {
        grid_.refine();

        std::cout << "Generating Grid level" << i + 1 << std::endl;
        std::cout << "---------------------" << std::endl;
        std::cout << "NVT=" << grid_.nvt_ << " NEL=" << grid_.nel_ << std::endl;
        grid_.initStdMesh();
      }

      my_cuda_func(model, grid_);

      CMeshObject<Real> *object = dynamic_cast< CMeshObject<Real> *>(body->shape_);
      object->m_BVH.GenTreeStatistics();
      std::vector<CBoundingVolumeNode3<AABB3r, Real, CTraits> *> leaves =  object->m_BVH.getLeaves();
      printf("Number of leaves = %i\n", leaves.size());


      AABB3f *boxes = new AABB3f[leaves.size()];
      std::list<int> *triangleIdx = new std::list<int>[leaves.size()];
      int *sizes = new int[leaves.size()];
      int i = 0;
      for (auto &l : leaves)
      {
        Vector3<float> v0(l->m_BV.vertices_[0].x, l->m_BV.vertices_[0].y, l->m_BV.vertices_[0].z);
        Vector3<float> v1(l->m_BV.vertices_[1].x, l->m_BV.vertices_[1].y, l->m_BV.vertices_[1].z);
        boxes[i].setBox(v0, v1);
        for (auto &t : l->m_Traits.m_vTriangles)
        {
          triangleIdx[i].push_back(t.idx_);
        }
        sizes[i] = l->m_Traits.m_vTriangles.size();

        i++;
      }

      allocateNodes( triangleIdx, boxes, sizes, leaves.size());

      VECTOR3 vQuery(-1.0,0.25,0.030625);
      int nIntersections = 0;
      CUnstrGrid::VertexIter ive;
      int ivt = 65618;
      CPerfTimer timer;
      timer.Start();
      //for (int i = 0; i < grid_.nvt_; i++)
      ////for (int i = ivt; i <= ivt; i++)
      //{
      //  int id = i;
      //  VECTOR3 vQuery = grid_.vertexCoords_[i];
      //  //printf("Point [%f,%f,%f] hit with triangle\n", vQuery.x, vQuery.y, vQuery.z);

      //  if (!model->GetBox().isPointInside(vQuery))
      //  {
      //    continue;
      //  }

      //  nIntersections = 0;
      //  //int id = ive.GetPos();
      //  //VECTOR3 vQuery((*ive).x, (*ive).y, (*ive).z);
      //  for (int j = 0; j < model->m_vMeshes[0].m_iNumFaces; j++)
      //  {
      //    Triangle3<Real> tri(model->m_vMeshes[0].m_pVertices[model->m_vMeshes[0].m_pFaces[j][0]],
      //      model->m_vMeshes[0].m_pVertices[model->m_vMeshes[0].m_pFaces[j][1]],
      //      model->m_vMeshes[0].m_pVertices[model->m_vMeshes[0].m_pFaces[j][2]]);

      //    //determine ray direction
      //    Vector3<Real> dir(0.0, 0.0, 1.0);/// = vQuery - pNode->m_BV.GetCenter();

      //    Ray3<Real> ray(vQuery, dir);
      //    CIntersectorRay3Tri3<Real> intersector(ray, tri);
      //    //test for intersection//
      //    if (intersector.Intersection())
      //    {
      //      //printf("Intersection with triangle: %i \n", j);
      //      nIntersections++;
      //    }

      //  }
      //  if (nIntersections % 2 != 0)
      //  {
      //    grid_.m_myTraits[id].iTag = 1;
      //  }
      //  else
      //  {
      //    grid_.m_myTraits[id].iTag = 0;
      //  }

      //}
      //double dt_cpu = timer.GetTime();
      //printf("CPU time: %3.8f [ms]\n", dt_cpu);

      all_points_test(grid_);

      all_points_dist(grid_);

      writeOutput(0);
      writeOutput(1);
      cleanGPU();

    }

  };

}

using namespace i3d;



int main()
{

  GPUTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
