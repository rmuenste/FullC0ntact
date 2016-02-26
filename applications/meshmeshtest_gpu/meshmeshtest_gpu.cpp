#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <iostream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <difi.cuh>

namespace i3d {

  class MeshMeshTestGPU : public Application {

    public:

      UniformGrid<Real,ElementCell,VertexTraits<Real>> uniGrid_;

      MeshMeshTestGPU() : Application() {

      }

      ~MeshMeshTestGPU() {};

      void init(std::string fileName) {

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

        if (hasMeshFile_)
        {
          std::string fileName;
          grid_.initMeshFromFile(fileName.c_str());
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
        for (int j = 0; j < myWorld_.rigidBodies_.size(); j++)
        {
          myWorld_.rigidBodies_[j]->iID_ = j;
        }

        //Distance map initialization
        std::set<std::string> fileNames;

        for (auto &body : myWorld_.rigidBodies_)
        {

          if (body->shapeId_ != RigidBody::MESH)
            continue;

          CMeshObjectr *meshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
          std::string objName = meshObject->GetFileName();
          fileNames.insert(objName);
        }

        int iHandle=0;
        for (auto const &myName : fileNames)
        {
          bool created = false;
          for (auto &body : myWorld_.rigidBodies_)
          {
            if (body->shapeId_ != RigidBody::MESH)
              continue;

            CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);

            //pMeshObject->m_BVH.GenTreeStatistics();

            std::string objName = pMeshObject->GetFileName();
            if (objName == myName)
            {
              if (created)
              {
                //if map created -> add reference
                body->map_ = myWorld_.maps_.back();
              }
              else
              {
                //if map not created -> create and add reference
                body->buildDistanceMap();
                myWorld_.maps_.push_back(body->map_);
                created = true;
                CVtkWriter writer;
                std::string n = myName;
                const size_t last = n.find_last_of("\\/");
                if(std::string::npos != last)
                {
                  n.erase(0,last);
                }
                const size_t period = n.rfind(".");
                if(std::string::npos != period)
                {
                  n.erase(period);
                }
                n.append(".ps");
                std::string dir("output/");
                dir.append(n);
//                writer.writePostScriptTree(pMeshObject->m_BVH,dir.c_str());
              }
            }
          }
        }

        std::cout << myWorld_.maps_.size() << std::endl;
        allocate_distancemaps(myWorld_.rigidBodies_, myWorld_.maps_);
        exit(0);

        Real cells_x = 2.0*grid_.getAABB().extents_[0]/32.0;

        uniGrid_.initGrid(grid_.getAABB(), cells_x);
        uniGrid_.outputInfo();

        transfer_uniformgrid(&uniGrid_);
        
        configureTimeDiscretization();

        //link the boundary to the world
        myWorld_.setBoundary(&myBoundary_);

        //set the time control
        myWorld_.setTimeControl(&myTimeControl_);

        //set the gravity
        myWorld_.setGravity(dataFileParams_.gravity_);

        //set air friction
        myWorld_.setAirFriction(dataFileParams_.airFriction_);

        //Set the collision epsilon
        myPipeline_.setEPS(0.02);

        //initialize the collision pipeline
        myPipeline_.init(&myWorld_, dataFileParams_.solverType_, dataFileParams_.maxIterations_, dataFileParams_.pipelineIterations_);

        //set the broad phase to simple spatialhashing
        myPipeline_.setBroadPhaseHSpatialHash();

        //set which type of rigid motion we are dealing with
        myMotion_ = new RigidBodyMotion(&myWorld_);

        //set the integrator in the pipeline
        myPipeline_.integrator_ = myMotion_;

        myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

        myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

        myPipeline_.response_->m_pGraph = myPipeline_.graph_;

        myWorld_.graph_ = myPipeline_.graph_;

       }

      void writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
      {
        std::ostringstream sName, sNameParticles, sphereFile;
        std::string sModel("output/model.vtk");
        std::string sParticleFile("output/particle.vtk");
        std::string sParticle("solution/particles.i3d");
        CVtkWriter writer;
        int iTimestep = out;
        sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sNameParticles << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sModel.append(sName.str());
        sParticleFile.append(sName.str());
        sParticle.append(sNameParticles.str());
        sphereFile << "output/spheres.vtk." << std::setfill('0') << std::setw(5) << iTimestep;
        //Write the grid to a file and measure the time
        writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());
        //writer.WriteParticleFile(myWorld_.rigidBodies_, sParticleFile.c_str());
        writer.WriteUniformGrid2(uniGrid_,"output/unigrid.vtk");

        if(writeRBSpheres)
        {
          writer.WriteSpheresMesh(myWorld_.rigidBodies_, sphereFile.str().c_str());
        }

        if(writeRBCom)
        {
          std::ostringstream coms;
          coms << "output/com_data.vtk" << "." << std::setfill('0') << std::setw(5) << iTimestep;
          writer.WriteRigidBodyCom(myWorld_.rigidBodies_, coms.str().c_str());
        }

        if (out == 0 || out ==1)
        {
          std::ostringstream sNameGrid;
          std::string sGrid("output/grid.vtk");
          sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
          sGrid.append(sNameGrid.str());
          writer.WriteUnstr(grid_, sGrid.c_str());

          CUnstrGridr ugrid;
          for (auto &body : myWorld_.rigidBodies_)
          {
          
            if (body->shapeId_ != RigidBody::MESH)
              continue;
          
            body->map_->convertToUnstructuredGrid(ugrid);
            writer.WriteUnstr(ugrid, "output/DistanceMap1.vtk");
            break;
          
          }
        }
      }

      void run() {

        unsigned nOut = 0;

        query_uniformgrid(myWorld_.rigidBodies_.front(),uniGrid_);
        
        //start the main simulation loop

//        for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
//        {
//          Real simTime = myTimeControl_.GetTime();
//          Real energy0 = myWorld_.getTotalEnergy();
//          std::cout << "------------------------------------------------------------------------" << std::endl;
//          std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
//            << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
//          std::cout << "Energy: " << energy0 << std::endl;
//          std::cout << "------------------------------------------------------------------------" << std::endl;
//          std::cout << std::endl;
//          myPipeline_.startPipeline();
//          Real energy1 = myWorld_.getTotalEnergy();
//          std::cout << "Energy after collision: " << energy1 << std::endl;
//          std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
//          std::cout << "Timestep finished... writing vtk." << std::endl;
//          //if(nOut%10==0)
          writeOutput(nOut,false,false);
//          std::cout << "Finished writing vtk." << std::endl;
//          nOut++;
//          myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
//        }//end for

      }

  };

}

int main()
{
  using namespace i3d;

  MeshMeshTestGPU myApp;

  myApp.init(std::string("start/sampleRigidBody.xml"));

  myApp.run();

  return 0;
}
