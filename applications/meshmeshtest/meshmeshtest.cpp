#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>

namespace i3d {

  class MeshMeshTest : public Application {

  public:
    MeshMeshTest() : Application() {

    }

    ~MeshMeshTest() {};

    void init() {

      Application::init();

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


      DistanceMap<Real> *map = NULL;  
      int iHandle=0;
      for (auto const &myName : fileNames)
      {
        bool created = false;
        for (auto &body : myWorld_.rigidBodies_)
        {
          if (body->shapeId_ != RigidBody::MESH)
            continue;

          CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
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
            }
          }
        }
      }        
      std::cout<<"Number of different meshes: "<<fileNames.size()<<std::endl;  
    }

    void run() {

      unsigned nOut = 0;
      //start the main simulation loop
      for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
      {
        Real simTime = myTimeControl_.GetTime();
        Real energy0 = myWorld_.getTotalEnergy();
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
        std::cout << "Energy: " << energy0 << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        Application::writeOutput(nOut);
        std::cout << "Finished writing vtk." << std::endl;
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}


  //if(iout==0)
  //{
  //  
  //  CUnstrGridr hgrid;
  //  myWorld.rigidBodies_[0]->map_->ConvertToUnstructuredGrid(hgrid);
  //  writer.WriteUnstr(hgrid,"output/distancemap.vtk");         
  //  
  //  std::ostringstream sNameGrid;
  //  std::string sGrid("output/grid.vtk");
  //  sNameGrid<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  //  sGrid.append(sNameGrid.str());
  //  writer.WriteUnstr(myGrid,sGrid.c_str());
  //}

int main()
{
  using namespace i3d;

  MeshMeshTest myApp;
  
  myApp.init();
  
  myApp.run();

  return 0;
}
