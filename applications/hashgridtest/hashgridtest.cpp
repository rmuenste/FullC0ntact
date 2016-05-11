#include <iostream>
#include <application.h>
#include <iomanip>
#include <sstream>
#include <vtkwriter.h>
#include <broadphase.h>
#include <broadphasestrategy.h>

namespace i3d {

  class HashGridTest : public Application {

  public:

    HashGridTest() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

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
        myPipeline_.strategy_->implicitGrid_->convertToUnstructuredGrid(ugrid);
        writer.WriteUnstr(ugrid, "output/broadphase.vtk");
      }
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
        writeOutput(nOut,false,false);
        std::cout << "Finished writing vtk." << std::endl;
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}

using namespace i3d;



int main()
{

  HashGridTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
