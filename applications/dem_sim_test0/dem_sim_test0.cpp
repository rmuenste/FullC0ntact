#include <iostream>
#include <application.h>

namespace i3d {

  class DemSimTest0 : public Application {

  public:

    DemSimTest0() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      unsigned nOut = 0;
      myWorld_.extGraph_ = true;
      Application::writeOutput(5000,true,false);
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
        int itime=myWorld_.timeControl_->m_iTimeStep;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        std::cout << "Size World: "<<myWorld_.rigidBodies_.size()<< std::endl;

        //if(nOut%10==0)
          Application::writeOutput(nOut,true,false);
        
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

  DemSimTest0 myApp;

  myApp.init("start/rigidbody.xml");

  myApp.run();

  return 0;

}
