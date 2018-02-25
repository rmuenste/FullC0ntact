#include <iostream>
#include <application.h>

namespace i3d {

  class Test1 : public Application<> {

  public:

    Test1() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      unsigned nOut = 0;
      myWorld_.extGraph_ = true;
      //start the main simulation loop
      for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
      {
        Real simTime = myTimeControl_.GetTime();
        Real energy0 = myWorld_.getTotalEnergy();
#ifndef FC_SILENT
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
        std::cout << "Energy: " << energy0 << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
#endif        
        int itime=myWorld_.timeControl_->m_iTimeStep;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
#ifndef FC_SILENT
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
#endif
        //if(nOut%20==0)
        Application::writeOutput(nOut,true,true);
#ifndef FC_SILENT
        std::cout << "Finished writing vtk." << std::endl;
#endif 
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}

using namespace i3d;

int main()
{
  
  Test1 myApp;

  myApp.init("start/rigidbody.xml");

  myApp.run();

  return 0;
}
