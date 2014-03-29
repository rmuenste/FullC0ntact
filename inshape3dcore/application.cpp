#include "application.h"
#include <reader.h>
#include <particlefactory.h>

namespace i3d {

Application::Application() : hasMeshFile_(0)
{

  Reader myReader;
  
  myReader.readParameters("start/data.TXT", this->dataFileParams);

  if (hasMeshFile_)
  {
    std::string fileName;
    grid_.initMeshFromFile(fileName.c_str()); 
  }

}

void Application::configureTimeDiscretization()
{

  myTimeControl.SetDeltaT(dataFileParams.timeStep_);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

}

void Application::configureRigidBodies()
{
  ParticleFactory factory;
}

Application::~Application()
{

}

}
