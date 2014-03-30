#include "basicapplication.h"
#include <reader.h>
#include <particlefactory.h>
#include <motionintegratorsi.h>
#include <iostream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>

namespace i3d {

Application::Application() : hasMeshFile_(0)
{

}

void Application::init()
{

  Reader myReader;

  xmin_ = -2.5f;
  ymin_ = -2.5f;
  zmin_ = -4.5f;
  xmax_ = 2.5f;
  ymax_ = 2.5f;
  zmax_ = 1.5f;

  //Get the name of the mesh file from the
  //configuration data file.

  myReader.readParameters("start/data.TXT", this->dataFileParams_);

  if (hasMeshFile_)
  {
    std::string fileName;
    grid_.initMeshFromFile(fileName.c_str());
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

  myPipeline_.response_->m_pGraph = myPipeline_.graph_;

}

void Application::configureBoundary()
{
  //initialize the box shaped boundary
  myWorld_.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld_.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_ = 0;
  body->volume_ = 0;
  body->invMass_ = 0;
  body->angle_ = VECTOR3(0, 0, 0);
  body->setAngVel(VECTOR3(0, 0, 0));
  body->velocity_ = VECTOR3(0, 0, 0);
  body->shapeId_ = RigidBody::BOUNDARYBOX;
  BoundaryBoxr *box = new BoundaryBoxr();
  box->boundingBox_.init(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
  box->calcValues();
  box->setBoundaryType(BoundaryBoxr::BOXBDRY);
  body->com_ = box->boundingBox_.getCenter();
  body->shape_ = box;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

void Application::configureTimeDiscretization()
{

  myTimeControl_.SetDeltaT(dataFileParams_.timeStep_);
  myTimeControl_.SetTime(0.0);
  myTimeControl_.SetCautiousTimeStep(0.005);
  myTimeControl_.SetPreferredTimeStep(0.005);
  myTimeControl_.SetReducedTimeStep(0.0001);
  myTimeControl_.SetTimeStep(0);

}

void Application::configureRigidBodies()
{

  ParticleFactory factory(myWorld_, dataFileParams_);

}

void Application::writeOutput(int out)
{
  std::ostringstream sName, sNameParticles, sContacts;
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
  sContacts << "output/contacts.vtk." << std::setfill('0') << std::setw(5) << iTimestep;
  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());
  writer.WriteParticleFile(myWorld_.rigidBodies_, sParticleFile.c_str());
  RigidBodyIO rbwriter;
  rbwriter.write(myWorld_, sParticle.c_str());

  if (out == 0)
  {
    std::ostringstream sNameGrid;
    std::string sGrid("output/grid.vtk");
    sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
    sGrid.append(sNameGrid.str());
    writer.WriteUnstr(grid_, sGrid.c_str());
  }

}

void Application::run()
{
  unsigned nOut=0;
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
    writeOutput(nOut);
    std::cout << "Finished writing vtk." << std::endl;
    nOut++;
    myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
  }//end for
}

Application::~Application()
{

  for (auto &i : myWorld_.rigidBodies_)
  {
    RigidBody *body = i;
    delete body;
  }

}

}
