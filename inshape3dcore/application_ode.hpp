#ifndef APPLICATION_ODE_HPP_VCEM0QJS
#define APPLICATION_ODE_HPP_VCEM0QJS

#include <worldparameters.h>
#include <unstructuredgrid.h>
#include <collisionpipeline.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <world.h>
#include <string>
#include <rigidbodyio.h>

#include <reader.h>
#include <particlefactory.h>
#include <iostream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <json.hpp>
#include <collisionpipeline_backend_ode.hpp>

namespace i3d
{

  template <>
  class Application<BackEnd::backendODE>
  {
  public:

    World myWorld_;

    UnstructuredGrid<Real, DTraits> grid_;

    WorldParameters dataFileParams_;

    int hasMeshFile_;

    CollisionPipeline<executionDefault, BackEnd::backendODE> myPipeline_;

    TimeControl myTimeControl_;

    BoundaryBoxr myBoundary_;

    RigidBodyMotion *myMotion_;

    double xmin_;
    double ymin_;
    double zmin_;
    double xmax_;
    double ymax_;
    double zmax_;

    Application();

    virtual ~Application();

    virtual void init(std::string fileName = std::string("start/data.TXT"));

    virtual void run()=0;

  protected:

    void configureTimeDiscretization();

    void configureRigidBodies();

    void configureBoundary();

    void configureCylinderBoundary();

    void configureHollowCylinderBoundary();

    void writeOutput(int out, bool writeRBCom = false, bool writeRBSpheres = false);
  
  private:
    /* data */
  };

Application<BackEnd::backendODE>::Application() : hasMeshFile_(0), myMotion_(nullptr)
{
  xmin_ = -2.5f;
  ymin_ = -2.5f;
  zmin_ = -4.5f;
  xmax_ = 2.5f;
  ymax_ = 2.5f;
  zmax_ = 1.5f;
}

void Application<BackEnd::backendODE>::init(std::string fileName)
{

//  BodyStorage storage;
//  storage.affectedByGravity_ = 0;
//
//  storage.shapeId_ = RigidBody::MESH;
//
//  std::string Name = "arch.obj";
//  Name.copy(storage.fileName_, Name.size() + 1);
//
//  storage.fileName_[Name.size()] = '\0'; 
//
//  RigidBody* pointerBody = new RigidBody(&storage);

//  RigidBody::RigidBody(BodyStorage *pBody, bool sub)
//  {
//    velocity_    = pBody->velocity_;
//    density_     = pBody->density_;
//    restitution_ = pBody->restitution_;
//    angle_       = pBody->angle_;
//    angVel_      = pBody->angVel_;
//    shapeId_     = pBody->shapeId_;
//    iID_         = pBody->id_;
//    com_         = pBody->com_;
//    force_       = pBody->force_;
//    torque_      = pBody->torque_;
//    quat_        = pBody->quat_;
//
//    if(pBody->matrixAvailable_)
//    {
//      matTransform_ = quat_.GetMatrix();
//      transform_.setMatrix(matTransform_);
//      transform_.setOrigin(com_);
//    }
//    else
//    {
//      setOrientation(angle_);
//      matTransform_ = quat_.GetMatrix();
//      transform_.setMatrix(matTransform_);
//      transform_.setOrigin(com_);
//    }
//
//    std::memcpy(invInertiaTensor_.m_dEntries, pBody->tensor_, sizeof pBody->tensor_);
//
//    {
//
//      affectedByGravity_ = false;
//
//      // Generate a mesh object
//      shape_ = new MeshObject<Real>(pBody->useMeshFiles_, pBody->meshFiles_);
//
//      MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(shape_);
//
//      pMeshObject->setFileName(pBody->fileName_);
//
//      volume_   = shape_->getVolume();
//      invMass_  = 0.0;
//
//      GenericLoader Loader;
//      Loader.readModelFromFile(&pMeshObject->getModel(),pMeshObject->getFileName().c_str());
//
//      pMeshObject->getModel().generateBoundingBox();
//
//      Transformationr tf = getTransformation();
//      pMeshObject->initTree(tf);
//
//    }

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
  for (unsigned j = 0; j < myWorld_.rigidBodies_.size(); j++)
  {
    myWorld_.rigidBodies_[j]->iID_ = j;
    if (myWorld_.rigidBodies_[j]->shapeId_ == RigidBody::COMPOUND)
    {
      CompoundBody *c = dynamic_cast<CompoundBody*>(myWorld_.rigidBodies_[j]);
      for (unsigned i = 0; i < c->rigidBodies_.size(); i++)
      {
        c->rigidBodies_[i]->iID_ = j;
      }
    }
  }
  configureTimeDiscretization();

  //link the boundary to the world
  myWorld_.setBoundary(&myBoundary_);

  //set the time control
  myWorld_.setTimeControl(&myTimeControl_);

  //set the gravity
  myWorld_.setGravity(dataFileParams_.gravity_);

  //set air friction
  myWorld_.setAirFriction(dataFileParams_.airFriction_);

  myPipeline_.world_ = &myWorld_;

  //Set the collision epsilon
  myPipeline_.setEPS(0.02);

  //set which type of rigid motion we are dealing with
  myMotion_ = new RigidBodyMotion(&myWorld_);

  myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

  myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

  myWorld_.graph_ = myPipeline_.graph_;

}

void Application<BackEnd::backendODE>::configureBoundary()
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
  box->boundingBox_.init(dataFileParams_.extents_[0], dataFileParams_.extents_[2], dataFileParams_.extents_[4],
                         dataFileParams_.extents_[1], dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
  box->calcValues();
  body->com_ = box->boundingBox_.getCenter();
  body->shape_ = box;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

void Application<BackEnd::backendODE>::configureCylinderBoundary()
{
  //initialize the cylinder shaped boundary
  myWorld_.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld_.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_ = 0;
  body->volume_ = 0;
  body->invMass_ = 0;
  body->angle_ = VECTOR3(0, 0, 0);
  body->setAngVel(VECTOR3(0, 0, 0));
  body->velocity_ = VECTOR3(0, 0, 0);
  body->shapeId_ = RigidBody::CYLINDERBDRY;

  BoundaryCylr *cyl = new BoundaryCylr();
  cyl->boundingBox_.init(dataFileParams_.extents_[0], dataFileParams_.extents_[2], dataFileParams_.extents_[4],
                         dataFileParams_.extents_[1], dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
  cyl->cylinder_ = Cylinderr(VECTOR3(0.0, 0.0, 0.0), VECTOR3(0.0, 0.0, 1.0), cyl->boundingBox_.extents_[0], cyl->boundingBox_.extents_[2]);
  body->com_ = cyl->boundingBox_.getCenter();
  body->shape_ = cyl;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

void Application<BackEnd::backendODE>::configureHollowCylinderBoundary()
{
  //initialize the cylinder shaped boundary
  myWorld_.rigidBodies_.push_back(new RigidBody());
  RigidBody *body = myWorld_.rigidBodies_.back();
  body->affectedByGravity_ = false;
  body->density_ = 0;
  body->volume_ = 0;
  body->invMass_ = 0;
  body->angle_ = VECTOR3(0, 0, 0);
  body->setAngVel(VECTOR3(0, 0, 3.14));
  body->velocity_ = VECTOR3(0, 0, 0);
  body->shapeId_ = RigidBody::HOLLOWCYLINDER;

  BoundaryCylr *cyl = new BoundaryCylr();
  cyl->boundingBox_.init(dataFileParams_.extents_[0], dataFileParams_.extents_[2], dataFileParams_.extents_[4],
                         dataFileParams_.extents_[1], dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
  cyl->cylinder_ = Cylinderr(VECTOR3(0.0, 0.0, 0.0), VECTOR3(0.0, 0.0, 1.0), cyl->boundingBox_.extents_[0], cyl->boundingBox_.extents_[2]);
  body->com_ = cyl->boundingBox_.getCenter();
  body->shape_ = cyl;
  body->invInertiaTensor_.SetZero();
  body->restitution_ = 0.0;
  body->setOrientation(body->angle_);
}

void Application<BackEnd::backendODE>::configureTimeDiscretization()
{

  myTimeControl_.SetDeltaT(dataFileParams_.timeStep_);
  myTimeControl_.SetTime(0.0);
  myTimeControl_.SetCautiousTimeStep(0.005);
  myTimeControl_.SetPreferredTimeStep(0.005);
  myTimeControl_.SetReducedTimeStep(0.0001);
  myTimeControl_.SetTimeStep(0);

}

void Application<BackEnd::backendODE>::configureRigidBodies()
{

  ParticleFactory factory(myWorld_, dataFileParams_);
  myWorld_.solverType_ = dataFileParams_.solverType_;

}

void Application<BackEnd::backendODE>::writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
{

  using namespace std;

  std::ostringstream sName;

  sName << "output/model." << std::setw(5) << std::setfill('0') << out << ".vtk";
  std::string strFileName(sName.str());

  CVtkWriter writer;

  writer.WriteODE2VTK(myWorld_.bodies_, strFileName.c_str());

}

Application<BackEnd::backendODE>::~Application()
{

  for (auto &i : myWorld_.rigidBodies_)
  {
    RigidBody *body = i;
    delete body;
  }

  if(myMotion_ != nullptr)
  {
    delete myMotion_;
    myMotion_ = nullptr;
  }

}


  
} /* i3d */ 

#endif /* end of include guard: APPLICATION_ODE_HPP_VCEM0QJS */
