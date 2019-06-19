#include <reader.h>

#ifdef WITH_ODE
void init_ode_simulation()
{

  using json = nlohmann::json;

  ParticleFactory myFactory;

  dMass m;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);

  Vec3 g_eff = myParameters.gravity_;

  Real d_eff = myParameters.defaultDensity_ - myParameters.densityMedium_;

  Real r = 0.0075;

  Real v = 4.0/3.0 * CMath<Real>::SYS_PI * std::pow(r,3.0);   

  Real mass = myParameters.defaultDensity_ * v;

  Real m_inv = 1.0/mass;

  g_eff = v * d_eff * m_inv * g_eff; 

  std::cout << "G' = " << v << std::endl;
  //std::cout << "G' = " << g_eff;

  dWorldSetGravity(world,g_eff.x,
                         g_eff.y,
                         g_eff.z);

  dWorldSetQuickStepNumIterations (world, 32);

  std::ifstream i("cube.json");
  json j;
  i >> j;
  std::cout << j << std::endl;

  for (int i(0); i < j.size(); ++i)
  {

    Vec3 p(j[i]["Pos"][0], j[i]["Pos"][1], j[i]["Pos"][2]);
    Vec3 d(j[i]["Dim"][0], j[i]["Dim"][1], j[i]["Dim"][2]);

    BodyODE b;

    if (j[i]["Type"] == "Sphere")
    {
      sphbody = dBodyCreate (world);

      b._bodyId = sphbody;
      b._type = std::string("Sphere");

      double rho = myParameters.defaultDensity_;

      dMassSetSphere (&m,rho ,0.5 * d.y);
      dBodySetMass (b._bodyId,&m);

      dMass pmass;
      dBodyGetMass (b._bodyId,&pmass);

      sphgeom = dCreateSphere(0, 0.5 * d.y);
      b._geomId = sphgeom;

      dGeomSetBody (b._geomId,b._bodyId);

      dBodySetPosition (b._bodyId, p.x, p.y, p.z);
      dSpaceAdd (space, b._geomId);

      BodyStorage body;
      body.shapeId_ = RigidBody::SPHERE;

      body.com_.x = p.x; 
      body.com_.y = p.y;
      body.com_.z = p.z;

      body.velocity_ = Vec3(0,0,0);

      body.angVel_ = Vec3(0,0,0);
      body.angle_  = Vec3(0,0,0);
      body.force_  = Vec3(0,0,0);
      body.torque_ = Vec3(0,0,0);

      body.extents_[0] = 0.5 * d.x;
      body.extents_[1] = 0.5 * d.y;
      body.extents_[2] = 0.5 * d.z;

      body.uvw_[0] = Vec3(1,0,0);
      body.uvw_[1] = Vec3(0,1,0);
      body.uvw_[2] = Vec3(0,0,1);

      body.density_ = myParameters.defaultDensity_;
      body.restitution_ = 0.0;
      body.volume_ = 0.0;

      memset(body.tensor_, 0, 9*sizeof(Real));

      b._type   = std::string("Sphere");
      b._index  = myWorld.rigidBodies_.size();

      RigidBody *pBody = new RigidBody(&body);

      pBody->odeIndex_ = myWorld.bodies_.size();

      myWorld.rigidBodies_.push_back(pBody);

      myWorld.bodies_.push_back(b);

    }
    else if (j[i]["Type"] == "Plane")
    {
      dGeomID p = dCreatePlane (space,0,0,1, 0.0);

      b._geomId = p;
      b._bodyId = dBodyID(-10);
      b._type   = std::string("Plane");
      b._index  = -1;
      myWorld.bodies_.push_back(b);
    }
    else if (j[i]["Type"] == "Cube")
    {
      b._bodyId = dBodyCreate (world);

      dMassSetBox(&m, 1.0, d.x, d.y, d.z);
      dBodySetMass (b._bodyId,&m);

      b._geomId = dCreateBox(0, d.x, d.y, d.z);

      dGeomSetBody (b._geomId,b._bodyId);

      dBodySetPosition (b._bodyId, p.x , p.y, p.z);
      dSpaceAdd (space, b._geomId);

      BodyStorage body;
      body.shapeId_ = RigidBody::BOX;

      body.com_.x = p.x; 
      body.com_.y = p.y;
      body.com_.z = p.z;

      body.velocity_ = Vec3(0,0,0);

      body.angVel_ = Vec3(0,0,0);
      body.angle_ = Vec3(0,0,0);
      body.force_ = Vec3(0,0,0);
      body.torque_ = Vec3(0,0,0);

      body.extents_[0] = d.x;
      body.extents_[1] = d.y;
      body.extents_[2] = d.z;

      body.uvw_[0] = Vec3(1,0,0);
      body.uvw_[1] = Vec3(0,1,0);
      body.uvw_[2] = Vec3(0,0,1);

      body.density_ = 1.0;
      body.restitution_ = 0.0;
      body.volume_ = 0.0;

      memset(body.tensor_, 0, 9*sizeof(Real));

      b._type  = std::string("Cube");
      b._index = myWorld.rigidBodies_.size();

      RigidBody *pBody = new RigidBody(&body);

      pBody->odeIndex_ = myWorld.bodies_.size();

      myWorld.rigidBodies_.push_back(pBody);

      myWorld.bodies_.push_back(b);

    }

  }

  //set the timestep
  myTimeControl.SetDeltaT(myParameters.timeStep_);
  myTimeControl.SetTime(0.0);
  myTimeControl.SetCautiousTimeStep(0.005);
  myTimeControl.SetPreferredTimeStep(0.005);
  myTimeControl.SetReducedTimeStep(0.0001);
  myTimeControl.SetTimeStep(0);

  //set the time control
  myWorld.setTimeControl(&myTimeControl);

  //set the gravity
  myWorld.setGravity(myParameters.gravity_);

  //Set the collision epsilon
  myPipeline.setEPS(0.02);

  myWorld.densityMedium_ = myParameters.densityMedium_;

  myWorld.liquidSolid_   = (myParameters.liquidSolid_ == 1) ? true : false;

}

extern "C" void init_fc_ode(int *iid)
{
  using namespace std;
  int iOut=0;

  Reader reader;
  std::string meshFile;

  int id = *iid;
  myWorld.parInfo_.setId(id);
  
  //read the user defined configuration file
  std::string fileName("start/sampleRigidBody.xml");
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Loading config file: " <<
    termcolor::reset << fileName  << std::endl;

  size_t pos = fileName.find(".");

  std::string ending = fileName.substr(pos);

  std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
  if (ending == ".txt")
  {

    Reader myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.readParameters(fileName, myParameters);

  }//end if
  else if (ending == ".xml")
  {

    FileParserXML myReader;

    //Get the name of the mesh file from the
    //configuration data file.
    myReader.parseDataXML(myParameters, fileName);

  }//end if
  else
  {
    std::cerr << "Invalid data file ending: " << ending << std::endl;
    std::exit(EXIT_FAILURE);
  }//end else

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.initMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.initCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  init_ode_simulation();
    
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Initialized setup: rigid body " <<
    termcolor::reset  << std::endl;

  if(myWorld.parInfo_.getId()==1)
  {
//    RigidBody *body = myWorld.rigidBodies_[0];
//    std::cout << termcolor::bold << termcolor::blue <<  "> Volume: " << body->volume_  <<
//        termcolor::reset  << std::endl;
//
//    std::cout << termcolor::bold << termcolor::blue <<  "> mass: " << 1.0/body->invMass_  <<
//        termcolor::reset  << std::endl;
  }

}

#endif

extern "C" void init_fc_rigid_body(int *iid)
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  Reader reader;
  std::string meshFile;

  xmin = -2.5f;
  ymin = -2.5f;
  zmin = -4.5f;
  xmax = 2.5f;
  ymax = 2.5f;
  zmax = 1.5f;
  int id = *iid;
  myWorld.parInfo_.setId(id);
  
  //read the user defined configuration file
  std::string fileName("start/sampleRigidBody.xml");

  if(myWorld.parInfo_.getId()==1)
  {
    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Loading config file: " <<
      termcolor::reset << fileName  << std::endl;
  }

  size_t pos = fileName.find(".");

  std::string ending = fileName.substr(pos);

  std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);

  if (ending == ".txt")
  {
    Reader myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.readParameters(fileName, myParameters);
  }//end if
  else if (ending == ".offs")
  {
    OffsReader reader;
    //Get the name of the mesh file from the
    //configuration data file.
    reader.readParameters(fileName, myParameters);
  }//end if
  else if (ending == ".xml")
  {
    FileParserXML myReader;

    //Get the name of the mesh file from the
    //configuration data file.
    myReader.parseDataXML(myParameters, fileName);
  }//end if
  else
  {
    std::cerr << "Invalid data file ending: " << ending << std::endl;
    std::exit(EXIT_FAILURE);
  }//end else

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.initMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.initCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  initsimulation();
    
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Initialized setup: rigid body " <<
    termcolor::reset  << std::endl;

#ifdef OPTIC_FORCES
  init_external_ot(); 
#endif

}

extern "C" void init_fc_cgal(int *iid)
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  Reader reader;
  std::string meshFile;

  xmin = -2.5f;
  ymin = -2.5f;
  zmin = -4.5f;
  xmax = 2.5f;
  ymax = 2.5f;
  zmax = 1.5f;
  int id = *iid;
  myWorld.parInfo_.setId(id);
  
  //read the user defined configuration file
  std::string fileName("mesh_names.offs");
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Loading config file: " <<
    termcolor::reset << fileName  << std::endl;

  size_t pos = fileName.find(".");

  std::string ending = fileName.substr(pos);

  std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
  if (ending == ".txt")
  {

    Reader myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.readParameters(fileName, myParameters);

  }//end if
  else if (ending == ".offs")
  {

    OffsReader reader;
    //Get the name of the mesh file from the
    //configuration data file.
    reader.readParameters(fileName, myParameters);

  }//end if
  else if (ending == ".xml")
  {

    FileParserXML myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.parseDataXML(myParameters, fileName);

  }//end if
  else
  {
    std::cerr << "Invalid data file ending: " << ending << std::endl;
    std::exit(EXIT_FAILURE);
  }//end else

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.initMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.initCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.startType_ == 0)
  {
    initsimulation();
  }
  else
  {
    continuesimulation();
  }
    
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Initialized setup: rigid body " <<
    termcolor::reset  << std::endl;

  if(myWorld.parInfo_.getId()==1)
  {

  }

}

extern "C" void init_fc_soft_body(int *iid)
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  Reader reader;
  std::string meshFile;

  xmin = -2.5f;
  ymin = -2.5f;
  zmin = -4.5f;
  xmax = 2.5f;
  ymax = 2.5f;
  zmax = 1.5f;
  int id = *iid;

  myWorld.parInfo_.setId(id);
  
  //read the user defined configuration file
  std::string fileName("start/sampleRigidBody.xml");
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Loading config file: " <<
    termcolor::reset << fileName  << std::endl;

  size_t pos = fileName.find(".");

  std::string ending = fileName.substr(pos);

  std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
  if (ending == ".txt")
  {

    Reader myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.readParameters(fileName, myParameters);

  }//end if
  else if (ending == ".xml")
  {

    FileParserXML myReader;

    //Get the name of the mesh file from the
    //configuration data file.
    myReader.parseDataXML(myParameters, fileName);

  }//end if
  else
  {
    std::cerr << "Invalid data file ending: " << ending << std::endl;
    std::exit(EXIT_FAILURE);
  }//end else

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.initMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.initCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.startType_ == 0)
  {
    initsimulation();
  }
  else
  {
    continuesimulation();
  }

  softBodyPointer = std::make_shared< SoftBody4<Real, ParamLine<Real>>>(
                    myParameters.rigidBodies_[0].nSoftBodyParticles_, 0.35,
                    myParameters.rigidBodies_[0].ks_, myParameters.rigidBodies_[0].kb_,
                    myParameters.rigidBodies_[0].kd_, 0.01);

//  softBodyPointer = std::make_shared< SoftBody4<Real, ParamLine<Real>>>(
//                    myParameters.rigidBodies_[0].nSoftBodyParticles_, 
//                    myParameters.rigidBodies_[0].ks_, myParameters.rigidBodies_[0].kb_,
//                    myParameters.rigidBodies_[0].kd_, 0.01);

  if(myWorld.parInfo_.getId() == 1)
  {
     std::cout << "Spring stiffness: " << myParameters.rigidBodies_[0].ks_ << " " << myParameters.rigidBodies_[0].kb_ << std::endl;
  }

  softBody_ = softBodyPointer.get();

  SpringConfiguration<Real> sc(softBody_->N_, softBody_->ks_,
                               softBody_->kd_, softBody_->a0_, softBody_->l0_);
  
  InitSpringMesh2 initSprings(sc, *softBody_); 
  initSprings.init();

  softBody_->istep_ = 0;
  istep_ = 0;
  simTime_ = 0.0;

  for(int i(0); i < softBody_->geom_.vertices_.size(); ++i)
  {
    myWorld.rigidBodies_[i]->com_      = softBody_->geom_.vertices_[i];
    myWorld.rigidBodies_[i]->velocity_ = softBody_->u_[i];
    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Particle position: " << myWorld.rigidBodies_[i]->com_;
    }
  }

  if(myWorld.parInfo_.getId() == 1)
  {
    std::cout << "Soft Body length: " << softBody_->length() << std::endl;
  }

  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> FC initialized " <<
    termcolor::reset  << std::endl;

}

void fallingparticles()
{
  using namespace std;
  int iOut=0;
  Real dTimePassed=1;
  Real energy0=0.0;
  Real energy1=0.0;
  Reader reader;
  std::string meshFile;

  xmin = -2.5f;
  ymin = -2.5f;
  zmin = -4.5f;
  xmax = 2.5f;
  ymax = 2.5f;
  zmax = 1.5f;
  
  //read the user defined configuration file
  std::string fileName("start/sampleRigidBody.xml");
  std::cout << termcolor::green << "> Loading config file: " << termcolor::reset << fileName  << std::endl;

  size_t pos = fileName.find(".");

  std::string ending = fileName.substr(pos);

  std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
  if (ending == ".txt")
  {

    Reader myReader;
    //Get the name of the mesh file from the
    //configuration data file.
    myReader.readParameters(fileName, myParameters);

  }//end if
  else if (ending == ".xml")
  {

    FileParserXML myReader;

    //Get the name of the mesh file from the
    //configuration data file.
    myReader.parseDataXML(myParameters, fileName);

  }//end if
  else
  {
    std::cerr << "Invalid data file ending: " << ending << std::endl;
    std::exit(EXIT_FAILURE);
  }//end else


  int argc=1;
  std::string s("./stdQ2P1");

  char *argv[1];
   
#ifdef FC_CUDA_SUPPORT
  char* argument = new char[s.size()+1];
  std::copy(s.begin(), s.end(), argument);
  argument[s.size()]='\0';
  argv[0] = argument;
  initGL(&argc,argv);
  cudaGLInit(argc,argv);
	
  uint gridDim = GRID_SIZE;
  gridSize.x = gridSize.y = gridSize.z = gridDim;
#endif

  //initialize the grid
  if(iReadGridFromFile == 1)
  {
    myGrid.initMeshFromFile(meshFile.c_str());
    //refine grid: Parameter iMaxRefinement
  }
  else
  {
    myGrid.initCube(xmin,ymin,zmin,xmax,ymax,zmax);
  }

  //initialize a start from zero or
  //continue a simulation
  if(myParameters.startType_ == 0)
  {
    initsimulation();
  }
  else
  {
    continuesimulation();
  }

    
}

extern "C" void initaneurysm()
{
}



