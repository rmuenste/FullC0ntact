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
//    RigidBody *body = myWorld.rigidBodies_[0];
//    std::cout << termcolor::bold << termcolor::blue <<  "> Volume: " << body->volume_  <<
//        termcolor::reset  << std::endl;
//
//    std::cout << termcolor::bold << termcolor::blue <<  "> mass: " << 1.0/body->invMass_  <<
//        termcolor::reset  << std::endl;
//
//    //check if inside, if so then leave the function
//    if(body->isInBody(Vec3(-0.0033333,0,0.1)))
//    {
//      std::cout << termcolor::bold << termcolor::blue <<  "> inside " <<
//        termcolor::reset  << std::endl;
//    }
//    else
//    {
//      std::cout << termcolor::bold << termcolor::blue <<  "> outside " <<
//        termcolor::reset  << std::endl;
//    }
  }

#ifdef OPTIC_FORCES
  init_optical_tweezers();
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

//          ks_ = 10.0;
//          kd_ = -0.2;

  SpringConfiguration<Real> sc(9, 10.0, -0.2, 0.04, 0.04);
    

  InitSpringMesh2 initSprings(sc, softBody_.springs_, softBody_.geom_,
                              softBody_.u_,softBody_.force_, softBody_.externalForce_ ); 

  //initSprings.init();
  softBody_.init2();

  softBody_.istep_ = 0;
  istep_ = 0;
  simTime_ = 0.0;

  for(int i(0); i < softBody_.geom_.vertices_.size(); ++i)
  {
    myWorld.rigidBodies_[i]->com_      = softBody_.geom_.vertices_[i];
    myWorld.rigidBodies_[i]->velocity_ = softBody_.u_[i];
    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Particle position: " << myWorld.rigidBodies_[i]->com_;
    }
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

extern "C" void initdeform()
{

  Reader reader;  
  ParticleFactory factory;

  //read the user defined configuration file
  reader.readParametersDeform(std::string("start/data.TXT"),myDeformParameters);  
  
  myWorld = factory.produceFromDeformParameters(myDeformParameters);  
  
}

extern "C" void initpointlocation()
{

  Reader reader;  

  //read the user defined configuration file
  reader.readParameters(string("start/data.TXT"),myParameters);  
  
  ParticleFactory factory;  
  
  myWorld = factory.produceFromParameters(myParameters);  
  
}

extern "C" void addbdryparam(int *iBnds, int *itype, char *name, int length)
{
  
  //null terminate string
  name[length--]='\0';
  int ilength=strlen(name);
  std::string fileName(name);
  int type = *itype;
  if(type==2)
  {
    RigidBody *param = new RigidBody();
    param->velocity_       = VECTOR3(0,0,0);
    param->density_        = 1.0;
    param->restitution_     = 0.0;
    param->angle_          = VECTOR3(0,0,0);
    param->setAngVel(VECTOR3(0,0,0));
    param->shapeId_          = RigidBody::MESH;
    param->iID_             = *iBnds;
    param->com_            = VECTOR3(0,0,0);
    param->force_          = VECTOR3(0,0,0);
    param->torque_         = VECTOR3(0,0,0);
    param->dampening_      = 1.0;  
    param->elementsPrev_   = 0;
    param->remote_         = false;
    param->setOrientation(param->angle_);
    param->affectedByGravity_ = false;
  }

    std::cout << "Error: Function addbdryparam is deprecated." << std::endl;
    std::exit(EXIT_FAILURE);
//    param->shape_ = new CMeshObject<Real>();
//    CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(param->shape_);
//    pMeshObject->SetFileName(fileName.c_str());
//    param->volume_   = param->shape_->getVolume();
//    param->invMass_  = 0.0;
//
//    GenericLoader Loader;
//    Loader.readModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());
//
//    pMeshObject->m_Model.generateBoundingBox();
//    for(int i=0;i< pMeshObject->m_Model.meshes_.size();i++)
//    {
//      pMeshObject->m_Model.meshes_[i].generateBoundingBox();
//    }
//    
//    Model3D model_out(pMeshObject->m_Model);
//    model_out.generateBoundingBox();
//    for(int i=0;i< pMeshObject->m_Model.meshes_.size();i++)
//    {
//      model_out.meshes_[i].transform_ = param->getTransformationMatrix();
//      model_out.meshes_[i].com_ = param->com_;
//      model_out.meshes_[i].TransformModelWorld();
//      model_out.meshes_[i].generateBoundingBox();
//    }
//
//    std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
//    CSubDivRessources myRessources(1,7,0,model_out.getBox(),&pTriangles);
//    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
//    pMeshObject->m_BVH.InitTree(&subdivider);      
//    param->invInertiaTensor_.SetZero();
//    
//    RigidBody *body = param;  
//    CMeshObjectr *pMeshObject2 = dynamic_cast<CMeshObjectr *>(body->shape_);
//    bdryParams.push_back(param);
//    if(myWorld.parInfo_.getId()==1)
//    {
//      printf("Boundary parameterization file %s initialized successfully with iBnds = %i.\n",fileName.c_str(),param->iID_);
//    }
//  }
//  else if(type==3)
//  {
//    RigidBody *param = new RigidBody();
//    param->velocity_       = VECTOR3(0,0,0);
//    param->density_        = 1.0;
//    param->restitution_     = 0.0;
//    param->angle_          = VECTOR3(0,0,0);
//    param->setAngVel(VECTOR3(0,0,0));
//    param->shapeId_          = RigidBody::PLINE;
//    param->iID_             = *iBnds;
//    param->com_            = VECTOR3(0,0,0);
//    param->force_          = VECTOR3(0,0,0);
//    param->torque_         = VECTOR3(0,0,0);
//    param->dampening_      = 1.0;  
//    param->elementsPrev_   = 0;
//    param->remote_         = false;
//    param->setOrientation(param->angle_);
//    param->affectedByGravity_ = false;
//
//    param->shape_ = new ParamLiner();
//    ParamLiner *line = dynamic_cast<ParamLiner *>(param->shape_);
//    SegmentListReader myReader;
//    myReader.readModelFromFile(line,fileName.c_str());      
//    bdryParams.push_back(param);
//    if(myWorld.parInfo_.getId()==1)
//    {
//      printf("Boundary parameterization file %s initialized successfully with iBnds = %i.\n",fileName.c_str(),param->iID_);
//    }
//  }
//  else
//  {
//    if(myWorld.parInfo_.getId()==1)
//    {
//      printf("Unknown boundary parameterization type %i.\n",type);    
//    }
//  }
}


extern "C" void initbdryparam()
{

  std::cout << "Error: Function initbdryparam is deprecated." << std::endl;
  std::exit(EXIT_FAILURE);

//  bdryParameterization = new RigidBody();
//  bdryParameterization->velocity_       = VECTOR3(0,0,0);
//  bdryParameterization->density_        = 1.0;
//  bdryParameterization->restitution_    = 0.0;
//  bdryParameterization->angle_          = VECTOR3(0,0,0);
//  bdryParameterization->setAngVel(VECTOR3(0,0,0));
//  bdryParameterization->shapeId_        = RigidBody::MESH;
//  bdryParameterization->iID_            = -1;
//  bdryParameterization->com_            = VECTOR3(0,0,0);
//  bdryParameterization->force_          = VECTOR3(0,0,0);
//  bdryParameterization->torque_         = VECTOR3(0,0,0);
//  bdryParameterization->dampening_      = 1.0;  
//  bdryParameterization->elementsPrev_   = 0;
//  bdryParameterization->remote_         = false;
//  bdryParameterization->setOrientation(bdryParameterization->angle_);
//  bdryParameterization->affectedByGravity_ = false;
//
//  bdryParameterization->shape_ = new CMeshObject<Real>();
//  CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(bdryParameterization->shape_);
//  std::string fileName;
//  pMeshObject->SetFileName(fileName.c_str());
//  bdryParameterization->volume_   = bdryParameterization->shape_->getVolume();
//  bdryParameterization->invMass_  = 0.0;
//
//  GenericLoader Loader;
//  Loader.readModelFromFile(&pMeshObject->m_Model,pMeshObject->GetFileName().c_str());
//
//  pMeshObject->m_Model.generateBoundingBox();
//  for(int i=0;i< pMeshObject->m_Model.meshes_.size();i++)
//  {
//    pMeshObject->m_Model.meshes_[i].generateBoundingBox();
//  }
//  
//  Model3D model_out(pMeshObject->m_Model);
//  model_out.generateBoundingBox();
//  for(int i=0;i< pMeshObject->m_Model.meshes_.size();i++)
//  {
//    model_out.meshes_[i].transform_ = bdryParameterization->getTransformationMatrix();
//    model_out.meshes_[i].com_ = bdryParameterization->com_;
//    model_out.meshes_[i].TransformModelWorld();
//    model_out.meshes_[i].generateBoundingBox();
//  }
//
//  std::vector<Triangle3r> pTriangles = model_out.genTriangleVector();
//  CSubDivRessources myRessources(1,9,0,model_out.getBox(),&pTriangles);
//  CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
//  pMeshObject->m_BVH.InitTree(&subdivider);      
//  bdryParameterization->invInertiaTensor_.SetZero();
}
