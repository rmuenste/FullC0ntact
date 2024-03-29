#include <reader.h>

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
                    myParameters.rigidBodies_[0].nSoftBodyParticles_,
                    myParameters.rigidBodies_[0].ks_, myParameters.rigidBodies_[0].kb_,
                    myParameters.rigidBodies_[0].kd_, 0.01);

    if(myWorld.parInfo_.getId() == 1)
    {
       std::cout << "Particle position: " << myParameters.rigidBodies_[0].ks_ << " " << myParameters.rigidBodies_[0].kb_ << std::endl;
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
