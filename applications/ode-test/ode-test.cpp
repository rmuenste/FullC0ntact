#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <json.hpp>

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;

static dBodyID cylbody;
static dGeomID cylgeom;

static dBodyID sphbody;
static dGeomID sphgeom;

static dJointGroupID contactgroup;

static bool show_contacts = true;

double simTime = 0.0;
double dt = 0.01;

const double CYLRADIUS = 0.6;
const double CYLLENGTH = 2.0;
const double SPHERERADIUS = 0.5;

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif

namespace i3d {


  class BodyODE
  {
  public:
    dGeomID _geomId;
    dBodyID _bodyId;

  };

  std::vector<BodyODE> bodies_;

  class DuckPond : public Application {

  public:

    SoftBody<Real, ParamLine<Real>[2]> bull;

    DuckPond() : Application() {

    }

    void init(std::string fileName) {

      using namespace std;

      xmin_ = -2.5f;
      ymin_ = -2.5f;
      zmin_ = -4.5f;
      xmax_ = 2.5f;
      ymax_ = 2.5f;
      zmax_ = 1.5f;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

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

      cout<<"startType = "<<dataFileParams_.startType_<<endl; 
      cout<<"solution = "<<dataFileParams_.solutionFile_<<endl; 
      cout<<"nBodies = "<<dataFileParams_.bodies_<<endl;  
      cout<<"bodyInit = "<<dataFileParams_.bodyInit_<<endl; 
      cout<<"bodyFile = "<<dataFileParams_.bodyConfigurationFile_<<endl; 
      cout<<"defaultDensity = "<<dataFileParams_.defaultDensity_<<endl; 
      cout<<"defaultRadius = "<<dataFileParams_.defaultRadius_<<endl; 
      cout<<"gravity = "<<dataFileParams_.gravity_;  
      cout<<"totalTimesteps = "<<dataFileParams_.nTimesteps_<<endl;
      cout<<"lcpSolverIterations = "<<dataFileParams_.maxIterations_<<endl;
      cout<<"collPipelineIterations = "<<dataFileParams_.pipelineIterations_<<endl;
      
      if(dataFileParams_.hasExtents_)
      {
        cout << "domain extents = " << dataFileParams_.extents_[0] << " " << dataFileParams_.extents_[1] << " " << dataFileParams_.extents_[2] << " "
                                    << dataFileParams_.extents_[3] << " " << dataFileParams_.extents_[4] << " " << dataFileParams_.extents_[5] << endl;
      }
      
      if(dataFileParams_.bodies_ > 0)
      {
        cout<<"type = "<< dataFileParams_.rigidBodies_[0].shapeId_ <<endl; 
        cout<<"position = "<< dataFileParams_.rigidBodies_[0].com_ <<endl; 
        cout<<"velocity = "<< dataFileParams_.rigidBodies_[0].velocity_ <<endl; 
        cout<<"density = "<< dataFileParams_.rigidBodies_[0].density_ <<endl;
        cout<<"meshfile = "<< dataFileParams_.rigidBodies_[0].fileName_ <<endl;       
      }

      bull.init();
      std::ostringstream name;
      std::ostringstream name2;
      int step = 0;
      name << "output/line." << std::setfill('0') << std::setw(5) << step << ".vtk";
      name2 << "output/head." << std::setfill('0') << std::setw(5) << step << ".vtk";
      CVtkWriter writer;
      writer.WriteParamLine(bull.geom_[0], name.str().c_str());
      writer.WriteParamLine(bull.geom_[1], name2.str().c_str());

    }

    void run()
    {
      const double pi = 3.1415926535897;

      int istep = 0;

      Real t  = 0.0;
      Real dt = 0.001;

      CVtkWriter writer;

      for(istep=1; t < 2500.0; istep++)
      {
        t+=dt;
        bull.internalForce(t); 
        bull.integrate();
        if(istep%1000==0)
        {
          std::ostringstream name2;
          std::ostringstream name3;
          name2 << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
          name3 << "output/head." << std::setfill('0') << std::setw(5) << istep << ".vtk";
          writer.WriteParamLine(bull.geom_[0], name2.str().c_str());
          writer.WriteParamLine(bull.geom_[1], name3.str().c_str());
        }
      }
    }
  };
}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
      fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
    for (int i=0; i<n; i++) 
    {
      contact[i].surface.mode = 0;
      contact[i].surface.mu = 50.0; // was: dInfinity
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      if (show_contacts) 
      {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal ss[3] = {0.12,0.12,0.12};

        dReal *pos  = contact[i].geom.pos;
        dReal depth = contact[i].geom.depth;
        dReal *norm = contact[i].geom.normal;
        dReal endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};

      }
    }
  }
}

using namespace i3d;

// start simulation - set viewpoint
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {-8,-9,3};
  static float hpr[3] = {45.0000f,-27.5000f,0.0000f};
}

// called when a key pressed
static void command (int cmd)
{
  switch (cmd) 
  {
    case ' ':
      break;
  }
}

// simulation loop
void simulationLoop (int istep)
{
  dSpaceCollide (space,0,&nearCallback);

  dWorldQuickStep (world, 0.01); // 100 Hz

  dJointGroupEmpty (contactgroup);

//  for (auto &i : bodies_)
//  {

//  }

  const dReal *SPos = dBodyGetPosition(bodies_[0]._bodyId);
  const dReal *SRot = dBodyGetRotation(bodies_[0]._bodyId);
  float spos[3] = {SPos[0], SPos[1], SPos[2]};
  float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };

  std::vector<Model3D> meshes;
  CVtkWriter writer;

  CTriangulator<Real, Sphere<Real> > triangulator;

  Spherer Sphere(Vec3(0,0,0),SPHERERADIUS);
  
  Model3D model_out=triangulator.Triangulate(&Sphere);

  double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ SRot[4], SRot[5], SRot[6], /* */ SRot[8], SRot[9], SRot[10] };

  MATRIX3X3 transform(entries);
  model_out.meshes_[0].transform_ = transform;
  model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
  model_out.meshes_[0].TransformModelWorld();

  meshes.push_back(model_out);

  std::ostringstream sName;

  sName << "output/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";

  writer.WriteModels(meshes, sName.str().c_str());

//  printf("Time: %f |Step: %d |Position cylinder: [%f %f %f]\n",simTime, istep, cpos[0], cpos[1], cpos[2]);
  printf("Time: %f |Step: %d |Position sphere: [%f %f %f]\n",simTime, istep, spos[0], spos[1], spos[2]);
  simTime += dt;
}

using json = nlohmann::json;

int main()
{

  //----------------------------------------------

  dMass m;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-9.8);
  dWorldSetQuickStepNumIterations (world, 32);

  dCreatePlane (space,0,0,1, 0.0);

  dQuaternion q;

  //----------------------------------------------

  //           Set up cylinder body
  //----------------------------------------------
//  cylbody = dBodyCreate (world);

//  dQFromAxisAndAngle (q,1,0,0, M_PI * -0.77);

//  dBodySetQuaternion (cylbody,q);

//  // set mass for a cylinder of density 1.0
//  dMassSetCylinder (&m,1.0,3,CYLRADIUS,CYLLENGTH);
//  dBodySetMass (cylbody,&m);

//  cylgeom = dCreateCylinder(0, CYLRADIUS, CYLLENGTH);
//  dGeomSetBody (cylgeom,cylbody);
//  dBodySetPosition (cylbody, 0, 0, 3);
//  dSpaceAdd (space, cylgeom);

  //----------------------------------------------

  std::ifstream i("cube.json");
  json j;
  i >> j;

  for (int i(0); i < j.size(); ++i)
  {

    Vec3 p(j[i]["Pos"][0], j[i]["Pos"][1], j[i]["Pos"][2]);
    Vec3 d(j[i]["Dim"][0], j[i]["Dim"][1], j[i]["Dim"][2]);

    BodyODE b;
    sphbody = dBodyCreate (world);
    b._bodyId = sphbody;

    dMassSetSphere (&m,1,d.y);
    dBodySetMass (b._bodyId,&m);

    sphgeom = dCreateSphere(0, d.y);
    b._geomId = sphgeom;

    dGeomSetBody (b._geomId,b._bodyId);

    dBodySetPosition (b._bodyId, p.x, p.y, p.z);
    dSpaceAdd (space, b._geomId);

    bodies_.push_back(b);

  }


  //----------------------------------------------

  // run simulation
  for (int i(0); i <= 300; ++i)
  {
    simulationLoop(i);
  }

  dJointGroupEmpty (contactgroup);
  dJointGroupDestroy (contactgroup);

  dGeomDestroy(sphgeom);

  //dGeomDestroy (cylgeom);

  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();

  //----------------------------------------------
  
//  DuckPond myApp;

//  myApp.init("start/sampleRigidBody.xml");

//  myApp.run();

  return 0;
}
