#include <iostream>
#include <application_ode.hpp>
#include <reader.h>
#include <paramline.h>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <collisionpipeline.h>


#include <obb3.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <intersectorray3tri3.h>
#include <perftimer.h>
#include <vtkwriter.h>
#include <geom_config.hpp>
#include <distancegridcgal.hpp>
#include <laplace_alpha.hpp>
#include <meshdecimater.hpp>
#include <distancemapbuilder.hpp>
#include <uniformgrid.h>
#include <huniformgrid.h>
#include <distancepointobb3.h>
#include <distanceaabbpoint.h>
#include <ode_rb_writer.hpp>
#include <perftimer.h>
//#include <filesystem>
//namespace fs = std::filesystem;

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

using FBMApp = i3d::Application<i3d::BackEnd::backendODE>;


namespace i3d {

  Real SqDistPointAABB(Vec3 p, const AABB3r& b) {

    Real sqDist = 0.0;
    for (int i = 0; i < 3; i++) {
      Real v = p.m_dCoords[i];
      if (v < b.vertices_[0].m_dCoords[i]) sqDist += (b.vertices_[0].m_dCoords[i] - v) * (b.vertices_[0].m_dCoords[i] - v);
      if (v > b.vertices_[1].m_dCoords[i]) sqDist += (v - b.vertices_[1].m_dCoords[i]) * (v - b.vertices_[1].m_dCoords[i]);
    }

    return sqDist;

  }

 
  class FBMTest3 : public FBMApp {

  public:  
    
  UniformGridHierarchy<Real,ElementCell,BasicTraits<Real>> myUniformGrid;

  FBMTest3() : Application()
  {
        
  }
  
  virtual ~FBMTest3() {};

  void writeOutput(int out)
  {
    std::ostringstream sName, sNameParticles, sphereFile;
    std::string sModel("output/model.vtk");

    CVtkWriter writer;
    int iTimestep = out;
    sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
    sModel.append(sName.str());

    std::cout << "Writing VTK surface mesh to: " << sModel.c_str() << std::endl;
    //Write the grid to a file and measure the time
    writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str(), true);

    ODERigidBodyWriter rigidBodyWriter;
    
    int iout = 1;
//    std::string folder("_sol_rb");
//
//    if(!fs::exists(folder))
//    {
//      fs::create_directory(folder);
//    }
//
//    folder.append("/");
//    folder.append(std::to_string(iout));
//
//    if(!fs::exists(folder))
//    {
//      fs::create_directory(folder);
//    }
//
//    nlohmann::json array_explicit = nlohmann::json::array();


    std::string n("rb.json");
    rigidBodyWriter.write(myWorld_, n);

  }
  
  
  void run()
  {

    grid_.initMeshFromFile("geo090.tri");
    grid_.initStdMesh();

    for(int i = 0; i < 3; i++)
    {
      grid_.refine();
      std::cout<<"Generating Grid level"<<i+1<<std::endl;
      std::cout<<"---------------------"<<std::endl;
      std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
      grid_.initStdMesh();
    }       
    
    myUniformGrid.initGrid(grid_.getAABB(), 1);
    myUniformGrid.initGridLevel(0, 0.03);

    CPerfTimer timer0;
    timer0.Start();
    VertexIter<Real> v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      Vec3 &v = *v_it.Get();
      int idx = v_it.idx();
      for (RigidBody *body : myWorld_.rigidBodies_) {
        if (body->isInBody(v)) {
          grid_.m_myTraits[idx].iTag = 1;
        }
      }
    }
    std::cout<<"Finished distance computation in: "<<timer0.GetTime()<<std::endl;    

//========================================================================================

    int countIn = 0;
    v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      Vec3 &v = *v_it.Get();
      int idx = v_it.idx();
      if (grid_.m_myTraits[idx].iTag == 1) {
        countIn++;
      }
    }
    std::cout<<"Points inside: " << countIn <<std::endl;

    v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      myUniformGrid.insertElement(v_it.idx(), grid_.Vertex(v_it.idx()), 0.01);
    }
    myUniformGrid.printStatistics();

//========================================================================================

    v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      Vec3 &v = *v_it.Get();
      int idx = v_it.idx();
      grid_.m_myTraits[idx].iTag = 0;
    }

//========================================================================================

    timer0.Start();
    AABB3r box = myUniformGrid.boundingBox_;
    for (RigidBody* body : myWorld_.rigidBodies_) {

      if (box.isPointInside(body->com_)) {
        myUniformGrid.levels_[0].querySpherePoint(body);
      }
      else {
        CDistanceAabbPoint<Real> distAABBPoint(box, body->com_);

        Real dist = SqDistPointAABB(body->com_, box);
        std::cout << "Distance1: " << dist << std::endl;
        dist = distAABBPoint.ComputeDistanceSqr();
        std::cout << "Distance2: " << dist << std::endl;
        // Intersection: dist <= rad * rad
        std::cout << "Distance: " << dist << std::endl;
        if (dist < 0.015 * 0.015) {
          myUniformGrid.levels_[0].querySpherePoint(body);
        }
      }
    }

    for (RigidBody* body : myWorld_.rigidBodies_) {
      for (auto idx : body->elements_) {
        if (body->isInBody(grid_.Vertex(idx))) {
          grid_.m_myTraits[idx].iTag = 1;
        }
      }
    }
    std::cout<<"Finished distance computation in: "<<timer0.GetTime()<<std::endl;    

//========================================================================================

    countIn = 0;
    v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      Vec3 &v = *v_it.Get();
      int idx = v_it.idx();
      if (grid_.m_myTraits[idx].iTag == 1) {
        countIn++;
      }
    }

    std::cout<<"UniformGrid Points inside: " << countIn <<std::endl;

//========================================================================================

    CVtkWriter writer;
    writer.WriteUnstr(grid_, "output/mesh.vtk");
    writer.WriteUniformGrid(myUniformGrid.levels_[0], "output/unigrid.vtk");
    writeOutput(0);
    writeOutput(1);

  }
};
}

using namespace i3d;

int main()
{

  FBMTest3 myApp;

  // init application
  myApp.init("start/sampleRigidBody.xml");

  // run simulation
  myApp.run();
  std::cout << "Done" << std::endl;
  
  return EXIT_SUCCESS;
}
