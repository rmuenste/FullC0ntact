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

  }
  
  
  void run()
  {

    grid_.initMeshFromFile("testmesh.tri");
    myUniformGrid.initGrid(grid_.getAABB(), 1);
    myUniformGrid.initGridLevel(0, 0.03);
    VertexIter<Real> v_it = grid_.vertices_begin();
    for (; v_it != grid_.vertices_end(); v_it++) {
      myUniformGrid.insertElement(v_it.idx(), grid_.Vertex(v_it.idx()), 0.01);
    }
    myUniformGrid.printStatistics();

    std::cout << "Number of mesh vertices: " << grid_.nvt_ << std::endl;
    std::cout << "Number of rigid bodies: " << myWorld_.rigidBodies_.size() << std::endl;
    std::cout << "Shape: " << myWorld_.rigidBodies_[0]->shapeId_ << std::endl;

    //OBB3(const Vector3<T>& center, const Vector3<T> axis[3], const T extent[3]);

    AABB3r box = myUniformGrid.boundingBox_;

    if (box.isPointInside(myWorld_.rigidBodies_[0]->com_)) {
      myUniformGrid.levels_[0].querySpherePoint(myWorld_.rigidBodies_[0]);
    }
    else {
      CDistanceAabbPoint<Real> distAABBPoint(box, myWorld_.rigidBodies_[0]->com_);

      Real dist = SqDistPointAABB(myWorld_.rigidBodies_[0]->com_, box);
      std::cout << "Distance1: " << dist << std::endl;
      dist = distAABBPoint.ComputeDistanceSqr();
      std::cout << "Distance2: " << dist << std::endl;
      // Intersection: dist <= rad * rad
      std::cout << "Distance: " << dist << std::endl;
      if (dist < 0.015 * 0.015) {
        myUniformGrid.levels_[0].querySpherePoint(myWorld_.rigidBodies_[0]);
      }
    }

    std::cout << "Number of points: " << myWorld_.rigidBodies_[0]->elements_.size() << std::endl;

    CVtkWriter writer;
    writer.WriteUnstr(grid_, "output/mesh.vtk");
    writer.WriteUniformGrid(myUniformGrid.levels_[0], "unigrid.vtk");
    writeOutput(0);

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
