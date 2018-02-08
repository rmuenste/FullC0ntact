#include <iostream>
#include <application.h>
#include <reader.h>
#include <meshobject.h>
#include <perftimer.h>
#include <vtkwriter.h>
#include <termcolor.hpp>

/*
* Includes for CGAL
*/
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/config.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include <boundaryshape.hpp>
#include <boundarydescription.hpp>

// Choose a geometry kernel
typedef CGAL::Simple_cartesian<double> Kernel;

// Make a short-hand for the geometry Kernel type
typedef Kernel::FT FT;

// Define short-hands for the other CGAL types that we
// are going to use in the application
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;

typedef Kernel::Point_3 Point;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Vector_3 Vector;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef Kernel::Ray_3 Ray;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>        Facet_Primitive;
typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;

namespace i3d {
 
  class PreprocessingTest : public Application {

  public:  

  std::vector<Tree*> trees;

  std::vector<Polyhedron*> polyhedra;

  BoundaryDescription<cgalKernel> bndry_;

  PreprocessingTest() : Application()
  {
        
  }
  
  virtual ~PreprocessingTest() {};

  /*
   * Function to convert the CGAL class Point to a Vec3
   */ 
  inline Vec3 pointToVec3(const Point &p)
  {
    return Vec3(p.x(), p.y(), p.z());
  }

  double random_in(const double a, const double b)
  {
    double r = rand() / (double)RAND_MAX;
    return (double)(a + (b - a) * r);
  }

  /*
   * This function generates and returns a
   * random 3d vector with components x,y,z in [0,1]
   */ 
  Vector random_vector()
  {
    double x = random_in(0.0, 1.0);
    double y = random_in(0.0, 1.0);
    double z = random_in(0.0, 1.0);
    return Vector(x, y, z);
  }
  
  /*
  *
  * This function has to be called to initialize the CGAL component
  *
  */
  void initGeometry() {

    // only one output file
    unsigned nOut = 0;
    
    for (auto &b : myWorld_.rigidBodies_)
    {

      if (b->shapeId_ == RigidBody::BOUNDARYBOX)
        continue;

      MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(b->shape_);
      
      std::string objPath = pMeshObject->getFileName();
      std::string::size_type dpos = objPath.rfind(".");
      std::string offPath = objPath;
      offPath.replace(dpos + 1, 3, "off");

      if (myWorld_.parInfo_.getId() == 1)
      {
        std::cout << "Name of mesh file: " << objPath << std::endl;
        std::cout << "Name of off file: " << offPath << std::endl;
      }

      // Load a mesh from file in the CGAL off format
      std::ifstream in(offPath);

      if (!in)
      {
        std::cerr << "unable to open file" << std::endl;
        std::exit(EXIT_FAILURE);
      }

      Polyhedron *polyhedron = new Polyhedron();
      // Read the polyhedron from the stream
      in >> *polyhedron;

      if (!in)
      {
        std::cerr << "invalid OFF file" << std::endl;
        delete polyhedron;
        polyhedron = nullptr;
        std::exit(EXIT_FAILURE);
      }

      in.close();

      polyhedra.push_back(polyhedron);

      std::cout << "OFF file loaded successfully" << std::endl;
    }

  }

  void readE3d(std::string fileName)
  {

    E3dReader reader;
    reader.readE3dFile(fileName);
    
  }

  void init(std::string fileName)
  {

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
    else if (ending == ".offs")
    {

      OffsReader reader;
      //Get the name of the mesh file from the
      //configuration data file.
      reader.readParameters(fileName, dataFileParams_);

    }//end if
    else
    {
      std::cerr << "Invalid data file ending: " << ending << std::endl;
      std::exit(EXIT_FAILURE);
    }//end else

    std::string meshFile("meshes/Mesh.tri");
    hasMeshFile_ = 1;

    if (hasMeshFile_)
    {
      std::string fileName;
      grid_.initMeshFromFile(meshFile.c_str());
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

    //set which type of rigid motion we are dealing with
    myMotion_ = new RigidBodyMotion(&myWorld_);

    //set the integrator in the pipeline
    myPipeline_.integrator_ = myMotion_;

    myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

    myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

    myPipeline_.response_->m_pGraph = myPipeline_.graph_;


    bndry_.addBoundaryShape(new BoundaryShapeTriSurf<cgalKernel>("meshes/med_top.off"));
    bndry_.addBoundaryShape(new BoundaryShapeTriSurf<cgalKernel>("meshes/med_bot.off"));
    bndry_.addBoundaryShape(new BoundaryShapeTriSurf<cgalKernel>("meshes/med_side.off"));

    Vec3 test(0,0,0);

    std::cout << bndry_.boundaryShapes_[0]->projectPoint(test);
    std::cout << bndry_.boundaryShapes_[1]->projectPoint(test);
    std::cout << bndry_.boundaryShapes_[2]->projectPoint(test);

    BoundaryShapePolyLine<cgalKernel> bndryLine1("meshes/med_ring_top.obj");

  }

  /*
   * In this method we write out the calculated results into
   * a file that can be visualized by ParaView
   */
  void writeGrid(int out)
  {

    std::ostringstream sNameGrid;
    std::string sGrid("output/grid.vtk");

    sNameGrid << "." << std::setfill('0') << std::setw(5) << 0;
    sGrid.append(sNameGrid.str());

    CVtkWriter writer;
    writer.WriteUnstr(grid_, sGrid.c_str());

  }
  
  void run()
  {


    VertexIter<Real> vIter;
    std::cout<<"Computing FBM information and distance..."<<std::endl;
    int icount=0;

    // Generate a direction vector for the ray
    Vector dir = random_vector();

    for(vIter=grid_.vertices_begin();vIter!=grid_.vertices_end();vIter++)
    {
      VECTOR3 vec = VECTOR3((*vIter).x,(*vIter).y,(*vIter).z);
      int in;
      int id = vIter.idx();

      Vec3 vDir(dir.x(), dir.y(), dir.z());
      grid_.m_myTraits[vIter.idx()].iTag=0;

      for (auto &body : myWorld_.rigidBodies_)
      {

        if(body->isInBody(vec, vDir))
        {
          grid_.m_myTraits[vIter.idx()].iTag=1;

          break;
        }

      }

      Real dmin = std::numeric_limits<Real>::max();
      int idx = 0;
      for (auto &body : myWorld_.rigidBodies_)
      {

        if (body->shapeId_ != RigidBody::CGALMESH)
          continue;

        Real dist = body->getMinimumDistance(vec);
          
        if (dist < dmin)
        {
          dmin = dist;
        }
        idx++;
      }

      if(grid_.m_myTraits[vIter.idx()].iTag)
        dmin*=-1.0;
          
      grid_.m_myTraits[vIter.idx()].distance=dmin;        
      
      if(icount%1000==0)
      {
        std::cout<<"Progress: "<<icount<<"/"<<grid_.nvt_<<std::endl;        
      }        

      icount++;
    }      

    writeGrid(0);
  }
    
};
  
}

using namespace i3d;


int main()
{

  PreprocessingTest myApp;
  
  myApp.init("start/sampleRigidBody.xml");
  
  myApp.run();
  
  return 0;
}
