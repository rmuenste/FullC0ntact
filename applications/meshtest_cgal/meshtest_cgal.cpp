#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <iostream>
#include <fstream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>

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

  /*
   * Function to convert the CGAL class Point to a Vec3
   */ 
  inline Vec3 pointToVec3(const Point &p)
  {
    return Vec3(p.x(), p.y(), p.z());
  }

  class MeshTestCGAL : public Application {

  public:

    Polyhedron *Polyhedron_;

    MeshTestCGAL() : Application() {

    }

    ~MeshTestCGAL() {};

    double random_in(const double a,
      const double b)
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
     * The init function of a FullC0ntact application. The required
     * data structures are initialized here.
     * @param fileName File name of the application data file
     *
     */
    void init(std::string fileName) {

      size_t pos = fileName.find(".");
      std::string ending = fileName.substr(pos);

      std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
      if (ending == ".xml")
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

      for (int j = 0; j < myWorld_.rigidBodies_.size(); j++)
      {
        myWorld_.rigidBodies_[j]->iID_ = j;
      }

      //Distance map initialization
      std::set<std::string> fileNames;

      for (auto &body : myWorld_.rigidBodies_)
      {

        if (body->shapeId_ != RigidBody::MESH)
          continue;

        MeshObjectr *meshObject = dynamic_cast<MeshObjectr *>(body->shape_);
        std::string objName = meshObject->GetFileName();
        fileNames.insert(objName);

      }

      int iHandle = 0;
      for (auto const &myName : fileNames)
      {
        bool created = false;
        for (auto &body : myWorld_.rigidBodies_)
        {
          if (body->shapeId_ != RigidBody::MESH)
            continue;

          MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);

          std::string objName = pMeshObject->GetFileName();
          if (objName == myName)
          {
            if (created)
            {
              //if map created -> add reference
              body->map_ = myWorld_.maps_.back();
            }
            else
            {
              //if map not created -> create and add reference
              body->buildDistanceMap();
              myWorld_.maps_.push_back(body->map_);
              created = true;
              CVtkWriter writer;
              std::string n = myName;
              const size_t last = n.find_last_of("\\/");
              if (std::string::npos != last)
              {
                n.erase(0, last);
              }
              const size_t period = n.rfind(".");
              if (std::string::npos != period)
              {
                n.erase(period);
              }
              n.append(".ps");
              std::string dir("output/");
              dir.append(n);
            }
          }
        }
      }
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

      CUnstrGridr ugrid;
      for (auto &body : myWorld_.rigidBodies_)
      {

        if (body->shapeId_ != RigidBody::MESH)
          continue;

        body->map_->convertToUnstructuredGrid(ugrid);
        writer.WriteUnstr(ugrid, "output/DistanceMap.vtk");
        break;
      }

    }

    void run() {

      // only one output file
      unsigned nOut = 0;
        
      // Get access to the rigid body wrapper class
      RigidBody *body = myWorld_.rigidBodies_.front();

      MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(body->shape_);
      std::string objPath = pMeshObject->GetFileName();

      std::cout << "Name of mesh file: " << objPath << std::endl;

      // Load a mesh from file in the CGAL off format
      std::ifstream in(std::string("meshes/blood_cell.off"));

      if (!in)
      {
        std::cerr << "unable to open file" << std::endl;
        std::exit(EXIT_FAILURE);
      }

      Polyhedron_ = new Polyhedron();

      // Read the polyhedron from the stream
      in >> *Polyhedron_;
      if (!in)
      {
        std::cerr << "invalid OFF file" << std::endl;
        delete Polyhedron_;
        Polyhedron_ = nullptr;
        std::exit(EXIT_FAILURE);
      }

      in.close();

      std::cout << "OFF file loaded successfully" << std::endl;

      std::cout << "Construct AABB tree...";

      // Construct an instance of the CGAL::AABB_tree<Traits> tree
      // from the polyhedron we have just read in
      Tree tree(faces(*Polyhedron_).first, faces(*Polyhedron_).second, *Polyhedron_);

      // Use the acceleration method for distances
      tree.accelerate_distance_queries();

      std::cout << "done." << std::endl;

      // Compute the total number of points
      int total = (body->map_->cells_[0] + 1)*(body->map_->cells_[1] + 1)*(body->map_->cells_[2] + 1);

      // Generate a direction vector for the ray
      Vector vec = random_vector();

      // Get a timer object and start the timer 
      // to measure the ray intersection tests
      CPerfTimer timer;
      timer.Start();

      // Loop over all points and perform the ray intersection test 
      // for each point of the mesh
      for (int i = 0; i < total; i++)
      {
        VECTOR3 vQuery = body->map_->vertexCoords_[i];

        Point p(vQuery.x, vQuery.y, vQuery.z);

        Ray ray(p, vec);

        int nb_intersections = (int)tree.number_of_intersected_primitives(ray);

        // Check for odd or even number of intersections
        if (nb_intersections % 2 != 0)
          body->map_->stateFBM_[i] = 1;
        else
          body->map_->stateFBM_[i] = 0;

      }

      timer.Stop();
      std::cout << "FBM CGAL: " << timer.GetTime() << "[ms]" << std::endl;

      // Start the timer again to measure the distance calculation
      timer.Start();

      // Loop over all points and perform distance computation
      // for each point of the mesh
      for (int i = 0; i < total; i++)
      {
        VECTOR3 vQuery = body->map_->vertexCoords_[i];
        Point p(vQuery.x, vQuery.y, vQuery.z);
        Point cp = tree.closest_point(p);

        body->map_->distance_[i] = std::sqrt(CGAL::squared_distance(p, cp));

        // For an inner point we multiply by
        // -1 to get the signed distance function
        if (body->map_->stateFBM_[i])
          body->map_->distance_[i] *= -1.0;

        // also store the point on the surface of the object
        // with the shortest distance the input mesh point
        Vec3 contactPoint = pointToVec3(cp);
        body->map_->contactPoints_[i] = contactPoint;

        // construct a surface normal which may be
        // needed for other calculations
        body->map_->normals_[i] = vQuery - contactPoint;
        body->map_->normals_[i].normalize();
      }

      timer.Stop();
      std::cout << "Dist CGAL: " << timer.GetTime() << "[ms]" << std::endl;

      // Write out the computed result in VTK ParaView format 
      writeGrid(nOut);
    }

  };

}

int main()
{
  using namespace i3d;

  MeshTestCGAL myApp;

  myApp.init(std::string("start/sampleRigidBody.xml"));

  myApp.run();

  return EXIT_SUCCESS;
}
