#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <iostream>
#include <fstream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/config.h>
#include <CGAL/Polyhedron_3.h>


#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;

typedef Kernel::Point_3 Point;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Ray_3 Ray;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>        Facet_Primitive;
typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;


namespace i3d {

  inline Vec3 pointToVec3(const Point &p)
  {
    return Vec3(p.x(), p.y(), p.z());
  }
  
  class MeshMeshTest : public Application {

    public:

      Polyhedron *Polyhedron_;

      MeshMeshTest() : Application() {

      }

      ~MeshMeshTest() {};

      double random_in(const double a,
                          const double b)
      {
          double r = rand() / (double)RAND_MAX;
          return (double)(a + (b - a) * r);
      }


      Vector random_vector()
      {
          double x = random_in(0.0,1.0);
          double y = random_in(0.0,1.0);
          double z = random_in(0.0,1.0);
          return Vector(x,y,z);
      }


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

        std::ifstream in(std::string("meshes/blood_cell.off"));

        if (!in)
        {
          std::cerr << "unable to open file" << std::endl;
          std::exit(EXIT_FAILURE);
        }

        Polyhedron_ = new Polyhedron();

        in >> *Polyhedron_;
        if (!in)
        {
          std::cerr << "invalid OFF file" << std::endl;
          delete Polyhedron_;
          Polyhedron_ = nullptr;
          std::exit(EXIT_FAILURE);
        }
        std::cout << "OFF file loaded successfully" << std::endl;

        std::cout << "Construct AABB tree...";
        Tree tree(faces(*Polyhedron_).first, faces(*Polyhedron_).second,*Polyhedron_);
        tree.accelerate_distance_queries();
        std::cout << "done." << std::endl;

//        Polyhedron::Point_iterator it = Polyhedron_->points_begin();

//        for(; it != Polyhedron_->points_end();it++)
//            m_bbox = m_bbox + (*it).bbox();

//        std::cout << "done (" << Polyhedron_->size_of_facets()
//                  << " facets)" << std::endl;


        //assign the rigid body ids
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

          CMeshObjectr *meshObject = dynamic_cast<CMeshObjectr *>(body->shape_);
          std::string objName = meshObject->GetFileName();
          fileNames.insert(objName);
        }

        int iHandle=0;
        for (auto const &myName : fileNames)
        {
          bool created = false;
          for (auto &body : myWorld_.rigidBodies_)
          {
            if (body->shapeId_ != RigidBody::MESH)
              continue;

            CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);

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
                if(std::string::npos != last)
                {
                  n.erase(0,last);
                }
                const size_t period = n.rfind(".");
                if(std::string::npos != period)
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

        std::cout << myWorld_.maps_.size() << std::endl;

        RigidBody *body = myWorld_.rigidBodies_[0];

        int total = (body->map_->cells_[0]+1)*(body->map_->cells_[1]+1)*(body->map_->cells_[2]+1);

        CPerfTimer timer;
        timer.Start();
        Vector vec = random_vector();

        for (int i = 0; i < total; i++)
        {
          VECTOR3 vQuery = body->map_->vertexCoords_[i];

          Point p(vQuery.x,vQuery.y,vQuery.z);

          Ray ray(p,vec);

          int nb_intersections = (int)tree.number_of_intersected_primitives(ray);
          if(nb_intersections % 2 != 0)
          {
            body->map_->stateFBM_[i] = 1;
            //std::cout << "> Res: in" << std::endl;
          }
          else
          {
            body->map_->stateFBM_[i] = 0;
            //std::cout << "> Res: out" << std::endl;
          }

        }
        timer.Stop();
        std::cout << "FBM CGAL: " << timer.GetTime() << "[ms]" << std::endl;

        timer.Start();
        for (int i = 0; i < total; i++)
        {

          VECTOR3 vQuery = body->map_->vertexCoords_[i];
          Point p(vQuery.x,vQuery.y,vQuery.z);
          Point cp = tree.closest_point(p);

          body->map_->distance_[i] = std::sqrt(CGAL::squared_distance(p, cp));

          if(body->map_->stateFBM_[i])
            body->map_->distance_[i]*=-1.0;


          Vec3 contactPoint = pointToVec3(cp);
          body->map_->contactPoints_[i] = contactPoint;

          body->map_->normals_[i] = vQuery - contactPoint;
          body->map_->normals_[i].normalize();

        }
        timer.Stop();
        std::cout << "Dist CGAL: " << timer.GetTime() << "[ms]" << std::endl;
        
        body->storeDistanceMapToFile("output/dmap.dmp");

       }

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

        Point_2 p(1, 1), q(10, 10);

        std::cout << "p = " << p << std::endl;
        
        std::cout << "q = " << q.x() << " " << q.y() << std::endl;

        std::cout << " sqdist(p,q) = " << CGAL::squared_distance(p, q) << std::endl;

        Segment_2 s(p, q);

        Point_2 m(5, 9);

        std::cout << "m = " << m << std::endl;

        std::cout << " sqdist(Segment_2(p,q), m) = " << CGAL::squared_distance(s,m) << std::endl;

        std::cout << "p, q, and m ";
        switch (CGAL::orientation(p, q, m)) {

        case CGAL::COLLINEAR:
          std::cout << "are collinear\n";
          break;
        case CGAL::LEFT_TURN:
          std::cout << "make a left turn\n";
          break;
        case CGAL::RIGHT_TURN:
          std::cout << "make a left turn \n";
          break;
        }

        std::cout << " midpoint(p,q) =  " << CGAL::midpoint(p,q) << std::endl;

        // start the main simulation loop
        writeGrid(nOut);

      }

  };

}

int main()
{
  using namespace i3d;

  MeshMeshTest myApp;

  myApp.init(std::string("start/sampleRigidBody.xml"));

  myApp.run();

  return 0;
}
