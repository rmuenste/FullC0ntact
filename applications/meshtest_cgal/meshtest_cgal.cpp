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

        xmin_ = -2.5f;
        ymin_ = -2.5f;
        zmin_ = -4.5f;
        xmax_ = 2.5f;
        ymax_ = 2.5f;
        zmax_ = 1.5f;

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

            //pMeshObject->m_BVH.GenTreeStatistics();

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
//                writer.writePostScriptTree(pMeshObject->m_BVH,dir.c_str());
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

        //allocate_distancemaps(myWorld_.rigidBodies_, myWorld_.maps_);
        //exit(0);
        configureTimeDiscretization();

        //link the boundary to the world
        myWorld_.setBoundary(&myBoundary_);

        //set the time control
        myWorld_.setTimeControl(&myTimeControl_);

        //set the gravity
        myWorld_.setGravity(dataFileParams_.gravity_);

        //set air friction
        myWorld_.setAirFriction(dataFileParams_.airFriction_);

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

        myWorld_.graph_ = myPipeline_.graph_;

       }

      void writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
      {
        std::ostringstream sName, sNameParticles, sphereFile;
        std::string sModel("output/model.vtk");
        std::string sParticleFile("output/particle.vtk");
        std::string sParticle("solution/particles.i3d");
        CVtkWriter writer;
        int iTimestep = out;
        sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sNameParticles << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sModel.append(sName.str());
        sParticleFile.append(sName.str());
        sParticle.append(sNameParticles.str());
        sphereFile << "output/spheres.vtk." << std::setfill('0') << std::setw(5) << iTimestep;
        //Write the grid to a file and measure the time
        writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());
        //writer.WriteParticleFile(myWorld_.rigidBodies_, sParticleFile.c_str());

        if(writeRBSpheres)
        {
          writer.WriteSpheresMesh(myWorld_.rigidBodies_, sphereFile.str().c_str());
        }

        if(writeRBCom)
        {
          std::ostringstream coms;
          coms << "output/com_data.vtk" << "." << std::setfill('0') << std::setw(5) << iTimestep;
          writer.WriteRigidBodyCom(myWorld_.rigidBodies_, coms.str().c_str());
        }

        if (out == 0 || out ==1)
        {
          std::ostringstream sNameGrid;
          std::string sGrid("output/grid.vtk");
          sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
          sGrid.append(sNameGrid.str());
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
      }

      void run() {

        unsigned nOut = 0;
        //start the main simulation loop
        for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
        {
          Real simTime = myTimeControl_.GetTime();
          Real energy0 = myWorld_.getTotalEnergy();
          std::cout << "------------------------------------------------------------------------" << std::endl;
          std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
            << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
          std::cout << "Energy: " << energy0 << std::endl;
          std::cout << "------------------------------------------------------------------------" << std::endl;
          std::cout << std::endl;
//          myPipeline_.startPipeline();
          Real energy1 = myWorld_.getTotalEnergy();
          std::cout << "Energy after collision: " << energy1 << std::endl;
          std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
          std::cout << "Timestep finished... writing vtk." << std::endl;
          //if(nOut%10==0)
            writeOutput(nOut,false,false);
          std::cout << "Finished writing vtk." << std::endl;
          nOut++;
          myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
        }//end for

      }

  };

}



int main()
{
  using namespace i3d;

  MeshMeshTest myApp;

  myApp.init(std::string("start/sampleRigidBody.xml"));

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

  myApp.run();

  return 0;
}
