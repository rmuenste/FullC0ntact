#ifndef CGAL_INTERFACE_HPP_LKFPA5HE
#define CGAL_INTERFACE_HPP_LKFPA5HE


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

namespace i3d {
// Choose a geometry kernel
typedef CGAL::Simple_cartesian<double> Kernel;

// Make a short-hand for the geometry Kernel type
typedef Kernel::FT FT;

// Define short-hands for the other CGAL types that we
// are going to use in the application
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;

typedef Kernel::Point_3 cgalPoint;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Vector_3 Vec;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef Kernel::Ray_3 cgalRay;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>        Facet_Primitive;
typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;

std::vector<Tree*> trees;

std::vector<Polyhedron*> polyhedra;
};

/*
 * Function to convert the CGAL class Point to a Vec3
 */ 
inline Vec3 pointToVec3(const cgalPoint &p)
{
  return Vec3(p.x(), p.y(), p.z());
}

double random_in(const double a,
  const double b)
{
  double r = rand() / (double)RAND_MAX;
  return (double)(a + (b - a) * r);
}


using namespace i3d;
/*
 * This function generates and returns a
 * random 3d vector with components x,y,z in [0,1]
 */
Vec random_vector()
{
  double x = random_in(0.0, 1.0);
  double y = random_in(0.0, 1.0);
  double z = random_in(0.0, 1.0);
  return Vec(x, y, z);
}


/*
 * 
 * This function has to be called to initialize the CGAL component 
 *
 */
void initGeometry() {

  // only one output file
  unsigned nOut = 0;

  for (auto &b : myWorld.rigidBodies_)
  {

    if (b->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;

    MeshObjectr *pMeshObject = dynamic_cast<MeshObjectr *>(b->shape_);
    std::string objPath = pMeshObject->getFileName();
    std::string::size_type dpos = objPath.rfind(".");
    std::string offPath = objPath; 
    offPath.replace(dpos+1,3,"off");
  
    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Name of mesh file: " << objPath << std::endl;
      std::cout << "Name of off file: " << offPath << std::endl;
    }

    // Load a mesh from file in the CGAL off format
    std::ifstream in(offPath);

    if (!in)
    {
      std::cerr << "unable to open file: " << offPath << std::endl;
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

void buildTreeStructures()
{

  std::cout << "Construct AABB tree...";

  // Construct an instance of the CGAL::AABB_tree<Traits> tree
  // from the polyhedron we have just read in
  for (auto &polyhedron : polyhedra)
  {

    Tree *tree = new Tree(faces(*polyhedron).first, faces(*polyhedron).second, *polyhedron);

    // Use the acceleration method for distances
    tree->accelerate_distance_queries();

    trees.push_back(tree);

  }

  std::cout << "done." << std::endl;

}

int computeSinglePointInside(RigidBody *body, Tree &tree, const Vec3 &vQuery)
{

  // Generate a direction vector for the ray
  Vec vec = random_vector();

  cgalPoint p(vQuery.x, vQuery.y, vQuery.z);

  cgalRay ray(p, vec);

  int nb_intersections = (int)tree.number_of_intersected_primitives(ray);

  // Check for odd or even number of intersections
  if (nb_intersections % 2 != 0)
    return 1;
  else
    return 0;

}

void computePointsInside(RigidBody *body, Tree &tree)
{

  // Compute the total number of points
  int total = (body->map_->cells_[0] + 1)*(body->map_->cells_[1] + 1)*(body->map_->cells_[2] + 1);

  // Generate a direction vector for the ray
  Vec vec = random_vector();

  // Get a timer object and start the timer 
  // to measure the ray intersection tests
  CPerfTimer timer;
  timer.Start();

  // Loop over all points and perform the ray intersection test 
  // for each point of the mesh
  for (int i = 0; i < total; i++)
  {
    VECTOR3 vQuery = body->map_->vertexCoords_[i];

    cgalPoint p(vQuery.x, vQuery.y, vQuery.z);

    cgalRay ray(p, vec);

    int nb_intersections = (int)tree.number_of_intersected_primitives(ray);

    // Check for odd or even number of intersections
    if (nb_intersections % 2 != 0)
      body->map_->stateFBM_[i] = 1;
    else
      body->map_->stateFBM_[i] = 0;

  }

  timer.Stop();
  std::cout << "FBM CGAL: " << timer.GetTime() << "[ms]" << std::endl;

}

double computeSinglePointDistance(Tree &tree, const Vec3 &vQuery)
{

  cgalPoint p(vQuery.x, vQuery.y, vQuery.z);
  cgalPoint cp = tree.closest_point(p);

  double d = std::sqrt(CGAL::squared_distance(p, cp));

  return d;

}

//void computePointsDistance(double coords[][3],double *distances,RigidBody *body, Tree &tree) {
//
//  // Compute the total number of points
//  int total = (body->map_->cells_[0] + 1)*(body->map_->cells_[1] + 1)*(body->map_->cells_[2] + 1);
//
//  CPerfTimer timer;
//  // Start the timer again to measure the distance calculation
//  timer.Start();
//
//  // Loop over all points and perform distance computation
//  // for each point of the mesh
//  for (int i = 0; i < total; i++)
//  {
//    VECTOR3 vQuery = body->map_->vertexCoords_[i];
//    Point p(vQuery.x, vQuery.y, vQuery.z);
//    Point cp = tree.closest_point(p);
//
//    body->map_->distance_[i] = std::sqrt(CGAL::squared_distance(p, cp));
//
//    // For an inner point we multiply by
//    // -1 to get the signed distance function
//    if (body->map_->stateFBM_[i])
//      body->map_->distance_[i] *= -1.0;
//
//    // also store the point on the surface of the object
//    // with the shortest distance the input mesh point
//    Vec3 contactPoint = pointToVec3(cp);
//    body->map_->contactPoints_[i] = contactPoint;
//
//    // construct a surface normal which may be
//    // needed for other calculations
//    body->map_->normals_[i] = vQuery - contactPoint;
//    body->map_->normals_[i].normalize();
//  }
//
//  timer.Stop();
//  std::cout << "Dist CGAL: " << timer.GetTime() << "[ms]" << std::endl;
//
//}
#endif
