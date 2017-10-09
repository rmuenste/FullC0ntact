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

typedef Kernel::Point_3 Point_3;

typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;

typedef Polyhedron::Vertex_iterator Vertex_iterator;

int main()
{

  Point_3 p( 1.0, 0.0, 0.0);
  Point_3 q( 0.0, 1.0, 0.0);
  Point_3 r( 0.0, 0.0, 1.0);
  Point_3 s( 0.0, 0.0, 0.0);

  Polyhedron P;

  P.make_tetrahedron(p, q, r, s);

  CGAL::set_ascii_mode(std::cout);

  std::copy(P.points_begin(), P.points_end(), std::ostream_iterator<Point_3>(std::cout, "\n"));

  return EXIT_SUCCESS;
}
