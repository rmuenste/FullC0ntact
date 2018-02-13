#ifndef GEOM_CONFIG_HPP_Z96BD1LO
#define GEOM_CONFIG_HPP_Z96BD1LO


#ifdef WITH_CGAL
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
#include <CGAL/AABB_segment_primitive.h>
#endif

namespace i3d {

  enum
  {
    defaultKernel,
    cgalKernel
  };


#ifdef WITH_CGAL

const int geom_kernel = cgalKernel;
#else
const int geom_kernel = defaultKernel;
#endif

}


#endif /* end of include guard: GEOM_CONFIG_HPP_Z96BD1LO */
