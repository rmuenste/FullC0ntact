#ifndef BOUNDARYSHAPE_HPP_J5UYEBMN
#define BOUNDARYSHAPE_HPP_J5UYEBMN

#include <string>
#include <vector3.h>
#include <geom_config.hpp>
#include <objloader.h>

namespace i3d
{

  template<int geomBackend>
  class BasicBoundaryShape
  {

    public:

      int backEnd_;

      std::string fileName_;

      BasicBoundaryShape () = default;

      BasicBoundaryShape(const std::string &fileName) : fileName_(fileName)
      {
      }

      virtual ~BasicBoundaryShape () {};

      virtual Vec3 projectPoint(const Vector3<Real> &v) = 0;


    private:
      /* data */

  };

  template<int geomBackend>
  class BoundaryShapeTriSurf : public BasicBoundaryShape<geomBackend>
  {

    public:

      BoundaryShapeTriSurf () = default;

      virtual ~BoundaryShapeTriSurf () {};

      virtual Vec3 projectPoint(const Vector3<Real> &v)
      {
        return Vec3();
      }

    private:
      /* data */

  };

 #ifdef WITH_CGAL

  template<>
  class BoundaryShapeTriSurf<cgalKernel> : public BasicBoundaryShape<cgalKernel>
  {

    public:

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


      Tree *tree_;

      Polyhedron polyhedron_;

      BoundaryShapeTriSurf () = default;

      BoundaryShapeTriSurf(const std::string &fileName) : BasicBoundaryShape<cgalKernel>(fileName)
      {

        // Load a mesh from file in the CGAL off format
        std::ifstream in(fileName);

        if (!in)
        {
          std::cerr << "unable to open file" << std::endl;
          std::exit(EXIT_FAILURE);
        }

        // Read the polyhedron from the stream
        in >> polyhedron_;

        if (!in)
        {
          std::cerr << "File: "  << fileName << " invalid OFF file" << std::endl;
          std::exit(EXIT_FAILURE);
        }

        in.close();

        std::cout << "OFF file: " << fileName << " loaded successfully" << std::endl;

        std::cout << "Construct AABB tree...";


        tree_ = new Tree(faces(polyhedron_).first, faces(polyhedron_).second, polyhedron_);

        // Use the acceleration method for distances
        tree_->accelerate_distance_queries();

        std::cout << "done." << std::endl;

      }

      virtual ~BoundaryShapeTriSurf () {};

      virtual Vec3 projectPoint(const Vector3<Real> &v)
      {


        Point p(v.x, v.y, v.z);
        Point cp;
        Point nearestPoint;

        Real dmin = std::numeric_limits<Real>::max();

        cp = tree_->closest_point(p);
        //Real dist = CGAL::squared_distance(p, cp);

        return Vec3(cp.x(), cp.y(), cp.z());

      }

    private:
      /* data */

  };

#endif

  template<int geomBackend>
  class BoundaryShapePolyLine : public BasicBoundaryShape<geomBackend>
  {

    public:

      BoundaryShapePolyLine () = default;

      virtual ~BoundaryShapePolyLine () {};

      virtual Vec3 projectPoint(const Vector3<Real> &v)
      {
        return Vec3();
      }

    private:
      /* data */

  };

 #ifdef WITH_CGAL

  template<>
  class BoundaryShapePolyLine<cgalKernel> : public BasicBoundaryShape<cgalKernel>
  {

    public:

      BoundaryShapePolyLine () = default;

      BoundaryShapePolyLine(const std::string &fileName) : BasicBoundaryShape<cgalKernel>(fileName)
      {
        using namespace std;

        ObjLoader loader;

        size_t pos = fileName.find(".");

        string sType = fileName.substr(pos);

        if(sType == ".obj")
        {
          ObjLoader Loader;
          Loader.readPolyLine(fileName);
          std::cout << "PolyLine: " << fileName << " loaded successfully." << std::endl;
        }//end if


//        typedef CGAL::Simple_cartesian<double> K;
//        typedef K::FT FT;
//        typedef K::Point_3 Point;
//        typedef K::Plane_3 Plane;
//        typedef K::Segment_3 Segment;
//        typedef K::Triangle_3 Triangle;
//        typedef std::list<Segment>::iterator Iterator;
//        typedef CGAL::AABB_segment_primitive<K, Iterator> Primitive;
//        typedef CGAL::AABB_traits<K, Primitive> Traits;
//        typedef CGAL::AABB_tree<Traits> Tree;
//
//            Point a(1.0, 0.0, 0.0);
//            Point b(0.0, 1.0, 0.0);
//            Point c(0.0, 0.0, 1.0);
//            Point d(0.0, 0.0, 0.0);
//            std::list<Segment> segments;
//            segments.push_back(Segment(a,b));
//            segments.push_back(Segment(a,c));
//            segments.push_back(Segment(c,d));
//            // constructs the AABB tree and the internal search tree for
//            // efficient distance computations.
//            Tree tree(segments.begin(),segments.end());
//            tree.accelerate_distance_queries();
//            // counts #intersections with a plane query
//            Plane plane_query(a,b,d);
//            std::cout << tree.number_of_intersected_primitives(plane_query)
//                << " intersections(s) with plane" << std::endl;
//            // counts #intersections with a triangle query
//            Triangle triangle_query(a,b,c);
//            std::cout << tree.number_of_intersected_primitives(triangle_query)
//                << " intersections(s) with triangle" << std::endl;
//            // computes the closest point from a point query
//            Point point_query(2.0, 2.0, 2.0);
//            Point closest = tree.closest_point(point_query);
//            std::cerr << "closest point is: " << closest << std::endl;




      }

      virtual ~BoundaryShapePolyLine () {};

      virtual Vec3 projectPoint(const Vector3<Real> &v)
      {
        return Vec3();
      }

    private:
      /* data */

  };

#endif

} /* i3d */ 


#endif /* end of include guard: BOUNDARYSHAPE_HPP_J5UYEBMN */
