#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <iostream>
#include <fstream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <Eigen/Sparse>
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
typedef Kernel::Segment_3 Segment;

typedef Kernel::Point_3 Point;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Triangle_3 Triangle;
typedef Kernel::Vector_3 Vector;

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

typedef Polyhedron::HalfedgeDS HalfedgeDS;

typedef Kernel::Ray_3 Ray;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron>        Facet_Primitive;
typedef CGAL::AABB_traits<Kernel, Facet_Primitive>                  Facet_Traits;
typedef CGAL::AABB_tree<Facet_Traits>                               Facet_tree;

typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;
typedef Polyhedron::Facet_iterator                   Facet_iterator;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;

namespace i3d {

  // A modifier creating a triangle with the incremental builder.
  // This is a class with an operator that is used to build a triangle
  template <class HDS>
  class Build_triangle : public CGAL::Modifier_base<HDS> {
  public:
      std::vector<Point> _vertices;
      Build_triangle(std::vector<Point> &vertices) {

        _vertices = vertices;

      }

      void operator()( HDS& hds) {
          // Postcondition: hds is a valid polyhedral surface.
          CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
          B.begin_surface( 4, 2, 0);
          typedef typename HDS::Vertex   Vertex;
          typedef typename Vertex::Point Point;
          B.add_vertex( _vertices[0]);
          B.add_vertex( _vertices[1]);
          B.add_vertex( _vertices[2]);
          B.add_vertex( _vertices[3]);
          B.begin_facet();
          B.add_vertex_to_facet( 0);
          B.add_vertex_to_facet( 1);
          B.add_vertex_to_facet( 2);
          B.end_facet();
          B.begin_facet();
          B.add_vertex_to_facet( 2);
          B.add_vertex_to_facet( 3);
          B.add_vertex_to_facet( 0);
          B.end_facet();
          B.end_surface();
      }
  };  

  /*
   * Function to convert the CGAL class Point to a Vec3
   */ 
  inline Vec3 pointToVec3(const Point &p)
  {
    return Vec3(p.x(), p.y(), p.z());
  }

  class MeshTestCGAL : public Application<> {

  public:

    Polyhedron *Polyhedron_;

    Tree *_tree;

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

      if (dataFileParams_.hasExtents_)
      {
        grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
          dataFileParams_.extents_[4], dataFileParams_.extents_[1],
          dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
      }
      else
        grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);

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

    void intersectionTest() {

      // constructs segment query
      Point a(0.0, 0.0, -2.2);
      Point b(0.0, 0.0, 2.2);
      Segment segment_query(a,b);      

      // computes first encountered intersection with segment query
      // (generally a point)
      Segment_intersection intersection =
          _tree->any_intersection(segment_query);

      if(intersection)
      {
          // gets intersection object
        const Point* p = boost::get<Point>(&(intersection->first));
        if(p)
          std::cout << "intersection object is a point " << *p << std::endl;
      }

    }

    void treeTest(Polyhedron &p) {

      _tree = new Tree(faces(p).first, faces(p).second, p);

      _tree->accelerate_distance_queries();

    }

    void makeFaceTest() {

      Polyhedron P;
      std::vector<Point> polyVertices;
      polyVertices.push_back(Point(-1,-1,-1));
      polyVertices.push_back(Point( 1,-1,-1));
      polyVertices.push_back(Point( 1, 1,-1));
      polyVertices.push_back(Point(-1, 1,-1));
      Build_triangle<HalfedgeDS> triangle(polyVertices);

      // Delegate the building of the Polyhedron to the Build_triangle operator class
      P.delegate( triangle);

      // Write polyhedron in Object File Format (OFF).
      CGAL::set_ascii_mode( std::cout);
      // OFF format:
      // Line1 OFF
      // Line2 #Vertices #Facets (#Facets can simply be set to 0)
      // vertices
      // x y z
      // faces
      // number-of-vertices-in-face idx0...idxN
      std::cout << "OFF" << std::endl << P.size_of_vertices() << ' '
                << P.size_of_facets() << " 0" << std::endl;

      // Copy the vertices to the output stream
      std::copy( P.points_begin(), P.points_end(),
                std::ostream_iterator<Point_3>( std::cout, "\n"));

      // Loop over the facets with the facet iterator
      for (  Facet_iterator i = P.facets_begin(); i != P.facets_end(); ++i) {
          Halfedge_facet_circulator j = i->facet_begin();
          // Facets in polyhedral surfaces are at least triangles.
          CGAL_assertion( CGAL::circulator_size(j) >= 3);

          // Write out the number of unique vertices in the circulator
          std::cout << CGAL::circulator_size(j) << ' ';

          // Write the vertex idx of the half-edge to the stream and
          // proceed to the next one until we have reached the start again
          do {
              std::cout << ' ' << std::distance(P.vertices_begin(), j->vertex());
          } while ( ++j != i->facet_begin());
          std::cout << std::endl;
      }

      treeTest(P); 
      intersectionTest(); 

    }

    void run() {

      std::vector<Real> vmass;

      grid_.initStdMesh();

      grid_.calcVol();

      ElementIter e_it = grid_.elem_begin();

      for(; e_it != grid_.elem_end(); e_it++) {

        int index = e_it.idx();
        std::cout << index << ")Hexa volume: " << grid_.elemVol_[index] << std::endl;

        Hexa &hexa = *e_it;

        // Configure the deformation axes

        // First axis
        DeformationAxis ax0;
        
        ax0.axis_ = Vec3(0, 0, 1);

        ax0.intersecPoints_[0] = Vec3( 0, 0,-1);
        ax0.intersecPoints_[1] = Vec3( 0, 0, 1);

        ax0.coeffs_[0] = (std::pair<Real, Real>(0.5, 0.5));
        ax0.coeffs_[1] = (std::pair<Real, Real>(0.5, 0.5));

        ax0.faceIdx[0] = 0; 
        ax0.faceIdx[1] = 5; 

        // Second axis
        DeformationAxis ax1;
        
        ax1.axis_ = Vec3(0, 1, 0);

        ax1.intersecPoints_[0] = Vec3(0,-1, 0);
        ax1.intersecPoints_[1] = Vec3(0, 1, 0);

        ax1.coeffs_[0] = (std::pair<Real, Real>(0.5, 0.5));
        ax1.coeffs_[1] = (std::pair<Real, Real>(0.5, 0.5));

        ax1.faceIdx[0] = 1; 
        ax1.faceIdx[1] = 3; 

//	{0,1,2,3}, -Z 0
//	{0,4,5,1}, -Y 1
//	{1,5,6,2}, +X 2
//	{2,6,7,3}, +Y 3
//	{0,3,7,4}, -X 4
//	{4,7,6,5}  +Z 5

        // Third axis
        DeformationAxis ax2;
        
        ax2.axis_ = Vec3(-1, 0, 0);

        ax2.intersecPoints_[0] = Vec3( 1, 0, 0);
        ax2.intersecPoints_[1] = Vec3(-1, 0, 0);

        ax2.coeffs_[0] = (std::pair<Real, Real>(0.5, 0.5));
        ax2.coeffs_[1] = (std::pair<Real, Real>(0.5, 0.5));

        ax2.faceIdx[0] = 2; 
        ax2.faceIdx[1] = 4; 

        hexa.axes_.push_back(ax0);
        hexa.axes_.push_back(ax1);
        hexa.axes_.push_back(ax2);

      }

      std::cout << "Total mesh volume: " << grid_.vol_ << std::endl;

      VertexIter<Real> v_it = grid_.vertices_begin();
      VertexIter<Real> v_end = grid_.vertices_end();

      Real rho = 2.0;

      for(; v_it != v_end; v_it++) {

        int vidx = v_it.idx();

        int start = grid_.elementsAtVertexIdx_[vidx];
        int end = grid_.elementsAtVertexIdx_[vidx+1];

        std::cout << vidx << " = VertexId " << std::endl;
        std::cout << "Elements at vertex: " << end - start << std::endl;

        Real vertex_mass = 0.0;

        for (int i = start; i < end; i++) {

          int element_idx = grid_.elementsAtVertex_[i];
          vertex_mass += rho * grid_.elemVol_[element_idx]/8.0; 

        }

        std::cout << "Vertex mass: " << vertex_mass << std::endl;

        vmass.push_back(vertex_mass);

      }

      e_it = grid_.elem_begin();

      for(; e_it != grid_.elem_end(); e_it++) {

        Hexa &hexa = *e_it;

        hexa.shapeMatrix = Eigen::SparseMatrix<Real>(8,6);

        for (int j = 0; j < 6; ++j) {

          std::vector<Real> column = hexa.evalShapeFuncFace(j, 0.5, 0.5);
          for(int i(0); i < 8; ++i) {
            if(column[i] != 0.0)
              hexa.shapeMatrix.insert(i,j) = column[i];

            hexa.denseMatrix(i,j) = column[i]; 
          }
        }
        
        std::cout << hexa.denseMatrix << std::endl;
        for (int k=0; k<hexa.shapeMatrix.outerSize(); ++k)
          for (Eigen::SparseMatrix<Real>::InnerIterator it(hexa.shapeMatrix,k); it; ++it)
          {
            it.value();
            it.row();   // row index
            it.col();   // col index (here it is equal to k)
            it.index(); // inner index, here it is equal to it.row()
            std::cout << "(" << it.row() << "," << it.col() << ") = " << it.value() << std::endl;
          }

      }

      // Write out the computed result in VTK ParaView format 
      writeGrid(0);

    }

  };

}

int main()
{
  using namespace i3d;

  MeshTestCGAL myApp;

  myApp.init(std::string("start/sampleRigidBody.xml"));

  myApp.run();

  myApp.makeFaceTest();

  return EXIT_SUCCESS;
}

//#include <application.h>
//#include <reader.h>
//#include <distancemap.h>
//#include <meshobject.h>
//#include <iostream>
//#include <fstream>
//#include <vtkwriter.h>
//#include <iomanip>
//#include <sstream>
//
///*
// * Includes for CGAL
// */
//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/AABB_tree.h>
//#include <CGAL/AABB_traits.h>
//#include <CGAL/config.h>
//#include <CGAL/Polyhedron_3.h>
//
//#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
//#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
//#include <CGAL/AABB_face_graph_triangle_primitive.h>
//
//// Choose a geometry kernel
//typedef CGAL::Simple_cartesian<double> Kernel;
//
//typedef Kernel::Point_3 Point_3;
//
//typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
//
//typedef Polyhedron::Vertex_iterator Vertex_iterator;
//
//int main()
//{
//
//  Point_3 p( 1.0, 0.0, 0.0);
//  Point_3 q( 0.0, 1.0, 0.0);
//  Point_3 r( 0.0, 0.0, 1.0);
//  Point_3 s( 0.0, 0.0, 0.0);
//
//  Polyhedron P;
//
//  P.make_tetrahedron(p, q, r, s);
//
//  CGAL::set_ascii_mode(std::cout);
//
//  std::copy(P.points_begin(), P.points_end(), std::ostream_iterator<Point_3>(std::cout, "\n"));
//
//  return EXIT_SUCCESS;
//}
//