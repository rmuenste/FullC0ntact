/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#ifndef MESHOBJECT_H
#define MESHOBJECT_H

#include <shape.h>
#include <3dmodel.h>
#include <distancefuncgridmodel.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <string>
#include <transform.h>
#include <geom_config.hpp>

namespace i3d {

/** @brief A shape object that is described by a triangular mesh
 *
 * A shape object that is described by a triangular mesh
 */  
template<class T, int geomKernel = 0>
class MeshObject : public Shape<T>
{

public:

  MeshObject(void);

  MeshObject(const char* strFilename);

  MeshObject(bool useMeshFiles, std::vector<std::string> &meshNames) : useMeshFiles_(useMeshFiles), meshFiles_(meshNames) {

    if (useMeshFiles_)
    {
      m_Model.setHasSubMeshes(true);
      m_Model.setMeshFiles(meshFiles_);
    }

  };

  ~MeshObject(void);
  
  T getVolume() const;

  AABB3<T> getAABB() 
  {
    return m_Model.getBox();
  }

  void generateBoundingBox() 
  {
    m_Model.generateBoundingBox();
  }

  bool isPointInside(const Vector3<T> &vQuery) const
  {
    std::cout << "NEIN" << std::endl;
    std::exit(EXIT_FAILURE);
    DistanceFuncGridModel<T> distFunc;
    if(distFunc.BruteForceInnerPointsStatic(m_Model,vQuery)==1)
      return true;
    else
      return false;
  }      

  void setFileName(const char* strFileName)
  {
    m_sFileName=std::string(strFileName);
  };

  void updateMeshStructures(const Transformationr &tf)
  {

    Model3D model_out(m_Model);

    for (auto &subMesh : model_out.meshes_)
    {
      subMesh.transform_ = tf.getMatrix();
      subMesh.com_ = tf.getOrigin();
      subMesh.TransformModelWorld();
    }

    std::vector<Triangle3r> triangles = model_out.genTriangleVector();
    model_out.generateBoundingBox();

    CSubDivRessources myRessources(1, 4, 0, model_out.getBox(), &triangles);
    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
    m_BVH.DestroyAndRebuilt(&subdivider);

  };

  std::string getFileName() { return m_sFileName; };

  /**
    * Returns the geometric center of the shape
    *
    */
  Vector3<T> getCenter() const {return m_Model.box_.center_;};

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree(CSubdivisionCreator &sd) { m_BVH.InitTree(&sd); };

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree(const Transformationr &tf)
  {

    Model3D model_out(m_Model);

    for (auto &subMesh : model_out.meshes_)
    {
      subMesh.transform_ = tf.getMatrix();
      subMesh.com_ = tf.getOrigin();
      subMesh.TransformModelWorld();
    }

    std::vector<Triangle3r> triangles = model_out.genTriangleVector();
    model_out.generateBoundingBox();

    CSubDivRessources myRessources(1, 4, 0, model_out.getBox(), &triangles);
    CSubdivisionCreator subdivider = CSubdivisionCreator(&myRessources);
    m_BVH.InitTree(&subdivider);
  };

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree()
  {
    std::vector<Triangle3r> triangles = m_Model.genTriangleVector();
    CSubDivRessources myRessources_dm(1, 4, 0, m_Model.getBox(), &triangles);
    CSubdivisionCreator subdivider_dm = CSubdivisionCreator(&myRessources_dm);
    m_BVH.InitTree(&subdivider_dm);
  };

  /**
  *
  * Returns a reference to the 3d model data
  *
  */
  Model3D& getModel() { return m_Model; };

  /**
  *
  * Returns a pointer to the root bvh node
  *
  */
  CBoundingVolumeTree3<AABB3r, Real, CTraits, CSubdivisionCreator>& getBvhTree() { return m_BVH; };

  bool useMeshFiles_;

  std::vector<std::string> meshFiles_;
  
  private:

  std::string m_sFileName;

  Model3D m_Model;

  CBoundingVolumeTree3<AABB3r, Real, CTraits, CSubdivisionCreator> m_BVH;

};

typedef MeshObject<Real> MeshObjectr;

#ifdef WITH_CGAL

/** @brief A shape object that is described by a triangular mesh
*
* A shape object that is described by a triangular mesh
*
*/
template<class T>
class MeshObject<T, cgalKernel> : public Shape<T>
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
  typedef CGAL::AABB_tree<Traits>::Bounding_box theBoundingBox;
  typedef Tree::Point_and_primitive_id Point_and_primitive_id;

  Model3D m_Model;

  std::vector<Tree*> trees_;

  std::vector<Polyhedron*> polyhedra_;

  bool useMeshFiles_;

  std::vector<std::string> meshFiles_;

  std::string m_sFileName;


  MeshObject(bool useMeshFiles, std::vector<std::string> &meshNames) : useMeshFiles_(useMeshFiles), meshFiles_(meshNames) {

  };

  Model3D& getModel()
  {
    // writes P to `out' in the format provided by `writer'.
    typedef typename Polyhedron::Vertex_const_iterator                  VCI;
    typedef typename Polyhedron::Facet_const_iterator                   FCI;
    typedef typename Polyhedron::Halfedge_around_facet_const_circulator HFCC;

    if (m_Model.meshes_.empty())
    {

      std::vector<Vector3<T>> model_vertices;
      std::vector<TriFace> model_faces;

      for (auto vi = polyhedra_[0]->vertices_begin(); vi != polyhedra_[0]->vertices_end(); ++vi)
      {
        model_vertices.push_back(Vector3<T>(vi->point().x(), vi->point().y(), vi->point().z()));
      }

      typedef CGAL::Inverse_index<VCI> Index;
      Index index(polyhedra_[0]->vertices_begin(), polyhedra_[0]->vertices_end());

      for (FCI fi = polyhedra_[0]->facets_begin(); fi != polyhedra_[0]->facets_end(); ++fi) {
        HFCC hc = fi->facet_begin();
        HFCC hc_end = hc;
        std::size_t n = circulator_size(hc);
        CGAL_assertion(n >= 3);
        TriFace face;
        unsigned idx = 0;
        do {
          int vertIdx = index[VCI(hc->vertex())];
          face.SetIndex(idx, vertIdx);
          ++hc;
          ++idx;
        } while (hc != hc_end);
        model_faces.push_back(face);
      }

      m_Model.createFrom(model_vertices, model_faces);

    }
    return m_Model;
  };

  std::string getFileName() { return m_sFileName; };

  /*
   * Function to convert the CGAL class Point to a Vec3
   */ 
  inline Vec3 pointToVec3(const Point &p)
  {
    return Vec3(p.x(), p.y(), p.z());
  }

  void initCgalMeshes(void)
  {

    for (auto &s : meshFiles_)
    {

      // Load a mesh from file in the CGAL off format
      std::ifstream in(s);

      if (!in)
      {
        std::cerr << "unable to open file: " << s << std::endl;
        std::exit(EXIT_FAILURE);
      }

      Polyhedron *polyhedron = new Polyhedron();
      // Read the polyhedron from the stream
      in >> *polyhedron;

      if (!in)
      {
        std::cerr << "File: "  << s << " invalid OFF file" << std::endl;
        delete polyhedron;
        polyhedron = nullptr;
        std::exit(EXIT_FAILURE);
      }

      in.close();

      polyhedra_.push_back(polyhedron);
#ifdef CGAL_OUTPUT_DEBUG
      std::cout << "OFF file: " << s << " loaded successfully" << std::endl;
#endif 

    }

  };

  MeshObject(const char* strFilename)
  {

  };

  ~MeshObject(void)
  {

  };

  T getVolume() const
  {
    return T(1.0);
  };

  AABB3<T> getAABB()
  { 
    const theBoundingBox &box = trees_[0]->bbox(); 

    Vector3<T> vmin(box.xmin(), box.ymin(), box.zmin());
    Vector3<T> vmax(box.xmax(), box.ymax(), box.zmax());
    return AABB3<T>(vmin, vmax);
  }

  void generateBoundingBox()
  {

  }

  bool isPointInside(const Vector3<T> &vQuery) const
  {

    Vector vec = random_vector();

    Point p(vQuery.x, vQuery.y, vQuery.z);

    Ray ray(p, vec);

    for (auto &tree : trees_)
    {

      int nb_intersections = (int)tree->number_of_intersected_primitives(ray);

      // Check for odd or even number of intersections
      if (nb_intersections % 2 != 0)
        return true;

    }

    return false;

  }

  bool isPointInside(const Vector3<T> &vQuery, const Vector3<T> &vDir) const
  {

    Vector vec(vDir.x, vDir.y, vDir.z);

    Point p(vQuery.x, vQuery.y, vQuery.z);

    Ray ray(p, vec);

    for (auto &tree : trees_)
    {

      int nb_intersections = (int)tree->number_of_intersected_primitives(ray);

      // Check for odd or even number of intersections
      if (nb_intersections % 2 != 0)
        return true;

    }

    return false;

  }
  
  inline int triangleIntersectionQuery(const Vector3<T> &v0, const Vector3<T> &v1, const Vector3<T> &v2) const
  {

    Point p0(v0.x, v0.y, v0.z);
    Point p1(v1.x, v1.y, v1.z);
    Point p2(v2.x, v2.y, v2.z);

    for (auto &tree : trees_)
    {

      // counts #intersections with a triangle query
      Triangle triangle_query(p0, p1, p2);
      int intersections = tree->number_of_intersected_primitives(triangle_query);
//      std::cout << tree->number_of_intersected_primitives(triangle_query)
//        << " intersections(s) with triangle" << std::endl;
      if (intersections > 0)
        return intersections;

    }

    return 0;

  }

  T getMinimumDistance(const Vector3<T> &vQuery) const
  {

    Point p(vQuery.x, vQuery.y, vQuery.z);
    Point cp;
    Point nearestPoint;

    T dmin = std::numeric_limits<T>::max();

    for (auto &tree : trees_)
    {
      cp = tree->closest_point(p);
      Real dist = CGAL::squared_distance(p, cp);
      if (dist < dmin)
      {
        dmin = dist;
        nearestPoint = cp;
      }
    }

    return std::sqrt(dmin);

  }

  std::pair<Real, Vec3> getClosestPoint(const Vector3<T> &vQuery) const
  {

    Point p(vQuery.x, vQuery.y, vQuery.z);
    Point cp;
    Point nearestPoint;

    T dmin = std::numeric_limits<T>::max();

    for (auto &tree : trees_)
    {
      cp = tree->closest_point(p);
      Real dist = CGAL::squared_distance(p, cp);
      if (dist < dmin)
      {
        dmin = dist;
        nearestPoint = cp;
      }
    }

    return std::make_pair<Real, Vec3>(Real(std::sqrt(dmin)), Vec3(cp.x(), cp.y(), cp.z()));

  }

  void updateMeshStructures(const Transformationr &tf)
  {

  };

  /**
  *
  * Returns the geometric center of the shape
  *
  */
  Vector3<T> getCenter() const
  { 
    return Vector3<T>(0,0,0);
  };

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree(const Transformationr &tf)
  {

    std::cout << "Construct AABB tree...";

    // Construct an instance of the CGAL::AABB_tree<Traits> tree
    // from the polyhedron we have just read in
    for (auto &polyhedron : polyhedra_)
    {

      Tree *tree = new Tree(faces(*polyhedron).first, faces(*polyhedron).second, *polyhedron);

      // Use the acceleration method for distances
      tree->accelerate_distance_queries();

      tree->bbox();
      trees_.push_back(tree);

    }

    std::cout << "done." << std::endl;

  };

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree()
  {

    std::cout << "Construct AABB tree...";

    // Construct an instance of the CGAL::AABB_tree<Traits> tree
    // from the polyhedron we have just read in
    for (auto &polyhedron : polyhedra_)
    {

      Tree *tree = new Tree(faces(*polyhedron).first, faces(*polyhedron).second, *polyhedron);

      // Use the acceleration method for distances
      tree->accelerate_distance_queries();

      trees_.push_back(tree);

    }

    std::cout << "done." << std::endl;

  };

  void setFileName(const char* strFileName)
  {
    m_sFileName=std::string(strFileName);
  };

private:

  double random_in(const double a,
    const double b) const
  {
    double r = rand() / (double)RAND_MAX;
    return (double)(a + (b - a) * r);
  }

  /*
   * This function generates and returns a
   * random 3d vector with components x,y,z in [0,1]
   */
  Vector random_vector() const
  {
    double x = random_in(0.0, 1.0);
    double y = random_in(0.0, 1.0);
    double z = random_in(0.0, 1.0);
    return Vector(x, y, z);
  }

};

#endif



}

#endif // MESHOBJECT_H
