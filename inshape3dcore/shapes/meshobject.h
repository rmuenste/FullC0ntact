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

#endif

namespace i3d {

  enum
  {
    defaultKernel,
    cgalKernel
  };

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
    CDistanceFuncGridModel<T> distFunc;
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

  std::vector<Tree*> trees_;
  std::vector<Polyhedron*> polyhedra_;

  MeshObject(void)
  {

//    std::string objPath = pMeshObject->getFileName();
//    std::string::size_type dpos = objPath.rfind(".");
//    std::string offPath = objPath;
//    offPath.replace(dpos + 1, 3, "off");
//
//    // Load a mesh from file in the CGAL off format
//    std::ifstream in(offPath);
//
//    if (!in)
//    {
//      std::cerr << "unable to open file" << std::endl;
//      std::exit(EXIT_FAILURE);
//    }
//
//    Polyhedron *polyhedron = new Polyhedron();
//    // Read the polyhedron from the stream
//    in >> *polyhedron;
//
//    if (!in)
//    {
//      std::cerr << "invalid OFF file" << std::endl;
//      delete polyhedron;
//      polyhedron = nullptr;
//      std::exit(EXIT_FAILURE);
//    }
//
//    in.close();
//
//    polyhedra.push_back(polyhedron);
//
//    std::cout << "OFF file loaded successfully" << std::endl;

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
    return AABB3<T>(Vector3<T>(0, 0, 0), T(1.0));
  }

  void generateBoundingBox()
  {

  }

  bool isPointInside(const Vector3<T> &vQuery) const
  {

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

  };

  /**
  *
  * Initialize the Tree data structure
  *
  */
  void initTree()
  {

  };

private:

};

#endif



}

#endif // MESHOBJECT_H
