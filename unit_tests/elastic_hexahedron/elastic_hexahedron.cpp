// C++ includes
#include <iostream>
#include <vector>
#include <map>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>

// Attributes
//#include <OpenVolumeMesh/Attribs/NormalAttrib.hh>

// Include polyhedral mesh kernel
#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <boost/optional.hpp>
#include <CGAL/Surface_mesh.h>

// Import eigen
#include <Eigen/Dense>

#include "deformation_axes.hpp"

typedef CGAL::Simple_cartesian<double> K;

typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef K::Vector_3 cgalVec;

typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;

typedef CGAL::Surface_mesh<Point> Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;

// Make some typedefs to facilitate your life
typedef double ScalarType;
typedef OpenVolumeMesh::Geometry::VectorT<ScalarType, 3> VertexType;
typedef OpenVolumeMesh::GeometryKernel<VertexType>   HexMesh;
//typedef OpenVolumeMesh::NormalAttrib<HexMesh> HexFaceNormals;

using Eigen::MatrixXd;
typedef Eigen::Matrix<ScalarType, 8, 6> HexaMatrix ;

DeformationAxis<ScalarType> zeta[3];

struct Skip {
  face_descriptor fd;
  Skip(const face_descriptor fd)
    : fd(fd)
  {}
  bool operator()(const face_descriptor& t) const
  {
    if (t == fd) {
      std::cerr << "ignore " << t << std::endl;
    };
    return(t == fd);
  }

};


template <typename T>
class ElasticHexahedron {

public:

  typedef OpenVolumeMesh::Geometry::VectorT<T, 3> VertexType;

  OpenVolumeMesh::CellHandle cellHandle_;

  DeformationAxis<ScalarType> zeta_[3];

  HexaMatrix mat_;

  std::map<int, int> vertexMap_;

  void calculateGlobal2LocalMap(HexMesh &myMesh) {

    OpenVolumeMesh::CellIter c_it = OpenVolumeMesh::CellIter(&myMesh, cellHandle_);

    OpenVolumeMesh::CellVertexIter cv_it(*c_it, &myMesh);
    for (int i(0); cv_it.valid(); ++cv_it, ++i) {
      // The global vertex idx
      int idx = cv_it->idx();
      // The vertexMap maps the global idx to the local
      vertexMap_[idx] = i;
    }

  }

  ScalarType evaluateLocalBasisFunction(ScalarType xi, ScalarType eta, int vidx) {
    if (vidx == 0) return (1.0 - xi) * (1.0 - eta);
    else if (vidx == 1) return xi * (1.0 - eta);
    else if (vidx == 2) return xi * eta;
    else if (vidx == 3) return (1.0 - xi) * eta;
    else
      return 0.0;
  }

  void calculateCoefficientMatrix(HexMesh &myMesh) {

    mat_ = HexaMatrix::Zero();

    for (int idx(0); idx < 3; ++idx) {
      int col = idx * 2;

      OpenVolumeMesh::FaceHandle fh0 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[0]);

      OpenVolumeMesh::FaceIter f_it(&myMesh, fh0);
      OpenVolumeMesh::FaceVertexIter fv_it(*f_it, &myMesh);

      // Get the local vertex indices of the first intersected face 
      std::vector<int> entriesIdx;
      for (; fv_it.valid(); ++fv_it) {
        int vidx = fv_it->idx();
        int midx = vertexMap_[vidx];
        entriesIdx.push_back(midx);
      }

      // We have the local vertex indices of the first face, 
      // now we can calculate the values of the basis functions
      for (int i(0); i < 4; ++i) {
        int j = entriesIdx[i];
        ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[0].first, zeta_[idx].parameters[0].second, i);
        mat_(j, col) = value;
      }

      col++;
      entriesIdx.clear();

      OpenVolumeMesh::FaceHandle fh1 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[1]);
      OpenVolumeMesh::FaceIter f_it1(&myMesh, fh1);
      OpenVolumeMesh::FaceVertexIter fv_it1(*f_it1, &myMesh);

      // Get the local vertex indices of the second intersected face 
      for (; fv_it1.valid(); ++fv_it1) {
        int vidx = fv_it1->idx();
        int midx = vertexMap_[vidx];
        entriesIdx.push_back(midx);
      }

      // We have the local vertex indices of the second face, 
      // now we can calculate the values of the basis functions
      for (int i(0); i < 4; ++i) {
        int j = entriesIdx[i];
        ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[1].first, zeta_[idx].parameters[1].second, i);
        mat_(j, col) = value;
      }

    }

  }

  ScalarType computeCellVolume(HexMesh &mesh) {

    OpenVolumeMesh::CellIter c_it = OpenVolumeMesh::CellIter(&mesh, cellHandle_); 

    std::vector<OpenVolumeMesh::VertexHandle> vertices;

    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &mesh);
    for (; cv_it.valid(); ++cv_it) {
      vertices.push_back(*cv_it);
    }

    // V = |(a-d)*((b-d)x(c-d))| * (1.0/6.0)
    int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
    ScalarType vol = ScalarType(0.0);
    for(int i(0); i < 5; ++i)
    {
      VertexType a = mesh.vertex(vertices[tetras[i][0]]);
      VertexType b = mesh.vertex(vertices[tetras[i][1]]);
      VertexType c = mesh.vertex(vertices[tetras[i][2]]);
      VertexType d = mesh.vertex(vertices[tetras[i][3]]);

      vol += std::abs(OpenVolumeMesh::dot( a-d, OpenVolumeMesh::cross((b-d), (c-d)))) * (1.0/6.0);

    }
    return vol;
  }

};


// std::cout << "xi: " << 2.0 * T1 / area << std::endl;
// std::cout << "eta: " <<  2.0 * T2 / area << std::endl;
ScalarType evaluateFaceBasisFunction(ScalarType xi, ScalarType eta, int vidx) {
  if(vidx == 0) return (1.0 - xi) * (1.0 - eta);
  if(vidx == 1) return xi * (1.0 - eta);
  if(vidx == 2) return xi * eta;
  if(vidx == 3) return (1.0 - xi) * eta;
}

ScalarType computeCellVolume(OpenVolumeMesh::CellIter &c_it, HexMesh &mesh) {

    std::vector<OpenVolumeMesh::VertexHandle> vertices;

    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &mesh);
    for (; cv_it.valid(); ++cv_it) {
      vertices.push_back(*cv_it);
    }

    // V = |(a-d)*((b-d)x(c-d))| * (1.0/6.0)
    int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
    ScalarType vol = ScalarType(0.0);
    for(int i(0); i < 5; ++i)
    {
      VertexType a = mesh.vertex(vertices[tetras[i][0]]);
      VertexType b = mesh.vertex(vertices[tetras[i][1]]);
      VertexType c = mesh.vertex(vertices[tetras[i][2]]);
      VertexType d = mesh.vertex(vertices[tetras[i][3]]);

      vol += std::abs(OpenVolumeMesh::dot( a-d, OpenVolumeMesh::cross((b-d), (c-d)))) * (1.0/6.0);

    }
    return vol;

}

void triangleIntersection(const VertexType& p0, const VertexType& p1, const VertexType& p2, 
                          const VertexType& p3, const VertexType &direction, int faceIdx, DeformationAxis<ScalarType> &axis) {

  Point a(p0[0], p0[1], p0[2]);
  Point b(p1[0], p1[1], p1[2]);
  Point c(p2[0], p2[1], p2[2]);
  Point d(p3[0], p3[1], p3[2]);

  std::list<Triangle> triangles;
  triangles.push_back(Triangle(a, b, c));
  triangles.push_back(Triangle(a, c, d));

  Tree tree(triangles.begin(), triangles.end());

  Point center(0, 0, 0);

  cgalVec dir(direction[0], direction[1], direction[2]);

  Ray ray_query(center, dir);

//  std::cout << tree.number_of_intersected_primitives(ray_query)
//    << " intersections(s) with ray " << std::endl;

  Ray_intersection intersection = tree.first_intersection(ray_query);

  const Point* p;;
  if (tree.number_of_intersected_primitives(ray_query)) {
    p = boost::get<Point>(&(intersection->first));
    if (p) {
      std::cout << "Intersection object is a point " << *p << std::endl;
      VertexType q(p->x(), p->y(), p->z());

      ScalarType area = (p0 - p1).norm() * (p0 - p3).norm();
      std::cout << "Surface area rect: " << area << std::endl;

      ScalarType T1 = 0.5 * OpenVolumeMesh::cross((q - p1), (p1 - p2)).norm();
      std::cout << "Surface area T1: " << T1 << std::endl;

      ScalarType T2 = 0.5 * OpenVolumeMesh::cross((q - p2), (p2 - p3)).norm();
      std::cout << "Surface area T2: " << T2 << std::endl;

      ScalarType xi = 2.0 * T1 / area;
      ScalarType eta = 2.0 * T2 / area;

      std::cout << "xi: " << 2.0 * T1 / area << std::endl;
      std::cout << "eta: " <<  2.0 * T2 / area << std::endl;

      axis.faceIndices.push_back(faceIdx);

      axis.parameters.push_back(std::make_pair(xi, eta));
    }

  }
  
}

int main(int _argc, char** _argv) {

  // Define 3 deformation axes of the hexahedron
  zeta[0].dir = VertexType(0,-1.0, 0);

  zeta[1].dir = VertexType(-1.0, 0.0, 0);

  zeta[2].dir = VertexType(0, 0.0,-1.0);

  // Create mesh object
  HexMesh myMesh;

  // Add eight vertices
  OpenVolumeMesh::VertexHandle v0 = myMesh.add_vertex(VertexType(-1.0,-1.0,-1.0));
  OpenVolumeMesh::VertexHandle v1 = myMesh.add_vertex(VertexType( 1.0,-1.0,-1.0));
  OpenVolumeMesh::VertexHandle v2 = myMesh.add_vertex(VertexType( 1.0, 1.0,-1.0));
  OpenVolumeMesh::VertexHandle v3 = myMesh.add_vertex(VertexType(-1.0, 1.0,-1.0));

  OpenVolumeMesh::VertexHandle v4 = myMesh.add_vertex(VertexType(-1.0,-1.0, 1.0));
  OpenVolumeMesh::VertexHandle v5 = myMesh.add_vertex(VertexType( 1.0,-1.0, 1.0));
  OpenVolumeMesh::VertexHandle v6 = myMesh.add_vertex(VertexType( 1.0, 1.0, 1.0));
  OpenVolumeMesh::VertexHandle v7 = myMesh.add_vertex(VertexType(-1.0, 1.0, 1.0));
  
  std::vector<OpenVolumeMesh::VertexHandle> vertices;
  
  // Add faces
  vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v2);vertices.push_back(v3);
  OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
  
  vertices.clear();
  vertices.push_back(v4); vertices.push_back(v5);vertices.push_back(v6);vertices.push_back(v7);
  OpenVolumeMesh::FaceHandle f1 = myMesh.add_face(vertices);
  
  vertices.clear();
  vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v5);vertices.push_back(v4);
  OpenVolumeMesh::FaceHandle f2 = myMesh.add_face(vertices);
  
  vertices.clear();
  vertices.push_back(v1); vertices.push_back(v2);vertices.push_back(v6);vertices.push_back(v5);
  OpenVolumeMesh::FaceHandle f3 = myMesh.add_face(vertices);
  
  vertices.clear();
  vertices.push_back(v2); vertices.push_back(v3);vertices.push_back(v7);vertices.push_back(v6);
  OpenVolumeMesh::FaceHandle f4 = myMesh.add_face(vertices);

  vertices.clear();
  vertices.push_back(v3); vertices.push_back(v0);vertices.push_back(v4);vertices.push_back(v7);
  OpenVolumeMesh::FaceHandle f5 = myMesh.add_face(vertices);
  
  std::vector<OpenVolumeMesh::HalfFaceHandle> halffaces;
  
  // Add faces  
  halffaces.push_back(myMesh.halfface_handle(f0, 1));
  halffaces.push_back(myMesh.halfface_handle(f1, 1));
  halffaces.push_back(myMesh.halfface_handle(f2, 0)); 
  halffaces.push_back(myMesh.halfface_handle(f3, 1)); 

  halffaces.push_back(myMesh.halfface_handle(f4, 0)); 
  halffaces.push_back(myMesh.halfface_handle(f5, 1)); 

  // Add a cell  
  myMesh.add_cell(halffaces);

  VertexType baryCenter(0,0,0);

  // Print positions of vertices to std out
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
          v_it != myMesh.vertices_end(); ++v_it) {

      std::cout << "Position of vertex " << v_it->idx() << ": " <<
          myMesh.vertex(*v_it) << std::endl;

  }
  
  std::cout << "Number of cells in the mesh " << myMesh.n_cells() << std::endl;

  OpenVolumeMesh::CellIter c_it = myMesh.cells_begin();
  for (; c_it != myMesh.cells_end(); ++c_it) {
    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &myMesh);
    for (; cv_it.valid(); ++cv_it) {
      baryCenter += myMesh.vertex(*cv_it);
      std::cout << "Vertex idx " << cv_it->idx() << " in Cell " << (*c_it).idx() << ")" << std::endl;
    }
    baryCenter *= 0.125;
    std::cout << "Barycenter of cell (" << (*c_it).idx() << ") :" << baryCenter << std::endl;
  }

  c_it = myMesh.cells_begin();
  for (; c_it != myMesh.cells_end(); ++c_it) {
    OpenVolumeMesh::CellFaceIter cf_it((*c_it), &myMesh);
    for (; cf_it.valid(); ++cf_it) {
      std::cout << "Cell (" << (*c_it).idx() << ") has face:" << cf_it->idx() << std::endl;

      std::vector<VertexType> vertices;
      OpenVolumeMesh::FaceVertexIter fv_it((*cf_it), &myMesh);
      for (; fv_it.valid(); ++fv_it) {
        std::cout << "   face (" << (*cf_it).idx() << ") has vertex:" << fv_it->idx() << std::endl;
        vertices.push_back(myMesh.vertex(*fv_it));
      }

      VertexType q0 = vertices[1] - vertices[0];
      VertexType q1 = vertices[2] - vertices[0];

      VertexType faceMid = 0.25 * (vertices[0] + vertices[1] + vertices[2] + vertices[3]);

      VertexType n = OpenVolumeMesh::cross(q0, q1);
      VertexType ndir = faceMid + n;
      n.normalize();

      //std::cout << "Dot prot: " << OpenVolumeMesh::dot(ndir, vertices[0]) << std::endl;
      if (OpenVolumeMesh::dot(ndir, vertices[0]) < 0) {
        n = -1.0 * n;
      }
      std::cout << "normal: " << n << std::endl;

      for (int idx(0); idx < 3; ++idx) {
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], zeta[idx].dir, cf_it->idx(), zeta[idx]);
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], -zeta[idx].dir, cf_it->idx(), zeta[idx]);
      }

    }

  }

  c_it = myMesh.cells_begin();
  ScalarType vol = computeCellVolume(c_it, myMesh);

  std::vector<OpenVolumeMesh::VertexHandle> vertices1;
  OpenVolumeMesh::CellIter cbegin = myMesh.cells_begin();

  OpenVolumeMesh::CellVertexIter cv_it((*cbegin), &myMesh);
  for (; cv_it.valid(); ++cv_it) {
    vertices1.push_back(*cv_it);
  }

  int vertexMap[8];
  for (int i = 0; i < 8; ++i) {
    // The global vertex idx
    int idx = vertices1[i].idx();
    // The vertexMap maps the global idx to the local
    vertexMap[idx] = i;
  }

  HexaMatrix mat = HexaMatrix::Zero();

  mat(0, 0) =  0;

  std::cout << "Result-----------------------------------------------------" << std::endl;
  for (int idx(0); idx < 3; ++idx) {
    int col = idx * 2;
    std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[0] << std::endl;

    OpenVolumeMesh::FaceHandle fh0 = OpenVolumeMesh::FaceHandle(zeta[idx].faceIndices[0]);

    OpenVolumeMesh::FaceIter f_it(&myMesh, fh0);
    OpenVolumeMesh::FaceVertexIter fv_it(*f_it, &myMesh);

    std::vector<int> entriesIdx;
    for (; fv_it.valid(); ++fv_it) {
      int vidx = fv_it->idx();
      int midx = vertexMap[vidx];
      entriesIdx.push_back(midx);
    }

    for (int i(0); i < 4; ++i) {
      int j = entriesIdx[i];
      ScalarType value = evaluateFaceBasisFunction(zeta[idx].parameters[0].first, zeta[idx].parameters[0].second, i);
      //mat(j, col) = col + 1;
      mat(j, col) = value;
    }

    col++;
    entriesIdx.clear();

    std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[1] << std::endl;

    OpenVolumeMesh::FaceHandle fh1 = OpenVolumeMesh::FaceHandle(zeta[idx].faceIndices[1]);
    OpenVolumeMesh::FaceIter f_it1(&myMesh, fh1);
    OpenVolumeMesh::FaceVertexIter fv_it1(*f_it1, &myMesh);
    for (; fv_it1.valid(); ++fv_it1) {
      int vidx = fv_it1->idx();
      int midx = vertexMap[vidx];
      entriesIdx.push_back(midx);
    }

    for (int i(0); i < 4; ++i) {
      int j = entriesIdx[i];
      ScalarType value = evaluateFaceBasisFunction(zeta[idx].parameters[1].first, zeta[idx].parameters[1].second, i);
      //mat(j, col) = col + 1;
      mat(j, col) = value;
    }

    std::cout << "Parameters [" << zeta[idx].parameters[0].first << ", " << zeta[idx].parameters[0].second << "]" << std::endl;
    std::cout << "Parameters [" << zeta[idx].parameters[1].first << ", " << zeta[idx].parameters[1].second << "]" << std::endl;
    std::cout << "Cell Volume = [" << vol << "]" << std::endl;
    std::cout << "----------------------------------------------------------- " << std::endl;
  }
  std::cout << mat << std::endl;
  std::cout << "----------------------------------------------------------- " << std::endl;

  ElasticHexahedron<ScalarType> hex;
  hex.zeta_[0] = zeta[0];
  hex.zeta_[1] = zeta[1];
  hex.zeta_[2] = zeta[2];
  hex.cellHandle_ = *(myMesh.cells_begin());

  hex.calculateGlobal2LocalMap(myMesh);
//  for (auto& x : hex.vertexMap_) {
//    std::cout << x.first << "," << x.second << std::endl;
//  }
  hex.calculateCoefficientMatrix(myMesh);
  std::cout << hex.mat_ << std::endl;
  std::cout << "----------------------------------------------------------- " << std::endl;
  std::cout << "Cell Volume = [" << hex.computeCellVolume(myMesh) << "]" << std::endl;
  

  return 0;
}
