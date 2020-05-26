// C++ includes
#include <iostream>
#include <vector>
#include <map>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Attribs/NormalAttrib.hh>

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
#include <Eigen/Sparse>

#include "deformation_axes.hpp"

#include "spring_model.hpp"

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
typedef OpenVolumeMesh::NormalAttrib<HexMesh> HexFaceNormals;

using Eigen::MatrixXd;
typedef Eigen::Matrix<ScalarType, 8, 6> HexaMatrix ;
typedef Eigen::SparseMatrix<ScalarType> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<ScalarType> Triplet;

//std::vector<T> tripletList;
//tripletList.reserve(estimation_of_entries);
//for(...)
//{
//  // ...
//  tripletList.push_back(T(i,j,v_ij));
//}
//SparseMatrixType mat(rows,cols);
//mat.setFromTriplets(tripletList.begin(), tripletList.end());

#include <vector>
#include <iostream>
 

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

#include <hexahedron_element.hpp>

#include "elastic_hexahedron.h"


#include <elastic_mesh_init.hpp>
#include <explicit_euler_integrator.hpp>

// Applies gravity of 9.81 [m/s**2] as a force by f = m * a 
void applyGravityForce(HexMesh& mesh) {

  OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
  mesh.request_vertex_property<VertexType>("force");

  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
  mesh.request_vertex_property<ScalarType>("mass");

  OpenVolumeMesh::VertexIter v_it{ mesh.vertices_begin() };

  for (; v_it != mesh.vertices_end(); ++v_it) {

    forceProp[*v_it] += massProp[*v_it] * VertexType(0, 0, -9.81);

  }

}

int main(int _argc, char** _argv) {

  // Create mesh object
  HexMesh myMesh;

  // Define 3 deformation axes of the hexahedron
  // -Y
  zeta[0].dir = VertexType(0,-1.0, 0);

  // -X
  zeta[1].dir = VertexType(-1.0, 0.0, 0);

  // -Z
  zeta[2].dir = VertexType(0, 0.0,-1.0);

  initHexMesh(myMesh);

  OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
  myMesh.request_vertex_property<VertexType>("force");

  myMesh.set_persistent(forceProp, true);

  OpenVolumeMesh::VertexPropertyT<VertexType> velocityProp =
  myMesh.request_vertex_property<VertexType>("velocity");

  myMesh.set_persistent(velocityProp, true);

  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
  myMesh.request_vertex_property<ScalarType>("mass");

  myMesh.set_persistent(massProp, true);

  OpenVolumeMesh::CellPropertyT<ScalarType> volProp =
  myMesh.request_cell_property<ScalarType>("volume");

  myMesh.set_persistent(volProp, true);

  // Print positions of vertices to std out
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
    forceProp[*v_it] = VertexType(0, 0, 0);
    velocityProp[*v_it] = VertexType(0, 0, 0);
//      std::cout << "Position of vertex " << v_it->idx() << ": " <<
//          myMesh.vertex(*v_it) << std::endl;

  }
  
  std::cout << "Number of cells in the mesh " << myMesh.n_cells() << std::endl;

  setupDeformationAxes(myMesh, zeta);

  OpenVolumeMesh::CellIter c_it = myMesh.cells_begin();

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

  //std::cout << "Result-----------------------------------------------------" << std::endl;
  for (int idx(0); idx < 3; ++idx) {
    int col = idx * 2;
    //std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[0] << std::endl;

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

    //std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[1] << std::endl;

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

//    std::cout << "Parameters [" << zeta[idx].parameters[0].first << ", " << zeta[idx].parameters[0].second << "]" << std::endl;
//    std::cout << "Parameters [" << zeta[idx].parameters[1].first << ", " << zeta[idx].parameters[1].second << "]" << std::endl;
//    std::cout << "Cell Volume = [" << vol << "]" << std::endl;
//    std::cout << "----------------------------------------------------------- " << std::endl;
  }
  std::cout << "----------------------------------------------------------- " << std::endl;
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
  std::cout << "-----------------------------------------------------------" << std::endl;

//  std::cout << "Cell Volume = [" << hex.computeCellVolume(myMesh) << "]" << std::endl;
//  
//  std::cout << "----------------------------------------------------------- " << std::endl;
//  std::cout << "------------------Force Computation Example-----------------" << std::endl;
//  //std::cout << "Axis length = [" << zeta[0].parameters[0].first << "]" << std::endl;
//  LinearSpring<ScalarType> spring(ScalarType(1.0), hex.zeta_[0].getDir().length());
//  std::cout << "Linear Spring Force = [" << spring.evaluateForce(hex.zeta_[0].q[0], hex.zeta_[0].q[1]) << "]" << std::endl;
//  
//  ScalarType alphaZero = OpenVolumeMesh::dot(hex.zeta_[0].getDir(), hex.zeta_[1].getDir());
//  std::cout << "Linear Torsion Spring rest angle = [" << std::acos(alphaZero) << "]" << std::endl;
//
//  LinearTorsionSpring<ScalarType> torsionSpring(ScalarType(1.0), alphaZero);
//  std::pair<VertexType, VertexType> torsionForcePair = torsionSpring.evaluateTorsion(hex.zeta_[0].getDir(), hex.zeta_[1].getDir());
//  std::cout << "Linear Torsion Spring Force = [" << torsionForcePair.first << "]" << std::endl;

  hex.printCellData();

  hex.calculateInitialValues(myMesh);

  hex.updateFaceArea(myMesh);

  computeAllCellVolumes(myMesh);

  calculateAllParticleMasses(myMesh);

  std::cout << "------------------Cell Data-----------------" << std::endl;

  // Add gravity force
  applyGravityForce(myMesh);

  // Print positions of vertices to std out
//  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
//    v_it != myMesh.vertices_end(); ++v_it) {
//    std::cout << "Force of vertex: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
//    std::cout << "Mass of vertex: " << v_it->idx() << " = " << massProp[*v_it]  << " [kg]" << std::endl;
//  }
//
//  for(OpenVolumeMesh::CellIter c_it = myMesh.cells_begin(); c_it != myMesh.cells_end(); ++c_it) {
//    std::cout << "Volume of cell: " << c_it->idx() << " = " << volProp[*c_it]  << " [m**3]" << std::endl;
//  }

  hex.calculateSpringForces(myMesh);

  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
    std::cout << "Force of vertex: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
  }

  std::vector<Triplet> tripletList;
  int matrixSize = myMesh.n_vertices() * 3;
  tripletList.reserve(matrixSize);

  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin(); v_it != myMesh.vertices_end(); ++v_it) {

    int idx = 3 * v_it->idx();
    ScalarType v_ij = 1.0 / massProp[*v_it];

    tripletList.push_back(Triplet(idx, idx, v_ij));
    tripletList.push_back(Triplet(idx + 1, idx + 1, v_ij));
    tripletList.push_back(Triplet(idx + 2, idx + 2, v_ij));
    
  }
  SpMat invMassMatrix(matrixSize, matrixSize);
  invMassMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  tripletList.clear();
  for (OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin(); v_it != myMesh.vertices_end(); ++v_it) {

    int idx = 3 * v_it->idx();
    VertexType fxyz = forceProp[*v_it];

    tripletList.push_back(Triplet(idx, idx, fxyz[0]));
    tripletList.push_back(Triplet(idx + 1, idx + 1, fxyz[1]));
    tripletList.push_back(Triplet(idx + 2, idx + 2, fxyz[2]));
    
  }

  SpMat forceMatrix(matrixSize, matrixSize);
  forceMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

  MeshIntegratorExplicitEuler<ScalarType> integratorEE;

//  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
//    v_it != myMesh.vertices_end(); ++v_it) {
//    std::cout << "Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;
//  }

  ScalarType dt = 0.01;
  integratorEE.setDeltaT(dt);
  integratorEE.interateMat(myMesh);

  //integratorEE.interate(myMesh);

//  OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
//  for (; v_it != myMesh.vertices_end(); ++v_it) {
//    std::cout << "2) Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;
//  }

  return 0;
}
