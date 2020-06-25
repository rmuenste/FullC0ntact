// C++ includes
#include <iostream>
#include <vector>
#include <map>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Attribs/NormalAttrib.hh>

// File Manager
#include <OpenVolumeMesh/FileManager/FileManager.hh>

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

#include <json.hpp>
#include <ovm_vtk_writer.hpp>

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
using ScalarType = double;
using VertexType = OpenVolumeMesh::Geometry::VectorT<ScalarType, 3>;
using HexMesh = OpenVolumeMesh::GeometryKernel<VertexType>;
using HexFaceNormals = OpenVolumeMesh::NormalAttrib<HexMesh>;

using Eigen::MatrixXd;
typedef Eigen::Matrix<ScalarType, 8, 6, Eigen::DontAlign> HexaMatrix ;
typedef Eigen::SparseMatrix<ScalarType> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<ScalarType> Triplet;
  
#include "deformation_axes.hpp"
#include "spring_model.hpp"

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
#include <elastic_hexahedron.hpp>

#include <elastic_mesh_init.hpp>
#include <explicit_euler_integrator.hpp>
#include <implicit_euler_integrator.hpp>

// file manager
#include <OpenVolumeMesh/FileManager/FileManager.hh>

VertexType Gravity;
ScalarType Density;
ScalarType Dt = 0.001;
ScalarType GlobalFriction = 0.98;
int MaxSteps = 1;
int OutputFreq = 10;

void readJsonFile() {

  using json = nlohmann::json;

  std::ifstream json_stream("elastic_hexahedron.json");

  json j;

  json_stream >> j;

  //std::cout << "Config: " << std::endl;
  //std::cout << "Gravity: " << j["Gravity"][2]  << " [m/s**2]" << std::endl;

  Gravity = VertexType(j["Gravity"][0], j["Gravity"][1], j["Gravity"][2]);
  Density = j["Density"];

  Dt = j["TimeStep"];
  MaxSteps = j["MaxSteps"];
  OutputFreq = j["OutputFreq"];

}

void ouputVelPos(HexMesh &mesh) {

  OpenVolumeMesh::VertexPropertyT<VertexType> velocityProp =
  mesh.request_vertex_property<VertexType>("velocity");

  OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
  for (; v_it != mesh.vertices_end(); ++v_it) {
    std::cout << "2) Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;
  }

  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
    v_it != mesh.vertices_end(); ++v_it) {
      std::cout << "Position of vertex " << v_it->idx() << ": " <<
          mesh.vertex(*v_it) << std::endl;

  }

}

void outputForce(HexMesh &mesh) {

  OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
  mesh.request_vertex_property<VertexType>("force");

  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
    v_it != mesh.vertices_end(); ++v_it) {
    std::cout << "Force of vertex: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
  }
}

void assembleMassMatrix(HexMesh &mesh) {

  OpenVolumeMesh::MeshPropertyT<SpMat> invMassMatrix =
    mesh.request_mesh_property<SpMat>("invmass");

  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
  mesh.request_vertex_property<ScalarType>("mass");

  OpenVolumeMesh::VertexPropertyT<std::string> fixedProp =
  mesh.request_vertex_property<std::string>("fixed");

  int matSize = 3 * mesh.n_vertices();
  *(invMassMatrix.begin()) = SpMat(matSize, matSize);

  std::vector<Triplet> tripletList;
  tripletList.reserve(matSize);

  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {

    int idx = 3 * v_it->idx();

    ScalarType v_ij;
    if(fixedProp[*v_it] != "xyz")
      v_ij = 1.0 / massProp[*v_it];
    else
      v_ij = 0.0;

    tripletList.push_back(Triplet(idx, idx, v_ij));
    tripletList.push_back(Triplet(idx + 1, idx + 1, v_ij));
    tripletList.push_back(Triplet(idx + 2, idx + 2, v_ij));
    
  }

  invMassMatrix.begin()->setFromTriplets(tripletList.begin(), tripletList.end());

}

// Applies gravity of 9.81 [m/s**2] as a force by f = m * a 
void applyGravityForce(HexMesh& mesh) {

  OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
  mesh.request_vertex_property<VertexType>("force");

  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
  mesh.request_vertex_property<ScalarType>("mass");

  OpenVolumeMesh::VertexIter v_it{ mesh.vertices_begin() };

  for (; v_it != mesh.vertices_end(); ++v_it) {
    forceProp[*v_it] += massProp[*v_it] * 1e3 * Gravity;;
    //std::cout << "Gravity Force: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
  }

}

// Applies the deformation force generated by the springs 
void applyDeformationForce(HexMesh& mesh) {

  OpenVolumeMesh::CellPropertyT<ElasticHexahedron<ScalarType>> elasticProp = 
  mesh.request_cell_property<ElasticHexahedron<ScalarType>>("elastic");

  OpenVolumeMesh::CellIter h_it = mesh.cells_begin();
  for(; h_it != mesh.cells_end(); ++h_it) {
    elasticProp[*h_it].calculateSpringForces(mesh);
  }

}

int main(int _argc, char** _argv) {

  readJsonFile();

  OpenVolumeMesh::IO::FileManager fileManager;

  // Create mesh object
  HexMesh myMesh;
  //fileManager.readFile("mesh2x2x4.ovm", myMesh);
  fileManager.readFile("meshref2.ovm", myMesh);

//  loadMeshFromFile("mesh2x2x4.dat", myMesh);

  // Define 3 deformation axes of the hexahedron
  // -Y
  zeta[0].dir = VertexType(0,-1.0, 0);

  // -X
  zeta[1].dir = VertexType(-1.0, 0.0, 0);

  // -Z
  zeta[2].dir = VertexType(0, 0.0,-1.0);

  initHexMesh(myMesh);

  //========================================================================================
  //                              Set up persistent mesh properties
  //========================================================================================
  OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
  myMesh.request_vertex_property<VertexType>("force");

  myMesh.set_persistent(forceProp, true);

  OpenVolumeMesh::VertexPropertyT<VertexType> velocityProp =
  myMesh.request_vertex_property<VertexType>("velocity");

  myMesh.set_persistent(velocityProp, true);

  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
  myMesh.request_vertex_property<ScalarType>("mass");

  myMesh.set_persistent(massProp, true);

  OpenVolumeMesh::VertexPropertyT<std::string> fixedProp =
  myMesh.request_vertex_property<std::string>("fixed");

  myMesh.set_persistent(fixedProp, true);

  OpenVolumeMesh::CellPropertyT<ScalarType> volProp =
  myMesh.request_cell_property<ScalarType>("volume");

  myMesh.set_persistent(volProp, true);

  OpenVolumeMesh::CellPropertyT<ElasticHexahedron<ScalarType>> elasticProp = 
  myMesh.request_cell_property<ElasticHexahedron<ScalarType>>("elastic");

  myMesh.set_persistent(volProp, true);

  OpenVolumeMesh::MeshPropertyT<SpMat> invMassMatrix =
    myMesh.request_mesh_property<SpMat>("invmass");

  myMesh.set_persistent(invMassMatrix, true);

  //========================================================================================
  //                           Initialize force and velocity
  //========================================================================================
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
    forceProp[*v_it] = VertexType(0, 0, 0);
    velocityProp[*v_it] = VertexType(0, 0, 0);
    if (v_it->idx() > 3) {
      //fixedProp[*v_it] = "xyz";
      fixedProp[*v_it] = "";
    }
    else {
      fixedProp[*v_it] = "";
    }

      std::cout << "Position of vertex " << v_it->idx() << ": " <<
          myMesh.vertex(*v_it) << std::endl;
#ifdef VERBOSE_DEBUG
#endif

  }
  
#ifdef VERBOSE_DEBUG
  std::cout << "Number of cells in the mesh " << myMesh.n_cells() << std::endl;
#endif

  //========================================================================================
  //                               Set Deformation Axes
  //========================================================================================
  setupDeformationAxes(myMesh, zeta);

  //AxesIntersector<ScalarType, HexMesh> intersector(myMesh); 

  //========================================================================================
  //                          Compute Cell Volume and Particle Masses 
  //========================================================================================
  computeAllCellVolumes(myMesh);

  calculateAllParticleMasses(myMesh, Density);

  assembleMassMatrix(myMesh);

#ifdef VERBOSE_DEBUG
    std::cout << "Mass Matrix: " << std::endl << invMassMatrix[0] << std::endl;
//    std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[1] << std::endl;
#endif

#ifdef VERBOSE_DEBUG
    std::cout << "Parameters [" << zeta[idx].parameters[0].first << ", " << zeta[idx].parameters[0].second << "]" << std::endl;
    std::cout << "Parameters [" << zeta[idx].parameters[1].first << ", " << zeta[idx].parameters[1].second << "]" << std::endl;
    std::cout << "Cell Volume = [" << vol << "]" << std::endl;
    std::cout << "----------------------------------------------------------- " << std::endl;
#endif

//========================================================================================
//                   Get an elastic hexahedron and make set it as a cell property
//========================================================================================

  OpenVolumeMesh::CellIter h_it = myMesh.cells_begin();
  for (; h_it != myMesh.cells_end(); ++h_it) {
    ElasticHexahedron<ScalarType> hex;
    hex.zeta_[0] = zeta[0];
    hex.zeta_[1] = zeta[1];
    hex.zeta_[2] = zeta[2];
    hex.cellHandle_ = *(myMesh.cells_begin());

    // Set up the vertex map
    hex.calculateGlobal2LocalMap(myMesh);

    // Calculate the "C"-Matrix of the Hexahedron
    hex.calculateCoefficientMatrix(myMesh);

#ifdef VERBOSE_DEBUG
  std::cout << hex.mat_ << std::endl;
  std::cout << "-----------------------------------------------------------" << std::endl;
#endif

    //========================================================================================
    //                Calculate basic properties of the elastic hexahedron
    //========================================================================================
    hex.printCellData();

    hex.calculateInitialValues(myMesh);

    hex.updateFaceArea(myMesh);

    elasticProp[*h_it] = hex;
  }

#ifdef VERBOSE_DEBUG

  // Print positions of vertices to std out
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
//    std::cout << "Force of vertex: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
    std::cout << "Mass of vertex: " << v_it->idx() << " = " << massProp[*v_it]  << " [kg]" << std::endl;
  }

  for(OpenVolumeMesh::CellIter c_it = myMesh.cells_begin(); c_it != myMesh.cells_end(); ++c_it) {
    std::cout << "Volume of cell: " << c_it->idx() << " = " << volProp[*c_it]  << " [m**3]" << std::endl;
  }

  for(; h_it != myMesh.cells_end(); ++h_it) {
    std::cout << "Cell: " << h_it->idx() << std::endl;
    std::cout << "Alpha : " << elasticProp[*h_it].restLengths[2] << std::endl;
  }
  std::cout << "------------------Cell Data-----------------" << std::endl;

#endif

#ifdef VERBOSE_DEBUG
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
    std::cout << "Force of vertex: " << v_it->idx() << " = " << forceProp[*v_it]  << " [N]" << std::endl;
  }
#endif

#ifdef VERBOSE_DEBUG
  for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
    v_it != myMesh.vertices_end(); ++v_it) {
    std::cout << "Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;
  }
#endif

//========================================================================================
//                          Integrate using Explicit Euler
//========================================================================================
  MeshIntegratorExplicitEuler<ScalarType> integratorEE;

  ScalarType dt = Dt;
  ScalarType globalFriction = GlobalFriction;
  //int maxSteps = MaxSteps;
  int maxSteps = 1;
  integratorEE.setDeltaT(dt);
  int outputFreq = OutputFreq;

  OpenVolMeshVtkWriter vtkWriter;
  vtkWriter.writeUnstructuredVTK(myMesh, "name", 0);
  return 0;

  for (int istep(0); istep <= maxSteps; ++istep) {
    //========================================================================================
    //                               Force Calculation
    //========================================================================================
    ScalarType time = istep * dt;
    std::cout << "|============================Time step: " << std::setw(2) << istep << " Time: " << std::fixed << std::setw(3) << std::setprecision(2) << istep * dt << "[s]============================|" << std::endl;
    
    // Add gravity force
    applyGravityForce(myMesh);
//    std::cout << "Gravity Contribution:" << std::endl;
//    outputForce(myMesh);

    // Add deformation forces
//    std::cout << "Deformation Contribution:" << std::endl;
    applyDeformationForce(myMesh);

  //========================================================================================
  //                        Calculation of the Jacobian
  //========================================================================================
//#define DO_JACOBIAN
//#ifdef DO_JACOBIAN
//
//    MeshIntegratorImplicitEuler<ScalarType> integratorIE;
//    integratorIE.setDeltaT(dt);
//    integratorIE.interateMat(myMesh);
//
//    // Get a new property for the jacobian
//    OpenVolumeMesh::VertexPropertyT<VertexType> forceD =
//    myMesh.request_vertex_property<VertexType>("forceD");
//    for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
//      v_it != myMesh.vertices_end(); ++v_it) {
//      forceD[*v_it] = VertexType(0, 0, 0);
//    }
//
//    ScalarType delta = 1e-4;
//    for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin(); v_it != myMesh.vertices_end(); ++v_it) {
//
//      std::cout << "Vertex: " << v_it->idx() << " = " << myMesh.vertex(*v_it) << std::endl;
//      OpenVolumeMesh::VertexHandle vh0 = *v_it;
//
//      OpenVolumeMesh::VertexCellIter vc_it(*v_it, &myMesh);
//      for (; vc_it.valid(); ++vc_it) {
//
//          OpenVolumeMesh::VertexHandle vhA = *v_it;
//          elasticProp[*vc_it].calculateDerSpringForces(myMesh, vhA, delta);
//      }
//    }
//
//    VertexType du = delta * VertexType(1, 1, 1);
//    for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin(); v_it != myMesh.vertices_end(); ++v_it) {
//      //std::cout << "F(d): " << v_it->idx() << " = " << forceD[*v_it]  << " [N]" << std::endl;
//      std::cout << "[F(u) - F(d)]/du: " << v_it->idx() << " = " << (forceProp[*v_it] -  forceD[*v_it]) * 1.0/du  << " [N]" << std::endl;
//    }
//#endif

    // Integration step with EE
    integratorEE.integrateMat(myMesh, globalFriction);

    // Update the Axes/Intersection Points
    h_it = myMesh.cells_begin();
    for (; h_it != myMesh.cells_end(); ++h_it) {
      elasticProp[*h_it].updateIntersectionPoints(myMesh);
    }

    vtkWriter.writeUnstructuredVTK(myMesh, "name", istep);
    if (istep%outputFreq == 0) {
      calculateLength(myMesh, time);
    }

  }

//========================================================================================
//                         Calculate Jacobian of deformation force
//========================================================================================
  vtkWriter.writeHexMesh(myMesh, "hexmesh", 0);
//========================================================================================
//                         Calculate Jacobian of deformation force
//========================================================================================
  // This is done in two loops:
  //  for x in positions:
  //    x* = x + du  
  //    foreach element connected to x:
  //      F(x*) += calculateDeformationForceAt(x*, element)
  //    Jx = [F(x*) - F(x)]/du 
  //    J = InsertToSystemJacobian(Jx, J) 

  return 0;
}
