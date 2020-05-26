#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>
#include <OpenVolumeMesh/Geometry/VectorT.hh>

// Import eigen
#include <Eigen/Sparse>

using SpMat = Eigen::SparseMatrix<ScalarType>;
using Triplet = Eigen::Triplet<ScalarType>;

template <typename T>
class MeshIntegratorExplicitEuler {

public:

  using VertexType = OpenVolumeMesh::Geometry::VectorT<T, 3>;

  T dt_;

  MeshIntegratorExplicitEuler() : dt_(1.0) {

  }

  void setDeltaT(T _dt) {
    dt_ = _dt;
  }

  void interateMat(HexMesh& mesh_) {

    OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
    mesh_.request_vertex_property<VertexType>("force");

    OpenVolumeMesh::VertexPropertyT<T> massProp =
    mesh_.request_vertex_property<T>("mass");

    OpenVolumeMesh::VertexPropertyT<VertexType> velocityProp =
      mesh_.request_vertex_property<VertexType>("velocity");

    std::vector<Triplet> tripletList;
    int matrixSize = mesh_.n_vertices() * 3;
    tripletList.reserve(matrixSize);

    for(OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      ScalarType v_ij = 1.0 / massProp[*v_it];

      tripletList.push_back(Triplet(idx, idx, v_ij));
      tripletList.push_back(Triplet(idx + 1, idx + 1, v_ij));
      tripletList.push_back(Triplet(idx + 2, idx + 2, v_ij));
      
    }
    SpMat invMassMatrix(matrixSize, matrixSize);
    invMassMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

    tripletList.clear();
    for (OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      VertexType fxyz = forceProp[*v_it];

      tripletList.push_back(Triplet(idx, idx, fxyz[0]));
      tripletList.push_back(Triplet(idx + 1, idx + 1, fxyz[1]));
      tripletList.push_back(Triplet(idx + 2, idx + 2, fxyz[2]));
      
    }

    SpMat forceMatrix(matrixSize, matrixSize);
    forceMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

    tripletList.clear();
    for (OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      VertexType fxyz = velocityProp[*v_it];

      tripletList.push_back(Triplet(idx, idx, fxyz[0]));
      tripletList.push_back(Triplet(idx + 1, idx + 1, fxyz[1]));
      tripletList.push_back(Triplet(idx + 2, idx + 2, fxyz[2]));
      
    }

    SpMat velocityMatrix(matrixSize, matrixSize);
    velocityMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

    tripletList.clear();
    for (OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      VertexType fxyz = mesh_.vertex(*v_it);

      tripletList.push_back(Triplet(idx, idx, fxyz[0]));
      tripletList.push_back(Triplet(idx + 1, idx + 1, fxyz[1]));
      tripletList.push_back(Triplet(idx + 2, idx + 2, fxyz[2]));
      
    }

    SpMat posMatrix(matrixSize, matrixSize);
    posMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

    velocityMatrix = velocityMatrix + dt_ * (invMassMatrix * forceMatrix);
    std::cout << "Velocity Matrix: " << std::endl << velocityMatrix << std::endl;

    std::cout << "Position Matrix: " << std::endl << posMatrix + dt_ * velocityMatrix << std::endl;

  }

  void interate(HexMesh& mesh_) {

    OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
    mesh_.request_vertex_property<VertexType>("force");

    OpenVolumeMesh::VertexPropertyT<T> massProp =
    mesh_.request_vertex_property<T>("mass");

    OpenVolumeMesh::VertexPropertyT<VertexType> velocityProp =
      mesh_.request_vertex_property<VertexType>("velocity");

    OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin();

    for (; v_it != mesh_.vertices_end(); ++v_it) {

//      std::cout << "Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;
      velocityProp[*v_it] += velocityProp[*v_it] + dt_ * forceProp[*v_it] / massProp[*v_it];
//      std::cout << "force of particles: " << v_it->idx() << " = " << forceProp[*v_it]  << " [m/s]" << std::endl;
//      std::cout << "mass of particles: " << v_it->idx() << " = " << massProp[*v_it]  << " [m/s]" << std::endl;
//      std::cout << "Velocity of particles: " << v_it->idx() << " = " << velocityProp[*v_it]  << " [m/s]" << std::endl;

    }

  }

};