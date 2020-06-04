#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>
#include <OpenVolumeMesh/Geometry/VectorT.hh>

// Import eigen
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include<Eigen/SparseLU>

using SpMat = Eigen::SparseMatrix<ScalarType>;
using Triplet = Eigen::Triplet<ScalarType>;


template <typename T>
class MeshIntegratorImplicitEuler {

public:

  using VertexType = OpenVolumeMesh::Geometry::VectorT<T, 3>;

  T dt_;

  MeshIntegratorImplicitEuler() : dt_(1.0) {

  }

  void setDeltaT(T _dt) {
    dt_ = _dt;
  }

  void integrateInternal(HexMesh& mesh_)
  {
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
    posMatrix = posMatrix + dt_ * velocityMatrix;

//    std::cout << "Velocity Matrix: " << std::endl << velocityMatrix << std::endl;
//    std::cout << "Position Matrix: " << std::endl << posMatrix << std::endl;

    for (OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      
      velocityProp[*v_it] = VertexType(velocityMatrix.coeffRef(idx, idx),
                                       velocityMatrix.coeffRef(idx + 1, idx + 1),
                                       velocityMatrix.coeffRef(idx + 2, idx + 2));

      mesh_.set_vertex(*v_it, VertexType(posMatrix.coeffRef(idx, idx),
                                         posMatrix.coeffRef(idx + 1, idx + 1),
                                         posMatrix.coeffRef(idx + 2, idx + 2)));
      
    }

  }

  void interateMat(HexMesh& mesh_) {

    OpenVolumeMesh::VertexPropertyT<T> massProp =
    mesh_.request_vertex_property<T>("mass");

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
    for(OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      ScalarType v_ij = massProp[*v_it];

      tripletList.push_back(Triplet(idx, idx, v_ij));
      tripletList.push_back(Triplet(idx + 1, idx + 1, v_ij));
      tripletList.push_back(Triplet(idx + 2, idx + 2, v_ij));
      
    }
    SpMat MassMatrix(matrixSize, matrixSize);
    MassMatrix.setFromTriplets(tripletList.begin(), tripletList.end());

    tripletList.clear();
    for(OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      ScalarType v_ij = 1.0;

      tripletList.push_back(Triplet(idx, idx, v_ij));
      tripletList.push_back(Triplet(idx + 1, idx + 1, v_ij));
      tripletList.push_back(Triplet(idx + 2, idx + 2, v_ij));
      
    }
    SpMat A(matrixSize, matrixSize);
    A.setFromTriplets(tripletList.begin(), tripletList.end());

    Eigen::VectorXd fA(matrixSize);
    for(OpenVolumeMesh::VertexIter v_it = mesh_.vertices_begin(); v_it != mesh_.vertices_end(); ++v_it) {

      int idx = 3 * v_it->idx();
      ScalarType v_ij = massProp[*v_it];

      fA(idx) = -9.81;
      fA(idx + 1) = -9.81;
      fA(idx + 2) = -9.81;
      
    }

    fA = MassMatrix * fA;
    //std::cout << "Vector FA: " << std::endl << fA << std::endl;

    Eigen::VectorXd b(matrixSize);

    b = dt_ * invMassMatrix * fA;
    //std::cout << "Vector b: " << std::endl << b << std::endl;
    //std::cout << "Vector b2: " << std::endl << dt_ * invMassMatrix * fA << std::endl;

//    SolverClassName<SparseMatrix<double> > solver;
//    solver.compute(A);

    Eigen::SparseLU<SpMat> mySolver;
    mySolver.compute(A);
    std::cout << "dV = " << std::endl << mySolver.solve(b) << std::endl;
  }
};