#pragma once
#include <vector>

#include "general_definitions.hpp"
#include <ostream>
#include <iostream>

namespace i3d {

template <typename T>
class PBDBody {
public:

  typedef typename T ScalarType;
  typedef OpenMesh::VectorT<ScalarType, 3> VectorType;
  MyMesh* mesh_;

  std::vector< BendingConstraint<ScalarType> > bendingConstraints_;
  std::vector< DistanceConstraint<ScalarType> > distanceConstraints_;

  std::vector<VectorType> velocities_;
  std::vector<VectorType> forces_;

  std::vector<ScalarType> weights_;
  ScalarType mass = 1.0 / 4.0;

  void setMass(ScalarType _mass) { mass = _mass; };
  ScalarType getMass() { return mass;};

  void writeMesh(const std::string &fileName) {

    try {
      if (!OpenMesh::IO::write_mesh(*mesh_, fileName)) {
        std::cerr << "Cannot write mesh to file '"<< fileName << "'" << std::endl;
        std::exit(EXIT_FAILURE);
      }
      std::cout << "Mesh written to '" << fileName << "'" << std::endl;
    }
    catch( std::exception &x) {
      std::cerr << x.what() << std::endl;
      std::exit(EXIT_FAILURE);
    }

  }

};

typedef PBDBody<double> PBDBodyd;

}

