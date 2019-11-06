#include "pbd_solver.hpp"
#include "pbdbody.hpp"

namespace i3d {

void PBDSolver::solve() {

  integrateWithDampening();

  solveInternalConstraints();

  verletIntegration();

}

void PBDSolver::integrateWithDampening() {

  typedef OpenMesh::Vec3d VertexType;
  VertexType cog(0, 0, 0), velCog(0, 0, 0);
  VertexType gravity(0.0, -0.00981, 0.0);
  Real globalDampening = 0.98;
  Real totalMass = 0.0;

  std::cout << "Velocities: " << std::endl;
  for (int idx(0); idx < body_->mesh_->n_vertices(); ++idx) {
//    body_->velocities_[idx] *= 0.98;
//    body_->velocities_[idx] = body_->velocities_[idx] + (gravity * dt_) * body_->weights_[idx];
    std::cout << "<" << idx << ">" << body_->velocities_[idx][0] << " " << body_->velocities_[idx][1] << " " <<body_->velocities_[idx][2] << std::endl;

    cog += body_->mesh_->point(MyMesh::VertexHandle(idx)) * body_->mass;
    velCog += body_->velocities_[idx] * body_->mass;

    totalMass += body_->mass;
  }

  cog /= totalMass;
  velCog /= totalMass;

  calculatePositionPrediction();

}

void PBDSolver::solveInternalConstraints() {

  for (int iter(0); iter < solverIterations_; ++iter) {
    //solveDistanceConstraints();

    solveBendingConstraints();
  }

}

void PBDSolver::solveDistanceConstraints() {

  typedef MyMesh::Point Point;

  typedef OpenMesh::VectorT<double, 3> V3;

  auto e_end = body_->mesh_->edges_end();

  std::cout << "Distance Constraint correction: " << std::endl;

  for (auto& constraint : body_->distanceConstraints_) {
    MyMesh::VertexHandle vh0 = body_->mesh_->vertex_handle(constraint.vertexIdx_[0]);
    MyMesh::VertexHandle vh1 = body_->mesh_->vertex_handle(constraint.vertexIdx_[1]);

    Point p0 = tmpPosition_[constraint.vertexIdx_[0]];
    Point p1 = tmpPosition_[constraint.vertexIdx_[1]];

    V3 v0(p0[0], p0[1], p0[2]);
    V3 v1(p1[0], p1[1], p1[2]);

    //OpenMesh::VectorT<double, 3> dP = constraint.computeCorrection(v0, v1);
    OpenMesh::VectorT<double, 3> dP = constraint.computeCorrection(v0, v1, body_->weights_[constraint.vertexIdx_[0]], body_->weights_[constraint.vertexIdx_[1]]);
    std::cout << "<" << vh0.idx() << "," << vh1.idx() << ">" << dP[0] << " " << dP[1] << " " << dP[2] << std::endl;

    if (body_->weights_[vh0.idx()] > 0.0)
      tmpPosition_[vh0.idx()] -= dP * body_->weights_[vh0.idx()];

    if (body_->weights_[vh1.idx()] > 0.0)
      tmpPosition_[vh1.idx()] += dP * body_->weights_[vh1.idx()];

  }

}

void PBDSolver::solveBendingConstraints() {

  typedef OpenMesh::Vec3d VertexType;
  typedef float ScalarType;

  auto v_it = body_->mesh_->vertices_begin(); 

  std::cout << "==================== Bending Constraint correction ====================" << std::endl;

  for (auto& constraint : body_->bendingConstraints_) {
    Real d = 0, phi = 0, i_d = 0;

    // The two edge-connected triangles consist of four vertices
    // p1, p2, p3, p4
    // p1-p2 make up the shared edge and p3, p4 are unique to the
    // respective triangle

    MyMesh::FaceHandle fh0 = body_->mesh_->face_handle(constraint.fidx0);
    MyMesh::FaceHandle fh1 = body_->mesh_->face_handle(constraint.fidx1);

    // Let us get the Vectors from the handles and start computing
    VertexType p1 = tmpPosition_[constraint.p1];
    VertexType p2 = tmpPosition_[constraint.p2] - p1;
    VertexType p3 = tmpPosition_[constraint.p3] - p1;
    VertexType p4 = tmpPosition_[constraint.p4] - p1;

    VertexType p2p3 = OpenMesh::cross(p2, p3);
    VertexType p2p4 = OpenMesh::cross(p2, p4);

    Real lenp2p3 = OpenMesh::norm(p2p3);

    if (lenp2p3 == 0.0) {
      return;
    }

    Real lenp2p4 = OpenMesh::norm(p2p4);

    if (lenp2p4 == 0.0) {
      return;
    }

    VertexType n1 = p2p3.normalized();
    VertexType n2 = p2p4.normalized();
    std::cout << "<normal" << 1 << ">" << n1[0] << " " << n1[1] << " " << n1[2] << std::endl;
    std::cout << "<normal" << 2 << ">" << n2[0] << " " << n2[1] << " " << n2[2] << std::endl;

    d = OpenMesh::dot(n1, n2);
    phi = std::acos(d);
    std::cout << "Bending Angle between normals: " << phi << std::endl;
    std::cout << "<" << "rest angle" << ">" << constraint.restAngle_ << std::endl;

    if (d < -1.0) {
      d = -1.0;
      std::cout << "< -1 case: " << std::endl;
    }
    else if (d > 1.0) {
      d = 1.0;
      std::cout << "> 1 case: " << std::endl;
    }

    // 0 degree case, triangles in opposite direction, but folded together 
    if (d == -1.0) {
      phi = std::atan(1.0) * 4.0;
      std::cout << "== -1: " << std::endl;
      std::cout << "phi: " << phi << ":" << constraint.restAngle_ << std::endl;
      if (phi == constraint.restAngle_) {
        continue;
      }
    }

    // 180 degree case, triangles are in the same plane
    if (d == 1.0) {
      phi = 0.0;
      std::cout << "180 Degree case, dot product of normals " << "== 1: " << std::endl;
      if (phi == constraint.restAngle_) {
        continue;
      }
    }

    Real dArcCos = std::sqrt(1.0 - (d * d)) * (phi - constraint.restAngle_);
    std::cout << "dArcCos: " << "> " << dArcCos << std::endl;

    VertexType p2n1 = OpenMesh::cross(p2, n1);
    VertexType p2n2 = OpenMesh::cross(p2, n2);
    VertexType p3n2 = OpenMesh::cross(p3, n2);
    VertexType p4n1 = OpenMesh::cross(p4, n1);

    VertexType n1p2 = -p2n1;
    VertexType n2p2 = -p2n2;

    VertexType n1p3 = OpenMesh::cross(n1, p3);
    VertexType n2p4 = OpenMesh::cross(n2, p4);

    // compute the qs
    VertexType q3 = (p2n2 + n1p2 * d) / lenp2p3;
    VertexType q4 = (p2n1 + n2p2 * d) / lenp2p4;
    VertexType q2 = (-(p3n2 + n1p3 * d) / lenp2p3) - ((p4n1 + n2p4 * d) / lenp2p4);

    VertexType q1 = -q2 - q3 - q4;

    ScalarType q1_len2 = OpenMesh::dot(q1, q1);
    ScalarType q2_len2 = OpenMesh::dot(q2, q2);
    ScalarType q3_len2 = OpenMesh::dot(q3, q3);
    ScalarType q4_len2 = OpenMesh::dot(q4, q4);

    ScalarType sum =
      body_->weights_[constraint.p1] * (q1_len2) +
      body_->weights_[constraint.p2] * (q2_len2) +
      body_->weights_[constraint.p3] * (q3_len2) +
      body_->weights_[constraint.p4] * (q4_len2);

    VertexType dP1 = -((body_->weights_[constraint.p1] * dArcCos) / sum) * q1;
    VertexType dP2 = -((body_->weights_[constraint.p2] * dArcCos) / sum) * q2;
    VertexType dP3 = -((body_->weights_[constraint.p3] * dArcCos) / sum) * q3;
    VertexType dP4 = -((body_->weights_[constraint.p4] * dArcCos) / sum) * q4;

    if (body_->weights_[constraint.p1] > 0.0) {
      tmpPosition_[constraint.p1] += dP1 * constraint.k;
    }
    if (body_->weights_[constraint.p2] > 0.0) {
      tmpPosition_[constraint.p2] += dP2 * constraint.k;
    }
    if (body_->weights_[constraint.p3] > 0.0) {
      tmpPosition_[constraint.p3] += dP3 * constraint.k;
    }
    if (body_->weights_[constraint.p4] > 0.0) {
      tmpPosition_[constraint.p4] += dP4 * constraint.k;
    }

    std::cout << "<dP1" << ">" << dP1[0] << " " << dP1[1] << " " << dP1[2] << std::endl;
    std::cout << "<dP2" << ">" << dP2[0] << " " << dP2[1] << " " << dP2[2] << std::endl;
    std::cout << "<dP3" << ">" << dP3[0] << " " << dP3[1] << " " << dP3[2] << std::endl;
    std::cout << "<dP4" << ">" << dP4[0] << " " << dP4[1] << " " << dP4[2] << std::endl;

  }

}

void PBDSolver::calculatePositionPrediction() {

  tmpPosition_.clear();

  for (int idx(0); idx < body_->mesh_->n_vertices(); ++idx) {
    if(body_->weights_[idx] <= 0.0) 
      tmpPosition_.push_back(body_->mesh_->point(MyMesh::VertexHandle(idx)));
    else
      tmpPosition_.push_back(body_->mesh_->point(MyMesh::VertexHandle(idx)) + (body_->velocities_[idx] * dt_));
  }

  std::cout << "Predicted Position: " << std::endl;
  for (auto& v : tmpPosition_) {
    std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  }

}

void PBDSolver::verletIntegration() {

  typedef OpenMesh::Vec3d V3;
	Real inv_dt = 1.0 / dt_;
	size_t i=0; 

  for (int idx(0); idx < body_->mesh_->n_vertices(); ++idx) {
    MyMesh::VertexHandle vh = body_->mesh_->vertex_handle(idx);
    body_->velocities_[idx] = (tmpPosition_[idx] - body_->mesh_->point(vh)) * inv_dt;
    V3 old = body_->mesh_->point(vh);
    std::cout << "<" << idx << "old position" << ">" << old[0] << " " << old[1] << " " << old[2] << std::endl;
    body_->mesh_->set_point(vh, tmpPosition_[idx]);
    V3 newP = body_->mesh_->point(vh);
    std::cout << "<" << idx << "new position" << ">" << newP[0] << " " << newP[1] << " " << newP[2] << std::endl;
  }

}

}
