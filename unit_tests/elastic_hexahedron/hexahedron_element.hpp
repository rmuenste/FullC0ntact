
template <typename T>
class ElasticHexahedron {

public:

  typedef OpenVolumeMesh::Geometry::VectorT<T, 3> VertexType;

  OpenVolumeMesh::CellHandle cellHandle_;

  DeformationAxis<ScalarType> zeta_[3];

  HexaMatrix mat_;

  // The vertexMap maps the global idx to the local idx
  std::map<int, int> vertexMap_;

  ScalarType restLengths[3];
  ScalarType alphaZero[3];
  ScalarType faceArea[6];

  // Update Intersection points/axes after a time step
  void updateIntersectionPoints(HexMesh &myMesh) {

    // Computation of axial forces is a loop over the axes
    for (int idx(0); idx < 3; ++idx) {
      int col = idx * 2;

      VertexType q0(0, 0, 0);
      VertexType q1(0, 0, 0);

      OpenVolumeMesh::FaceHandle fh0 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[0]);

      OpenVolumeMesh::FaceIter f_it(&myMesh, fh0);
      OpenVolumeMesh::FaceVertexIter fv_it(*f_it, &myMesh);

      // Get the local vertex indices of the first intersected face 
      for (; fv_it.valid(); ++fv_it) {

        int vidx = fv_it->idx();
        int midx = vertexMap_[vidx];
        q0 += mat_(midx, col) * myMesh.vertex(*fv_it);
      }

//      std::cout << "Old intersection point " << col << " : " << zeta_[idx].q[0] << std::endl;
//      std::cout << "New intersection point " << col << " : " << q0 << std::endl;
      zeta_[idx].q[0] = q0;
      col++;
      OpenVolumeMesh::FaceHandle fh1 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[1]);

      f_it = OpenVolumeMesh::FaceIter(&myMesh, fh1);
      fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);

      // Get the local vertex indices of the first intersected face 
      for (; fv_it.valid(); ++fv_it) {
        int vidx = fv_it->idx();
        int midx = vertexMap_[vidx];
        q1 += mat_(midx, col) * myMesh.vertex(*fv_it);
      }

//      std::cout << "Old intersection point " << col << " : " << zeta_[idx].q[1] << std::endl;
//      std::cout << "New intersection point " << col << " : " << q1 << std::endl;
      zeta_[idx].q[1] = q1;

    }


  }

  // Calculates initial rest lengths and angles
  void calculateInitialValues(HexMesh& myMesh) {
    for (int idx(0); idx < 3; ++idx) {
      restLengths[idx] = zeta_[idx].getDir().length();
      std::cout << "Axis " << idx << " rest length: " << restLengths[idx] << std::endl;
    }

    int count = 0;
    for (int i(0); i < 2; ++i) {
      for (int j(i + 1); j < 3; ++j) {
        std::cout << "i: " << i << "j: " << j << std::endl;
        alphaZero[count] = OpenVolumeMesh::dot(zeta_[i].getDir(), zeta_[j].getDir());
        std::cout << "Spring " << count << " alpha zero: " << alphaZero[count] << std::endl;
        count++;
      }
    }
  }

  void calculateDerSpringForces(HexMesh& myMesh, OpenVolumeMesh::VertexHandle &vhA, ScalarType delta) {

    VertexType du = delta * VertexType(1, 1, 1);

    OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
      myMesh.request_vertex_property<VertexType>("forceD");

    // Computation of axial forces is a loop over the axes
    for (int idx(0); idx < 3; ++idx) {

      // Get face handle 
      OpenVolumeMesh::FaceHandle fh0 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[0]);
      OpenVolumeMesh::FaceHandle fh1 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[1]);

      OpenVolumeMesh::FaceIter f_it(&myMesh, fh0);
      OpenVolumeMesh::FaceVertexIter fv_it(*f_it, &myMesh);
      //====================================================================================================================
      //                                          Compute the first face force
      //====================================================================================================================
      //VertexType v0 = 1.1 * zeta_[idx].q[1];
      VertexType v0 = zeta_[idx].q[1];
      VertexType v1 = zeta_[idx].q[0];

      //f_it = OpenVolumeMesh::FaceIter(&myMesh, fh0);
      //fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);

      // If it is our vertex add onto it
      for (; fv_it.valid(); ++fv_it) {

        int vidx = fv_it->idx();
        OpenVolumeMesh::VertexHandle vhB = *fv_it;

        if (vhB == vhA) {
          v1 = zeta_[idx].q[0] + du;
          //std::cout << "Adding to v1 = [" << v1 << ", " << vhA.idx() << "," << vidx << "," << idx << "]" << std::endl;
        }

      }

      f_it = OpenVolumeMesh::FaceIter(&myMesh, fh1);
      fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);
      for (; fv_it.valid(); ++fv_it) {

        int vidx = fv_it->idx();
        OpenVolumeMesh::VertexHandle vhB = *fv_it;

        if (vhB == vhA) {
          v0 = zeta_[idx].q[1] + du;
          //std::cout << "Adding to v0 = [" << v0 << ", " << vhA.idx() << "," << vidx << "," << idx << "]" << std::endl;
        }

      }

      //VertexType zetaTilde = (zeta_[idx].q[1] - zeta_[idx].q[0]).normalize();
      VertexType zetaTilde = (v0 - v1).normalize();

      VertexType faceNormal = computeFaceNormal(fh0, myMesh);
      //std::cout << "Face normal = [" << faceNormal << "] face idx: " << zeta_[idx].faceIndices[0] << std::endl;

      std::vector<ScalarType> k = {ScalarType(9.5908288), ScalarType(-1.09148493), ScalarType(3.69164292e-02)};
      for (auto &kl : k) {
        ScalarType dotProd = std::abs(OpenVolumeMesh::dot(faceNormal, zetaTilde));
        //std::cout << "New kl = [" << dotProd << "," << kl << "," << faceArea[zeta_[idx].faceIndices[0]] << "]" << std::endl;
        kl = faceArea[zeta_[idx].faceIndices[0]] * kl * dotProd;
      }

//      for (auto &kl : k) {
//        std::cout << "New kl = [" << kl << "] on face: " << zeta_[idx].faceIndices[0] << std::endl;
//      }
      
      CubicSpring<ScalarType> spring(k[0], k[1], k[2], restLengths[idx]);
      VertexType forceVector = spring.evaluateForce(v1, v0);
//      std::cout << "Cubic Spring Force = [" << spring.evaluateForce(v1, v0) << "] on face: " << zeta_[idx].faceIndices[0] << std::endl;

      // Get the local vertex indices of the first intersected face 
      VertexType mid(0,0,0);

      f_it = OpenVolumeMesh::FaceIter(&myMesh, fh0);
      fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);

      for (int i(0); fv_it.valid(); ++fv_it, ++i) {
        int vidx = fv_it->idx();
        mid += myMesh.vertex(*fv_it);
        OpenVolumeMesh::VertexHandle vhB = *fv_it;
        if (vhB == vhA) {
          ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[0].first, zeta_[idx].parameters[0].second, i);
          std::cout << "Partial Force = [" << value * forceVector << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
          std::cout << "Adding to  " << vidx << std::endl;
          forceProp[*fv_it] += value * forceVector;
        }
      }

      mid *= 0.25;
//      std::cout << "Face mid point = [" << mid << "]" << std::endl;
//      std::cout << "Direction by dot prod = [" << OpenVolumeMesh::dot(mid, faceNormal) << "]" << std::endl;

      //====================================================================================================================
      //                                          Compute the second face force
      //====================================================================================================================
      faceNormal = computeFaceNormal(fh1, myMesh);
//      std::cout << "Face normal = [" << faceNormal << "] face idx: " << zeta_[idx].faceIndices[1] << std::endl;

      k = {ScalarType(9.5908288), ScalarType(-1.09148493), ScalarType(3.69164292e-02)};

      for (auto &kl : k) {
        ScalarType dotProd = std::abs(OpenVolumeMesh::dot(faceNormal, zetaTilde));
//        std::cout << "New kl = [" << dotProd << "," << kl << "," << faceArea[zeta_[idx].faceIndices[1]] << "]" << std::endl;
        kl = faceArea[zeta_[idx].faceIndices[1]] * kl * dotProd;
      }

//      for (auto &kl : k) {
//        std::cout << "New kl = [" << kl << "] on face: " << zeta_[idx].faceIndices[1] << std::endl;
//      }
      
      CubicSpring<ScalarType> spring1(k[0], k[1], k[2], restLengths[idx]);
      VertexType forceVector1 = -spring1.evaluateForce(v1, v0);
//      std::cout << "Cubic Spring Force = [" << forceVector1 << "] on face: " << zeta_[idx].faceIndices[1] << std::endl;

      // Get the local vertex indices of the first intersected face 
      f_it = OpenVolumeMesh::FaceIter(&myMesh, fh1);
      fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);
      mid = VertexType(0,0,0);
      for (int i(0); fv_it.valid(); ++fv_it, ++i) {
        int vidx = fv_it->idx();
        mid += myMesh.vertex(*fv_it);
        OpenVolumeMesh::VertexHandle vhB = *fv_it;

        if (vhB == vhA) {
          ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[1].first, zeta_[idx].parameters[1].second, i);
          std::cout << "Partial Force = [" << value * forceVector1 << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
          forceProp[*fv_it] += value * forceVector1;
        }

      }

      mid *= 0.25;
//      std::cout << "Face mid point = [" << mid << "]" << std::endl;
//      std::cout << "Direction by dot prod = [" << OpenVolumeMesh::dot(mid, faceNormal) << "]" << std::endl;

    }

  }

  void calculateSpringForces(HexMesh& myMesh) {

    OpenVolumeMesh::VertexPropertyT<VertexType> forceProp =
      myMesh.request_vertex_property<VertexType>("force");

    // Computation of axial forces is a loop over the axes
    for (int idx(0); idx < 3; ++idx) {

      //====================================================================================================================
      //                                          Compute the first face force
      //====================================================================================================================
      //VertexType v0 = 1.1 * zeta_[idx].q[1];
      VertexType &v0 = zeta_[idx].q[1];
      //std::cout << "v0 = [" << v0 << "]" << std::endl;
      //std::cout << "Cubic Spring Force = [" << spring.evaluateForce(zeta_[idx].q[0], zeta_[idx].q[1]) << "]" << std::endl;

      // Get face handle 
      OpenVolumeMesh::FaceHandle fh0 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[0]);
      OpenVolumeMesh::FaceHandle fh1 = OpenVolumeMesh::FaceHandle(zeta_[idx].faceIndices[1]);

      OpenVolumeMesh::FaceIter f_it(&myMesh, fh0);
      OpenVolumeMesh::FaceVertexIter fv_it(*f_it, &myMesh);

      VertexType zetaTilde = (zeta_[idx].q[1] - zeta_[idx].q[0]).normalize();
      //VertexType zetaTilde = (zeta_[idx].q[0] - zeta_[idx].q[1]).normalize();

      VertexType faceNormal = computeFaceNormal(fh0, myMesh);

#ifdef VERBOSE_DEBUG
      std::cout << "Force for axis [" << idx << "], zeta_tilde = " << "[" << zetaTilde << "]" << std::endl;
      std::cout << "Face normal = [" << faceNormal << "] face idx: " << zeta_[idx].faceIndices[0] << std::endl;
#endif

      std::vector<ScalarType> k = {ScalarType(9.5908288), ScalarType(-1.09148493), ScalarType(3.69164292e-02)};
      for (auto &kl : k) {
        ScalarType dotProd = std::abs(OpenVolumeMesh::dot(faceNormal, zetaTilde));
#ifdef VERBOSE_DEBUG
        std::cout << "New kl = [" << dotProd << "," << kl << "," << faceArea[zeta_[idx].faceIndices[0]] << "]" << std::endl;
#endif
        kl = faceArea[zeta_[idx].faceIndices[0]] * kl * dotProd;
      }
#ifdef VERBOSE_DEBUG
      for (auto &kl : k) {
        std::cout << "New kl = [" << kl << "] on face: " << zeta_[idx].faceIndices[0] << std::endl;
      }
#endif
      
      CubicSpring<ScalarType> spring(k[0], k[1], k[2], restLengths[idx]);
      VertexType forceVector = -spring.evaluateForce(zeta_[idx].q[0], v0);
#ifdef VERBOSE_DEBUG
      std::cout << "Cubic Spring Force = [" << spring.evaluateForce(zeta_[idx].q[0], v0) << "] on face: " << zeta_[idx].faceIndices[0] << std::endl;
#endif

      // Get the local vertex indices of the first intersected face 
      VertexType mid(0,0,0);
      for (int i(0); fv_it.valid(); ++fv_it, ++i) {
        int vidx = fv_it->idx();
        mid += myMesh.vertex(*fv_it);
        ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[0].first, zeta_[idx].parameters[0].second, i);
#ifdef VERBOSE_DEBUG
        std::cout << "Partial Force = [" << value * forceVector << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
        std::cout << "Partial Force = [" << value * forceVector << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
#endif
        forceProp[*fv_it] += value * forceVector;
      }
      mid *= 0.25;
#ifdef VERBOSE_DEBUG
      std::cout << "Face mid point = [" << mid << "]" << std::endl;
      std::cout << "Direction by dot prod = [" << OpenVolumeMesh::dot(mid, faceNormal) << "]" << std::endl;
#endif

      //====================================================================================================================
      //                                          Compute the second face force
      //====================================================================================================================
      faceNormal = computeFaceNormal(fh1, myMesh);
#ifdef VERBOSE_DEBUG
      std::cout << "Face normal = [" << faceNormal << "] face idx: " << zeta_[idx].faceIndices[1] << std::endl;
#endif

      k = {ScalarType(9.5908288), ScalarType(-1.09148493), ScalarType(3.69164292e-02)};

      for (auto &kl : k) {
        ScalarType dotProd = std::abs(OpenVolumeMesh::dot(faceNormal, zetaTilde));
#ifdef VERBOSE_DEBUG
        std::cout << "New kl = [" << dotProd << "," << kl << "," << faceArea[zeta_[idx].faceIndices[1]] << "]" << std::endl;
#endif
        kl = faceArea[zeta_[idx].faceIndices[1]] * kl * dotProd;
      }

#ifdef VERBOSE_DEBUG
      for (auto &kl : k) {
        std::cout << "New kl = [" << kl << "] on face: " << zeta_[idx].faceIndices[1] << std::endl;
      }
#endif
      
      CubicSpring<ScalarType> spring1(k[0], k[1], k[2], restLengths[idx]);
      VertexType forceVector1 = spring1.evaluateForce(zeta_[idx].q[0], v0);
#ifdef VERBOSE_DEBUG
      std::cout << "Cubic Spring Force = [" << forceVector1 << "] on face: " << zeta_[idx].faceIndices[1] << std::endl;
#endif

      // Get the local vertex indices of the first intersected face 
      f_it = OpenVolumeMesh::FaceIter(&myMesh, fh1);
      fv_it = OpenVolumeMesh::FaceVertexIter(*f_it, &myMesh);
      mid = VertexType(0,0,0);
      for (int i(0); fv_it.valid(); ++fv_it, ++i) {
        int vidx = fv_it->idx();
        mid += myMesh.vertex(*fv_it);
        ScalarType value = evaluateLocalBasisFunction(zeta_[idx].parameters[1].first, zeta_[idx].parameters[1].second, i);
#ifdef VERBOSE_DEBUG
        std::cout << "Partial Force = [" << value * forceVector1 << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
        std::cout << "Partial Force = [" << value * forceVector1 << "] for global vertex: " << vidx << " local idx: " << i << std::endl;
#endif
        forceProp[*fv_it] += value * forceVector1;
      }

      mid *= 0.25;
#ifdef VERBOSE_DEBUG
      std::cout << "Face mid point = [" << mid << "]" << std::endl;
      std::cout << "Direction by dot prod = [" << OpenVolumeMesh::dot(mid, faceNormal) << "]" << std::endl;
#endif
    }

//    int count = 0;
//    for (int i(0); i < 2; ++i) {
//      for (int j(i + 1); j < 3; ++j) {
//        CubicTorsionSpring<ScalarType> torsionSpring(ScalarType(1.0), ScalarType(1.0), ScalarType(1.0), alphaZero[count]);
//        std::pair<VertexType, VertexType> torsionForcePair = torsionSpring.evaluateTorsion(zeta_[i].getDir(), zeta_[j].getDir());
//        std::cout << "Cubic Torsion Spring Force evaluate = [" << i << "," << j << "]" << std::endl;
//        count++;
//      }
//    }

  }

  VertexType computeFaceNormal(const OpenVolumeMesh::FaceHandle& _fh, HexMesh &kernel_) {
  
      if(kernel_.face(_fh).halfedges().size() < 3) {
          std::cerr << "Warning: Degenerate face detected!" << std::endl;
          return VertexType(0,0,0);
      }
  
      typedef OpenVolumeMesh::HalfEdgeHandle HalfEdgeHandle;
      const std::vector<HalfEdgeHandle>& halfedges = kernel_.face(_fh).halfedges();
      std::vector<HalfEdgeHandle>::const_iterator he_it = halfedges.begin();
  
      VertexType p1 = kernel_.vertex(kernel_.halfedge(*he_it).from_vertex());
      VertexType p2 = kernel_.vertex(kernel_.halfedge(*he_it).to_vertex());
      ++he_it;
      VertexType p3 = kernel_.vertex(kernel_.halfedge(*he_it).to_vertex());
  
      VertexType n = OpenVolumeMesh::cross( (p2 - p1), (p3 - p2) );
      //VertexType n = (p2 - p1) % (p3 - p2);
      n.normalize();
      //std::cout << "Face Normal = [" << n << "]" << std::endl;
      return n;
  
  }

  void printCellData() {

    for (int idx(0); idx < 3; ++idx) {
      std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[0] << std::endl;
      std::cout << "Parameters [" << zeta_[idx].parameters[0].first << ", " << zeta_[idx].parameters[0].second << "]" << std::endl;

      std::cout << "Axis " << idx << " intersects with face: " << zeta[idx].faceIndices[1] << std::endl;
      std::cout << "Parameters [" << zeta_[idx].parameters[1].first << ", " << zeta_[idx].parameters[1].second << "]" << std::endl;
    }

  }

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

  ScalarType calculateFaceArea(std::vector<VertexType> &vertices) {
    ScalarType area1 = 0.5 * OpenVolumeMesh::cross((vertices[1] - vertices[0]), (vertices[2] - vertices[0])).norm();
    ScalarType area2 = 0.5 * OpenVolumeMesh::cross((vertices[2] - vertices[0]), (vertices[3] - vertices[0])).norm();
    return (area1 + area2);
  }

  void updateFaceArea(HexMesh& myMesh) {

//      ScalarType T1 = 0.5 * OpenVolumeMesh::cross((q - p1), (p1 - p2)).norm();
//      std::cout << "Surface area T1: " << T1 << std::endl;
    OpenVolumeMesh::CellFaceIter cf_it(cellHandle_, &myMesh);
    for (int fidx=0; cf_it.valid(); ++cf_it, ++fidx) {
      OpenVolumeMesh::FaceVertexIter fv_it(*cf_it, &myMesh);
      std::vector<VertexType> vertexCoords;
      for (; fv_it.valid(); ++fv_it) {
        vertexCoords.push_back(myMesh.vertex(*fv_it));
      }
      ScalarType area = calculateFaceArea(vertexCoords);
      
      faceArea[fidx] = area; 
      std::cout << "Area " << cf_it->idx() << " = " << area << std::endl;
    }
  }

  //========================================================================================
  //                       Calculate the "C"-Matrix of the Hexahedron
  //========================================================================================
  void calculateCoefficientMatrix(HexMesh &myMesh) {
    // In column j of the C-Matrix we store the basis function values for intersection point q_j
    // Since intersection point q_j lies in face f_j there is a non-zero value in the column
    // if vertex v_i, i in [0...7], is part of face f_j

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


