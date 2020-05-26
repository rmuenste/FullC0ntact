void initHexMesh(HexMesh& myMesh) {
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
  
  // Add faces to face array 
  halffaces.push_back(myMesh.halfface_handle(f0, 1));
  halffaces.push_back(myMesh.halfface_handle(f1, 1));
  halffaces.push_back(myMesh.halfface_handle(f2, 0)); 
  halffaces.push_back(myMesh.halfface_handle(f3, 1)); 

  halffaces.push_back(myMesh.halfface_handle(f4, 0)); 
  halffaces.push_back(myMesh.halfface_handle(f5, 1)); 

  // Construct a cell from the faces  
  myMesh.add_cell(halffaces);

}

void setupDeformationAxes(HexMesh &myMesh, DeformationAxis<ScalarType> zeta[3]) {

  VertexType baryCenter(0,0,0);
  OpenVolumeMesh::CellIter c_it = myMesh.cells_begin();
  for (; c_it != myMesh.cells_end(); ++c_it) {
    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &myMesh);
    for (; cv_it.valid(); ++cv_it) {
      baryCenter += myMesh.vertex(*cv_it);
//      std::cout << "Vertex idx " << cv_it->idx() << " in Cell " << (*c_it).idx() << ")" << std::endl;
    }
    baryCenter *= 0.125;
//    std::cout << "Barycenter of cell (" << (*c_it).idx() << ") :" << baryCenter << std::endl;
  }

  c_it = myMesh.cells_begin();
  for (; c_it != myMesh.cells_end(); ++c_it) {
    OpenVolumeMesh::CellFaceIter cf_it((*c_it), &myMesh);
    for (; cf_it.valid(); ++cf_it) {
//      std::cout << "Cell (" << (*c_it).idx() << ") has face:" << cf_it->idx() << std::endl;

      std::vector<VertexType> vertices;
      OpenVolumeMesh::FaceVertexIter fv_it((*cf_it), &myMesh);
      for (; fv_it.valid(); ++fv_it) {
//        std::cout << "   face (" << (*cf_it).idx() << ") has vertex:" << fv_it->idx() << std::endl;
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
//      std::cout << "normal: " << n << std::endl;

      for (int idx(0); idx < 3; ++idx) {
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], zeta[idx].dir, cf_it->idx(), zeta[idx]);
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], -zeta[idx].dir, cf_it->idx(), zeta[idx]);
      }

    }

  }

}