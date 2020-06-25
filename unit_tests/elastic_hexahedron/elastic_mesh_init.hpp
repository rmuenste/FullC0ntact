#pragma once

typedef boost::optional< Tree::Intersection_and_primitive_id<Ray>::Type > Ray_intersection;

template <typename T, typename MeshType>
class AxesIntersector {

  using VertexType = OpenVolumeMesh::Geometry::VectorT<T, 3>;
  using K = CGAL::Simple_cartesian<T>;
  using Ray = typename K::Ray_3;
  using Line = typename K::Line_3;
  using Point = typename K::Point_3;
  using Triangle = typename K::Triangle_3;
  using cgalVec = typename K::Vector_3;

  using Iterator = typename std::list<Triangle>::iterator;
  using Primitive = typename CGAL::AABB_triangle_primitive<K, Iterator>;
  using AABB_triangle_traits = typename CGAL::AABB_traits<K, Primitive>;
  using Tree = typename CGAL::AABB_tree<AABB_triangle_traits>;

  using Mesh = typename CGAL::Surface_mesh<Point>;
  using face_descriptor = typename boost::graph_traits<Mesh>::face_descriptor;
  using halfedge_descriptor = typename boost::graph_traits<Mesh>::halfedge_descriptor;

  private:  
    MeshType &mesh_;

  public:
    explicit AxesIntersector(MeshType& _mesh) : mesh_(_mesh) {

    }

  void triangleIntersection(const VertexType& p0, const VertexType& p1, const VertexType& p2, 
                            const VertexType& p3, const VertexType &direction,
                            const VertexType& baryCenter,
                            int faceIdx, DeformationAxis<T> &axis) {

    Point a(p0[0], p0[1], p0[2]);
    Point b(p1[0], p1[1], p1[2]);
    Point c(p2[0], p2[1], p2[2]);
    Point d(p3[0], p3[1], p3[2]);

    std::list<Triangle> triangles;
    triangles.push_back(Triangle(a, b, c));
    triangles.push_back(Triangle(a, c, d));

    Tree tree(triangles.begin(), triangles.end());

    //Point center(0, 0, 0);
    Point center(baryCenter[0], baryCenter[1], baryCenter[2]);

    //cgalVec dir(direction[0], direction[1], direction[2]);
    cgalVec dir(direction[0]-baryCenter[0], direction[1]-baryCenter[1], direction[2]-baryCenter[2]);

    Ray ray_query(center, dir);

    Ray_intersection intersection = tree.first_intersection(ray_query);

    const Point* p;;
    if (tree.number_of_intersected_primitives(ray_query)) {
      p = boost::get<Point>(&(intersection->first));
      if (p) {
        VertexType q(p->x(), p->y(), p->z());

        T area = (p0 - p1).norm() * (p0 - p3).norm();

        T T1 = 0.5 * OpenVolumeMesh::cross((q - p1), (p1 - p2)).norm();

        T T2 = 0.5 * OpenVolumeMesh::cross((q - p2), (p2 - p3)).norm();

        T xi = 2.0 * T1 / area;
        T eta = 2.0 * T2 / area;

        axis.faceIndices.push_back(faceIdx);

        axis.parameters.push_back(std::make_pair(xi, eta));

        axis.q.push_back(q);
      }
    }
  }
};

void triangleIntersection(const VertexType& p0, const VertexType& p1, const VertexType& p2, 
                          const VertexType& p3, const VertexType &direction, 
                          const VertexType& baryCenter,
                          int faceIdx, DeformationAxis<ScalarType> &axis) {

  Point a(p0[0], p0[1], p0[2]);
  Point b(p1[0], p1[1], p1[2]);
  Point c(p2[0], p2[1], p2[2]);
  Point d(p3[0], p3[1], p3[2]);

  std::list<Triangle> triangles;
  triangles.push_back(Triangle(a, b, c));
  triangles.push_back(Triangle(a, c, d));

  Tree tree(triangles.begin(), triangles.end());

  //Point center(0, 0, 0);
  Point center(baryCenter[0], baryCenter[1], baryCenter[2]);

  //cgalVec dir(direction[0], direction[1], direction[2]);
  cgalVec dir(direction[0]-baryCenter[0], direction[1]-baryCenter[1], direction[2]-baryCenter[2]);

  Ray ray_query(center, dir);

//  std::cout << tree.number_of_intersected_primitives(ray_query)
//    << " intersections(s) with ray " << std::endl;

  Ray_intersection intersection = tree.first_intersection(ray_query);

  const Point* p;;
  if (tree.number_of_intersected_primitives(ray_query)) {
    p = boost::get<Point>(&(intersection->first));
    if (p) {
//      std::cout << "Intersection object is a point " << *p << std::endl;
      VertexType q(p->x(), p->y(), p->z());

      ScalarType area = (p0 - p1).norm() * (p0 - p3).norm();
//      std::cout << "Surface area rect: " << area << std::endl;

      ScalarType T1 = 0.5 * OpenVolumeMesh::cross((q - p1), (p1 - p2)).norm();
//      std::cout << "Surface area T1: " << T1 << std::endl;

      ScalarType T2 = 0.5 * OpenVolumeMesh::cross((q - p2), (p2 - p3)).norm();
//      std::cout << "Surface area T2: " << T2 << std::endl;

      ScalarType xi = 2.0 * T1 / area;
      ScalarType eta = 2.0 * T2 / area;

//      std::cout << "xi: " << 2.0 * T1 / area << std::endl;
//      std::cout << "eta: " <<  2.0 * T2 / area << std::endl;

      axis.faceIndices.push_back(faceIdx);

      axis.parameters.push_back(std::make_pair(xi, eta));

      axis.q.push_back(q);
    }

  }
  
}

void loadMeshFromFile(std::string fileName, HexMesh& myMesh) {

  std::ifstream in(fileName, std::ios::in);

  if (!in) {
    std::cout << "Cannot open file: " << fileName << std::endl;
    std::exit(EXIT_FAILURE);
  }

  char buf[1024];

  int numVertices, numLines;
  in >> numVertices;
  in >> numLines;

  // this will read until we find the
  // first newline character or 1024 bytes
  in.getline(buf, 1024);

  for (int i(0); i < numVertices; ++i) {

    VertexType vert;

    int vidx;

    in >> vidx >> vert[0] >> vert[1] >> vert[2];
    in.getline(buf, 1024);
    std::cout << "Vertex: " << vert << std::endl;
    myMesh.add_vertex(vert);

  }

  for (int i(1); i <= numLines; ++i) {

    VertexType vert;

    int idx, type;

    in >> idx >> type;
    std::cout << "line, type: " << idx << "," << type << std::endl;

    if (type == 102) {
      std::cout << "Found: Edge" << std::endl;
    }
    else if (type == 204) {
      std::cout << "Found: Face" << std::endl;
      int v0, v1, v2, v3;
      in >> v0 >> v1 >> v2 >> v3;
      std::vector<OpenVolumeMesh::VertexHandle> vertices;
      vertices.push_back(OpenVolumeMesh::VertexHandle(v0-1));
      vertices.push_back(OpenVolumeMesh::VertexHandle(v1-1));
      vertices.push_back(OpenVolumeMesh::VertexHandle(v2-1));
      vertices.push_back(OpenVolumeMesh::VertexHandle(v3-1));
      OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
      
      // Add faces
//      vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v2);vertices.push_back(v3);
//      OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
    }
    else if (type == 308) {
      std::cout << "Found: Cell" << std::endl;
    }
    else {
      std::cout << "Found: Unknown type" << std::endl;
    }
    in.getline(buf, 1024);

  }

  in.close();

}

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

  OpenVolumeMesh::VertexHandle v8  = myMesh.add_vertex(VertexType(-1.0,-1.0, 2.0));
  OpenVolumeMesh::VertexHandle v9  = myMesh.add_vertex(VertexType( 1.0,-1.0, 2.0));
  OpenVolumeMesh::VertexHandle v10 = myMesh.add_vertex(VertexType( 1.0, 1.0, 2.0));
  OpenVolumeMesh::VertexHandle v11 = myMesh.add_vertex(VertexType(-1.0, 1.0, 2.0));
  
  std::vector<OpenVolumeMesh::VertexHandle> vertices;
  
  // Add faces
  // 1 Bottom face
  vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v2);vertices.push_back(v3);
  OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
  
  vertices.clear();

  // 2 top face
  vertices.push_back(v4); vertices.push_back(v5);vertices.push_back(v6);vertices.push_back(v7);
  //vertices.push_back(v7); vertices.push_back(v6);vertices.push_back(v5);vertices.push_back(v4);
  OpenVolumeMesh::FaceHandle f1 = myMesh.add_face(vertices);
  
  vertices.clear();

  // 3 front face
  vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v5);vertices.push_back(v4);
  //vertices.push_back(v4); vertices.push_back(v5);vertices.push_back(v1);vertices.push_back(v0);
  OpenVolumeMesh::FaceHandle f2 = myMesh.add_face(vertices);
  
  vertices.clear();

  // 4 right face
  vertices.push_back(v1); vertices.push_back(v2);vertices.push_back(v6);vertices.push_back(v5);
  OpenVolumeMesh::FaceHandle f3 = myMesh.add_face(vertices);
  
  vertices.clear();

  // 5 back face
  vertices.push_back(v2); vertices.push_back(v3);vertices.push_back(v7);vertices.push_back(v6);
  OpenVolumeMesh::FaceHandle f4 = myMesh.add_face(vertices);

  vertices.clear();

  // 6 left face
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
  myMesh.add_cell(halffaces, true);

}

void setupDeformationAxes(HexMesh &myMesh, DeformationAxis<ScalarType> zeta[3]) {

  VertexType baryCenter(0,0,0);

  OpenVolumeMesh::CellPropertyT<VertexType> centerProp =
  myMesh.request_cell_property<VertexType>("barycenter");

  OpenVolumeMesh::CellIter c_it = myMesh.cells_begin();

  for (; c_it != myMesh.cells_end(); ++c_it) {
    baryCenter = VertexType(0,0,0);
    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &myMesh);
    for (; cv_it.valid(); ++cv_it) {
      baryCenter += myMesh.vertex(*cv_it);
//      std::cout << "Vertex idx " << cv_it->idx() << " in Cell " << (*c_it).idx() << ")" << std::endl;
    }

    baryCenter *= 0.125;
    std::cout << "Barycenter of cell (" << (*c_it).idx() << ") :" << baryCenter << std::endl;
    centerProp[*c_it] = baryCenter;
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
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], zeta[idx].dir, centerProp[*c_it], cf_it->idx(), zeta[idx]);
        triangleIntersection(vertices[0], vertices[1], vertices[2], vertices[3], -zeta[idx].dir, centerProp[*c_it], cf_it->idx(), zeta[idx]);
      }

    }

  }

}