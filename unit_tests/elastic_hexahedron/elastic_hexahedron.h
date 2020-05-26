#pragma once

int main(int _argc, char** _argv);

ScalarType computeCellVolume(HexMesh &mesh, const OpenVolumeMesh::CellHandle &cellHandle) {

  OpenVolumeMesh::CellIter c_it = OpenVolumeMesh::CellIter(&mesh, cellHandle); 

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

void computeAllCellVolumes(HexMesh &mesh) {

  OpenVolumeMesh::CellIter c_it = mesh.cells_begin();

  OpenVolumeMesh::CellPropertyT<ScalarType> volProp = mesh.request_cell_property<ScalarType>("volume");

  for (; c_it != mesh.cells_end(); ++c_it) {
    ScalarType volume = computeCellVolume(mesh, *c_it);
    volProp[*c_it] = volume;
  }

}

void calculateAllParticleMasses(HexMesh &mesh) {

  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {

    ScalarType mass = 0.0;

    ScalarType rho = 1.0;

    OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
    mesh.request_vertex_property<ScalarType>("mass");

    OpenVolumeMesh::CellPropertyT<ScalarType> volProp =
    mesh.request_cell_property<ScalarType>("volume");

    OpenVolumeMesh::VertexCellIter vc_it(*v_it, &mesh);
    for (; vc_it.valid(); ++vc_it) {
      ScalarType volume = volProp[*vc_it];
      mass += rho * volume;
    }

    mass *= 0.25;
    massProp[*v_it] = mass;
  }
}

// std::cout << "xi: " << 2.0 * T1 / area << std::endl;
// std::cout << "eta: " <<  2.0 * T2 / area << std::endl;
ScalarType evaluateFaceBasisFunction(ScalarType xi, ScalarType eta, int vidx) {
  if(vidx == 0) return (1.0 - xi) * (1.0 - eta);
  else if(vidx == 1) return xi * (1.0 - eta);
  else if(vidx == 2) return xi * eta;
  else if(vidx == 3) return (1.0 - xi) * eta;
  else
    return 0.0;
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
