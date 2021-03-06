#include "mesh_creation.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <array>

extern float kStretch; 
extern int numX, numY;
extern const size_t total_points;
extern int mysize;
extern float hsize;
typedef OpenMesh::Vec3d VertexType;

MyMesh generatePlaneMesh() {

  typedef MyMesh::Point Point;
  typedef MyMesh::VertexHandle VHandle;
  MyMesh mesh;

  std::vector<VHandle> vertexHandles;
  std::vector<VHandle> faceVh;
	int v = numY+1;
	int u = numX+1;

	for(int j=0;j<=numY;j++) {		 
		for(int i=0;i<=numX;i++) {	 
      VHandle vh = mesh.add_vertex(Point( ((double(i)/(u-1)) *2-1)* hsize, double(mysize+1), ((double(j)/(v-1) )* mysize)));
      vertexHandles.push_back(vh);
		}
	}

	for (int i = 0; i < numY; i++) {        
		for (int j = 0; j < numX; j++) {            
			int i0 = i * (numX+1) + j;            
			int i1 = i0 + 1;            
			int i2 = i0 + (numX+1);            
			int i3 = i2 + 1;            
			if ((j+i)%2) {                
         faceVh.clear();
         faceVh.push_back(VHandle(i0)); faceVh.push_back(VHandle(i2)); faceVh.push_back(VHandle(i1)); 
         std::cout << "Triangle <" << i0 << " " << i2 << " " << i1 << std::endl;
         mesh.add_face(faceVh);
         faceVh.clear();
         faceVh.push_back(VHandle(i1)); faceVh.push_back(VHandle(i2)); faceVh.push_back(VHandle(i3)); 
         std::cout << "Triangle <" << i1 << " " << i2 << " " << i3 << std::endl;
         mesh.add_face(faceVh);
			} else {                
         faceVh.clear();
         faceVh.push_back(VHandle(i0)); faceVh.push_back(VHandle(i2)); faceVh.push_back(VHandle(i3)); 
         std::cout << "Triangle <" << i0 << " " << i2 << " " << i3 << std::endl;
         mesh.add_face(faceVh);
         faceVh.clear();
         faceVh.push_back(VHandle(i0)); faceVh.push_back(VHandle(i3)); faceVh.push_back(VHandle(i1)); 
         std::cout << "Triangle <" << i0 << " " << i3 << " " << i1 << std::endl;
         mesh.add_face(faceVh);
			}        
		}    
	}

  mesh.request_vertex_normals();
  mesh.request_face_normals();
  mesh.update_normals();

  return mesh;
}

std::pair<int, int> getSortedPair(int idx0, int idx1) {

  if (idx0 <= idx1)
    return std::make_pair(idx0, idx1);
  else
    return std::make_pair(idx1, idx0);

}

std::array<MyMesh::VertexHandle, 4> getConstraintVertices(MyMesh& mesh, MyMesh::FaceHandle &fh0, MyMesh::FaceHandle &fh1) {

  std::array<MyMesh::VertexHandle, 4> verts;

  std::cout << "fh1: " << fh1.idx() << std::endl;
  std::cout << "fh0: " << fh0.idx() << std::endl;

  auto fh_it = mesh.fh_begin(fh0);
  for (; fh_it != mesh.fh_end(fh0); ++fh_it) {
    MyMesh::HalfedgeHandle he = *fh_it; 
    MyMesh::HalfedgeHandle opposingHalfEdge = mesh.opposite_halfedge_handle(he);
    MyMesh::FaceHandle faceHandle = mesh.face_handle(opposingHalfEdge);
    std::cout << "Found Opposing face" << faceHandle.idx() << std::endl;
    if (faceHandle.idx() == fh1.idx()) {
      std::cout << "Found Opposing face" << std::endl;
      std::cout << "To vertex: " << mesh.to_vertex_handle(fh_it) << std::endl;
      std::cout << "From vertex: " << mesh.from_vertex_handle(fh_it) << std::endl;
      std::cout << "3rd vertex: " << mesh.to_vertex_handle(mesh.next_halfedge_handle(fh_it)) << std::endl;
      std::cout << "4rd vertex: " 
        << mesh.to_vertex_handle(mesh.next_halfedge_handle(opposingHalfEdge)) 
        << std::endl;

      verts[0] = mesh.to_vertex_handle(fh_it);
      verts[1] = mesh.from_vertex_handle(fh_it);
      verts[2] = mesh.to_vertex_handle(mesh.next_halfedge_handle(fh_it));
      verts[3] = mesh.to_vertex_handle(mesh.next_halfedge_handle(opposingHalfEdge));
    }
  }

  return verts;
}

std::vector<BendingConstraintd> generateBendingConstraints(MyMesh& mesh, int solverIterations) {

  std::cout << "> Bending constraint generation: " << std::endl;
  auto f_end = mesh.faces_end();

  std::set<std::pair<int, int>> constraintPairs;
  for (auto f_it = mesh.faces_begin(); f_it != f_end; ++f_it) {
    std::cout << "> Face idx: " << (*f_it).idx() << std::endl;

    //MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it);
    auto fv_it = mesh.fv_iter(*f_it);
    for (; fv_it.is_valid(); ++fv_it) {
      std::cout << "> Vertex idx: " << (*fv_it).idx() << std::endl;
    }

    auto ff_it = mesh.ff_iter(*f_it);

    for (; ff_it.is_valid(); ++ff_it) {
      int f0 = (*f_it).idx(); int f1 = (*ff_it).idx();

      std::pair<int, int> sortedPair = getSortedPair(f0, f1);
      constraintPairs.insert(sortedPair);

    }
  }

  std::vector<BendingConstraintd> constraints(constraintPairs.size());
  std::cout << "> Constraint pairs: " << std::endl;
  // range-based for loop 
  int idx = 0;
  for (auto const &x : constraintPairs) { 
      std::cout << "(" << x.first << ", "
           << x.second << ")"
           << " "; 
      std::cout << std::endl;
      MyMesh::FaceHandle fh0 = mesh.face_handle(x.first);
      MyMesh::FaceHandle fh1 = mesh.face_handle(x.second);
      

      std::array<MyMesh::VertexHandle, 4> vidx = getConstraintVertices(mesh, fh0, fh1);
      //int vvidx[4] = { vidx[0].idx(), vidx[1].idx(), vidx[2].idx(), vidx[3].idx() };
      int vvidx[4] = { 3, 0, 2, 1 };

      VertexType p1 = mesh.point(mesh.vertex_handle(vvidx[0]));
      VertexType p2 = mesh.point(mesh.vertex_handle(vvidx[1])) - p1;
      VertexType p3 = mesh.point(mesh.vertex_handle(vvidx[2])) - p1;
      VertexType p4 = mesh.point(mesh.vertex_handle(vvidx[3])) - p1;

      VertexType p2p3 = OpenMesh::cross(p2, p3);
      VertexType p2p4 = OpenMesh::cross(p2, p4);

      double lenp2p3 = p2p3.norm();

      double lenp2p4 = p2p4.norm();

      VertexType n1 = p2p3.normalized();
      VertexType n2 = p2p4.normalized();
      std::cout << "<normal" << 1 << ">" << n1[0] << " " << n1[1] << " " << n1[2] << std::endl;
      std::cout << "<normal" << 2 << ">" << n2[0] << " " << n2[1] << " " << n2[2] << std::endl;

      std::cout << "> (Face, Face) idx: " << fh0 << "," << fh1 << std::endl;

      double restAngle = std::acos(OpenMesh::dot(n1, n2));
      std::cout << "Angle between normals: " << restAngle << std::endl;

      unsigned f0 = fh0.idx();
      unsigned f1 = fh1.idx();
      constraints[idx] = BendingConstraintd(restAngle, 0.2, f0, f1, vvidx, solverIterations);
      idx++;
  } 

  return constraints;

}

std::vector<BendingConstraintd> generatePlaneBendingConstraints(MyMesh& mesh, int solverIterations) {

	int v = numY+1;
	int u = numX+1;

	int l1=0, l2=0;

  std::vector<BendingConstraintd> constraints;

	for(int i = 0; i < v-1; ++i) {
		for(int j = 0; j < u-1; ++j) {	 			 
			int p1 = i * (numX+1) + j;            
			int p2 = p1 + 1;            
			int p3 = p1 + (numX+1);            
			int p4 = p3 + 1;   
			 

			if ((j+i)%2) {  						
        std::cout << "A-bending<" << p3 << ">" << p2 << " " << p1 << " " << p4 << std::endl;
        int vvidx[4] = { p3, p2, p1, p4 };
        unsigned f0 = 0;
        unsigned f1 = 1;

        VertexType p1 = mesh.point(mesh.vertex_handle(vvidx[0]));
        VertexType p2 = mesh.point(mesh.vertex_handle(vvidx[1])) - p1;
        VertexType p3 = mesh.point(mesh.vertex_handle(vvidx[2])) - p1;
        VertexType p4 = mesh.point(mesh.vertex_handle(vvidx[3])) - p1;

        VertexType p2p3 = OpenMesh::cross(p2, p3);
        VertexType p2p4 = OpenMesh::cross(p2, p4);

        double lenp2p3 = p2p3.norm();

        double lenp2p4 = p2p4.norm();

        VertexType n1 = p2p3.normalized();
        VertexType n2 = p2p4.normalized();
        std::cout << "<normal" << 1 << ">" << n1[0] << " " << n1[1] << " " << n1[2] << std::endl;
        std::cout << "<normal" << 2 << ">" << n2[0] << " " << n2[1] << " " << n2[2] << std::endl;

        double restAngle = std::acos(OpenMesh::dot(n1, n2));
        std::cout << "Angle between normals: " << restAngle << std::endl;

        constraints.push_back(BendingConstraintd(restAngle, 0.5, f0, f1, vvidx, solverIterations));
			} else {  
        std::cout << "B-bending<" << p4 << ">" << p1 << " " << p3 << " " << p2 << std::endl;
        unsigned f0 = 0;
        unsigned f1 = 1;
        int vvidx[4] = { p4, p1, p3, p2 };
        VertexType p1 = mesh.point(mesh.vertex_handle(vvidx[0]));
        VertexType p2 = mesh.point(mesh.vertex_handle(vvidx[1])) - p1;
        VertexType p3 = mesh.point(mesh.vertex_handle(vvidx[2])) - p1;
        VertexType p4 = mesh.point(mesh.vertex_handle(vvidx[3])) - p1;

        VertexType p2p3 = OpenMesh::cross(p2, p3);
        VertexType p2p4 = OpenMesh::cross(p2, p4);

        double lenp2p3 = p2p3.norm();

        double lenp2p4 = p2p4.norm();

        VertexType n1 = p2p3.normalized();
        VertexType n2 = p2p4.normalized();
        std::cout << "<normal" << 1 << ">" << n1[0] << " " << n1[1] << " " << n1[2] << std::endl;
        std::cout << "<normal" << 2 << ">" << n2[0] << " " << n2[1] << " " << n2[2] << std::endl;

        double restAngle = std::acos(OpenMesh::dot(n1, n2));
        std::cout << "Angle between normals: " << restAngle << std::endl;
        constraints.push_back(BendingConstraintd(restAngle, 0.5, f0, f1, vvidx, solverIterations));
			}     
		}
	}		 

  return constraints;
}

std::vector<DistanceConstraintd> generateDistanceConstraints(MyMesh& mesh, int solverIterations) {

  std::vector<DistanceConstraintd> constraints;

  auto e_end = mesh.edges_end();
  for (auto e_it=mesh.edges_begin(); e_it!=e_end; ++e_it)
  {
    VHandle vh0 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,0));
    VHandle vh1 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,1));

    VertexType v0 = mesh.point(vh0);
    VertexType v1 = mesh.point(vh1);

    double restLength = (v0 - v1).norm();

    constraints.push_back(DistanceConstraintd(restLength, kStretch, (*e_it).idx(), vh0.idx(), vh1.idx(), solverIterations));
  }

  return constraints;

}

std::vector<DistanceConstraintd> generatePlaneDistanceConstraints(MyMesh& mesh, int solverIterations) {

  std::vector<DistanceConstraintd> constraints;

	int v = numY+1;
	int u = numX+1;

	int l1=0, l2=0;

	//setup constraints
	// Horizontal
  int eidx = 0;
	for (l1 = 0; l1 < v; l1++)	// v
		for (l2 = 0; l2 < (u - 1); l2++) {
      std::cout << "H-distance<" << (l1 * u) + l2 << "," << (l1 * u) + l2 + 1 << std::endl;
      VHandle vh0 = mesh.vertex_handle((l1 * u) + l2);
      VHandle vh1 = mesh.vertex_handle((l1 * u) + l2 + 1);

      VertexType v0 = mesh.point(vh0);
      VertexType v1 = mesh.point(vh1);

      double restLength = (v0 - v1).norm();

      constraints.push_back(DistanceConstraintd(restLength, kStretch, eidx, vh0.idx(), vh1.idx(), solverIterations));
      eidx++;
		}

	// Vertical
	for (l1 = 0; l1 < (u); l1++)	
		for (l2 = 0; l2 < (v - 1); l2++) {
      std::cout << "V-distance<" << (l2 * u) + l1 << "," << ((l2 + 1) * u) + l1 << std::endl;
      VHandle vh0 = mesh.vertex_handle((l2 * u) + l1);
      VHandle vh1 = mesh.vertex_handle(((l2 + 1) * u) + l1);

      VertexType v0 = mesh.point(vh0);
      VertexType v1 = mesh.point(vh1);

      double restLength = (v0 - v1).norm();

      constraints.push_back(DistanceConstraintd(restLength, kStretch, eidx, vh0.idx(), vh1.idx(), solverIterations));
      eidx++;
		}

	
	// Shearing distance constraint
	for (l1 = 0; l1 < (v - 1); l1++)	
		for (l2 = 0; l2 < (u - 1); l2++) {
      std::cout << "S-distance<" << (l1 * u) + l2 << "," << ((l1 + 1) * u) + l2 + 1 << std::endl;

      VHandle vh0 = mesh.vertex_handle((l1 * u) + l2);
      VHandle vh1 = mesh.vertex_handle(((l1 + 1) * u) + l2 + 1);

      VertexType v0 = mesh.point(vh0);
      VertexType v1 = mesh.point(vh1);

      double restLength = (v0 - v1).norm();

      constraints.push_back(DistanceConstraintd(restLength, kStretch, eidx, vh0.idx(), vh1.idx(), solverIterations));
      eidx++;

      std::cout << "S-distance<" << ((l1 + 1) * u) + l2 << "," << (l1 * u) + l2 + 1 << std::endl;

      vh0 = mesh.vertex_handle(((l1 + 1) * u) + l2);
      vh1 = mesh.vertex_handle((l1 * u) + l2 + 1);

      v0 = mesh.point(vh0);
      v1 = mesh.point(vh1);

      restLength = (v0 - v1).norm();

      constraints.push_back(DistanceConstraintd(restLength, kStretch, eidx, vh0.idx(), vh1.idx(), solverIterations));
      eidx++;
		}

  return constraints;

}

MyMesh generateSimpleMesh() {

  MyMesh mesh_;
  
  MyMesh::VertexHandle vhandle[4];

  MyMesh::Point vertices[] = {
    MyMesh::Point(-2, 5, 0), 
    MyMesh::Point(2.0, 5, 0),
    MyMesh::Point(-2, 5, 4.0),
    MyMesh::Point(2.0, 5, 4.0)
  };

  int connectivity[][3] = { {0, 2, 3}, {0, 3, 1} };

  for (unsigned i(0); i < 4; ++i) {
    vhandle[i] = mesh_.add_vertex(vertices[i]);
  }

  std::vector<MyMesh::VertexHandle> face_vhandles;

  for (unsigned i(0); i < 2; ++i) {
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[connectivity[i][0]]);
    face_vhandles.push_back(vhandle[connectivity[i][1]]);
    face_vhandles.push_back(vhandle[connectivity[i][2]]);
    mesh_.add_face(face_vhandles);
  }

  mesh_.request_face_normals();
  mesh_.request_vertex_normals();

  mesh_.update_normals();

  if (!mesh_.has_face_normals())
  {
    std::cerr << "ERROR: Standard face property 'Normals' not available!\n";
    std::exit(EXIT_FAILURE);
  }

  if (!mesh_.has_vertex_normals())
  {
    std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    std::exit(EXIT_FAILURE);
  }

  VertexType normal1 = mesh_.normal(mesh_.face_handle(0));
  VertexType normal2 = mesh_.normal(mesh_.face_handle(1));

  //std::cout << "Angle between normals: " << std::acos(OpenMesh::dot(normal1, normal2)) << std::endl;

  return mesh_;

}
