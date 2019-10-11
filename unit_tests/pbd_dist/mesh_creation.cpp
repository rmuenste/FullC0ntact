#include "mesh_creation.hpp"
#include <vector>

extern float kStretch; 
extern int numX, numY;
extern const size_t total_points;
extern int mysize;
extern float hsize;

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
      VHandle vh = mesh.add_vertex(Point( ((float(i)/(u-1)) *2-1)* hsize, float(mysize+1), ((float(j)/(v-1) )* mysize)));
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
         mesh.add_face(faceVh);
         faceVh.clear();
         faceVh.push_back(VHandle(i1)); faceVh.push_back(VHandle(i2)); faceVh.push_back(VHandle(i3)); 
         mesh.add_face(faceVh);
			} else {                
         faceVh.clear();
         faceVh.push_back(VHandle(i0)); faceVh.push_back(VHandle(i2)); faceVh.push_back(VHandle(i3)); 
         mesh.add_face(faceVh);
         faceVh.clear();
         faceVh.push_back(VHandle(i0)); faceVh.push_back(VHandle(i3)); faceVh.push_back(VHandle(i1)); 
         mesh.add_face(faceVh);
			}        
		}    
	}

  mesh.request_vertex_normals();
  mesh.request_face_normals();
  mesh.update_normals();

  return mesh;
}

std::vector<BendingConstraint> generateBendingConstraints(MyMesh& mesh) {

  std::vector<BendingConstraint> constraints;

  std::cout << "> Bending constraint generation: " << std::endl;
  auto f_end = mesh.faces_end();

  for (auto f_it = mesh.faces_begin(); f_it != f_end; ++f_it) {
    std::cout << "> Face idx: " << (*f_it).idx() << std::endl;

    //MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it);
    auto fv_it = mesh.fv_iter(*f_it);
    for (; fv_it.is_valid(); ++fv_it) {
      std::cout << "> Vertex idx: " << (*fv_it).idx() << std::endl;
    }

    auto ff_it = mesh.ff_iter(*f_it);

    for (; ff_it.is_valid(); ++ff_it) {
      OpenMesh::Vec3f normal1 = mesh.normal((*f_it));
      OpenMesh::Vec3f normal2 = mesh.normal((*ff_it));
      std::cout << "Angle between normals: " << std::acos(OpenMesh::dot(normal1, normal2)) << std::endl;
    }
  }

  return constraints;

}

void updateBendingConstraint(MyMesh& mesh, BendingConstraint& constraint) {

  MyMesh::FaceHandle fh0 = mesh.face_handle(constraint.fidx0);
  MyMesh::FaceHandle fh1 = mesh.face_handle(constraint.fidx1);

  OpenMesh::Vec3f normal1 = mesh.normal(fh0);
  OpenMesh::Vec3f normal2 = mesh.normal(fh1);

  std::cout << "Angle between normals: " << std::acos(OpenMesh::dot(normal1, normal2)) << std::endl;
}

void generateConstraints(MyMesh& mesh) {

  auto e_end = mesh.edges_end();
  for (auto e_it=mesh.edges_begin(); e_it!=e_end; ++e_it)
  {
    VHandle vh0 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,0));
    VHandle vh1 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,1));

    OpenMesh::Vec3f v0 = mesh.point(vh0);
    OpenMesh::Vec3f v1 = mesh.point(vh1);

    double restLength = (v0 - v1).norm();

    mesh.data(*e_it).dc_ = DistanceConstraint(restLength, kStretch);
  }

}

MyMesh generateSimpleMesh() {

  MyMesh mesh_;
  
  MyMesh::VertexHandle vhandle[4];

  MyMesh::Point vertices[] = {
    MyMesh::Point(-2, 5, 0), 
    MyMesh::Point(-1.8, 5, 0),
    MyMesh::Point(-2, 5, 0.2),
    MyMesh::Point(-1.8, 5, 0.2)
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

  OpenMesh::Vec3f normal1 = mesh_.normal(mesh_.face_handle(0));
  OpenMesh::Vec3f normal2 = mesh_.normal(mesh_.face_handle(1));

  std::cout << "Angle between normals: " << std::acos(OpenMesh::dot(normal1, normal2)) << std::endl;

  generateConstraints(mesh_);

  return mesh_;

}
