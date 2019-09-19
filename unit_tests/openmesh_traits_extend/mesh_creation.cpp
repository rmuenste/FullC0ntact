#include "mesh_creation.hpp"
#include <vector>

int numX = 20, numY=20; //these ar the number of quads
const size_t total_points = (numX+1)*(numY+1);
int mysize = 4;
float hsize = mysize/2.0f;

SimpleMesh generatePlaneMesh() {

  typedef SimpleMesh::Point Point;
  typedef SimpleMesh::VertexHandle VHandle;
  SimpleMesh mesh;

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

