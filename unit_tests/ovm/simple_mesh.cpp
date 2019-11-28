// C++ includes
#include <iostream>
#include <vector>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>

// Include polyhedral mesh kernel
#include <OpenVolumeMesh/Mesh/TetrahedralMesh.hh>

// Make some typedefs to facilitate your life
typedef OpenVolumeMesh::Geometry::Vec3d         VertexType;
typedef OpenVolumeMesh::GeometryKernel<VertexType>   TetMesh;

int main(int _argc, char** _argv) {

    // Create mesh object
    TetMesh myMesh;

    // Add eight vertices
    OpenVolumeMesh::VertexHandle v0 = myMesh.add_vertex(VertexType(-1.0,-1.0,-1.0));
    OpenVolumeMesh::VertexHandle v1 = myMesh.add_vertex(VertexType( 1.0,-1.0,-1.0));
    OpenVolumeMesh::VertexHandle v2 = myMesh.add_vertex(VertexType(-1.0, 1.0,-1.0));
    OpenVolumeMesh::VertexHandle v4 = myMesh.add_vertex(VertexType(-1.0,-1.0, 1.0));
    
    std::vector<OpenVolumeMesh::VertexHandle> vertices;
    
    // Add faces
    vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v2);
    OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
    
    vertices.clear();
    vertices.push_back(v0); vertices.push_back(v1);vertices.push_back(v4);
    OpenVolumeMesh::FaceHandle f1 = myMesh.add_face(vertices);
    
    vertices.clear();
    vertices.push_back(v1); vertices.push_back(v2);vertices.push_back(v4);
    OpenVolumeMesh::FaceHandle f2 = myMesh.add_face(vertices);
    
    vertices.clear();
    vertices.push_back(v2); vertices.push_back(v0);vertices.push_back(v4);
    OpenVolumeMesh::FaceHandle f3 = myMesh.add_face(vertices);
    
    std::vector<OpenVolumeMesh::HalfFaceHandle> halffaces;
    
    // Add first tetrahedron
    halffaces.push_back(myMesh.halfface_handle(f0, 1));
    halffaces.push_back(myMesh.halfface_handle(f1, 1));
    halffaces.push_back(myMesh.halfface_handle(f2, 0)); 
    halffaces.push_back(myMesh.halfface_handle(f3, 1)); 
    myMesh.add_cell(halffaces);

    VertexType baryCenter(0,0,0);

    // Print positions of vertices to std out
    for(OpenVolumeMesh::VertexIter v_it = myMesh.vertices_begin();
            v_it != myMesh.vertices_end(); ++v_it) {

        std::cout << "Position of vertex " << v_it->idx() << ": " <<
            myMesh.vertex(*v_it) << std::endl;

    }
    
    std::cout << "Number of cells in the mesh " << myMesh.n_cells() << std::endl;

    OpenVolumeMesh::CellIter c_it = myMesh.cells_begin();
    for (; c_it != myMesh.cells_end(); ++c_it) {
      OpenVolumeMesh::CellVertexIter cv_it((*c_it), &myMesh);
      for (; cv_it.valid(); ++cv_it) {
        baryCenter += myMesh.vertex(*cv_it);
      }
      baryCenter *= 0.25;
      std::cout << "Barycenter of cell (" << (*c_it).idx() << ") :" << baryCenter << std::endl;
    }

    c_it = myMesh.cells_begin();
    for (; c_it != myMesh.cells_end(); ++c_it) {
      OpenVolumeMesh::CellFaceIter cf_it((*c_it), &myMesh);
      for (; cf_it.valid(); ++cf_it) {
        std::cout << "Cell (" << (*c_it).idx() << ") has face:" << cf_it->idx() << std::endl;

        OpenVolumeMesh::FaceVertexIter fv_it((*cf_it), &myMesh);
        for (; fv_it.valid(); ++fv_it) {
          std::cout << "   face (" << (*cf_it).idx() << ") has vertex:" << fv_it->idx() << std::endl;
        }
      }
    }

    return 0;
}
