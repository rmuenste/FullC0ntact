#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <ostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

struct MyTraits : OpenMesh::DefaultTraits {
  // Point and Normal
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;

  // Define VertexTraits
  VertexTraits{

    private:
      Point cog_;
      int originalIdx_;

    public:
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), originalIdx_(-1), extruded(false) {}

      const Point& cog() const { return cog_; }

      void set_cog(const Point& p) { cog_ = p; }

      void set_orig_idx(int i) { originalIdx_ = i; }

      int get_orig_idx() const { return originalIdx_; }

      bool extruded;

  };
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;
typedef double ScalarType;

void WritePrismMesh(std::vector<MyMesh> layers, const char *strFileName)
{

  using namespace std;
  FILE * myfile = fopen(strFileName,"w");

  if (myfile == NULL) {
    cout<<"Error opening file: "<<strFileName<<endl;
    exit(0);
  }

  int verts = 4 * layers[0].n_vertices();
  int totalVertices = verts;

  int i;
  fprintf(myfile,"# vtk DataFile Version 2.0\n");
  fprintf(myfile,"Generated by FullC0ntact\n");
  fprintf(myfile,"ASCII\n");
  fprintf(myfile,"DATASET UNSTRUCTURED_GRID\n");
  fprintf(myfile,"POINTS %i double\n", totalVertices);

  typedef MyMesh::VertexIter Viter;
  for(int j(0); j < layers.size(); ++j)
    for (Viter v_it = layers[j].vertices_begin(); v_it != layers[j].vertices_end(); ++v_it) {
      fprintf(myfile,"%f %f %f \n", layers[j].point(*v_it)[0], layers[j].point(*v_it)[1], layers[j].point(*v_it)[2]);
    }

  int ncells = 3 * layers[0].n_faces();
  int nfaces = layers[0].n_faces();

  fprintf(myfile,"CELLS %i %i\n",ncells, ncells*7);

  int offset = layers[0].n_vertices();

  MyMesh *mesh = &layers[0];

  for (int ilayers = 1; ilayers < layers.size(); ++ilayers)
  {

    for (auto f_it = mesh->faces_begin(); f_it != mesh->faces_end(); ++f_it) {
      auto fv_it = mesh->fv_iter(*f_it);

      std::vector<int> face1;

      for (; fv_it.is_valid(); ++fv_it) {
        int idx = (*fv_it).idx();
        face1.push_back(idx);
      }

      int i0 = face1[0] + (ilayers - 1) * offset;
      int i1 = face1[1] + (ilayers - 1) * offset;
      int i2 = face1[2] + (ilayers - 1) * offset;
      int i3 = face1[0] + (ilayers) * offset;
      int i4 = face1[1] + (ilayers) * offset;
      int i5 = face1[2] + (ilayers) * offset;
      fprintf(myfile, "6  %i %i %i %i %i %i\n", i0, i1, i2, i3, i4, i5);

    }
  }

  fprintf(myfile,"CELL_TYPES %i\n",ncells);
  for(i=0;i<ncells;i++)
  {
    fprintf(myfile,"13\n");
  }//end for

  fprintf(myfile,"CELL_DATA %i \n",ncells);
  fprintf(myfile,"SCALARS CellID int 1\n");
  fprintf(myfile,"LOOKUP_TABLE default\n");

  for (int ilayers = 1; ilayers < layers.size(); ++ilayers)
  {
    for(int fidx(0); fidx < nfaces; ++fidx)
      fprintf(myfile,"%i\n", ilayers);
  }

  //close the file
  fclose (myfile);

}

// As input we get surface meshes and we write them out as connected
// volume meshes
void WriteUnstr(std::vector<MyMesh*> &layers, const char *strFileName)
{

  using namespace std;
  FILE * myfile = fopen(strFileName,"w");

  if (myfile == NULL) {
    cout<<"Error opening file: "<<strFileName<<endl;
    exit(0);
  }  

  int verts = layers[0]->n_vertices();
  int totalVertices = layers.size() * layers[0]->n_vertices();

  int i;
  fprintf(myfile,"# vtk DataFile Version 2.0\n");
  fprintf(myfile,"Generated by FullC0ntact\n");
  fprintf(myfile,"ASCII\n");
  fprintf(myfile,"DATASET UNSTRUCTURED_GRID\n");
  fprintf(myfile,"POINTS %i double\n", totalVertices);      

  typedef MyMesh::VertexIter Viter;
  for(auto &mesh : layers)
  {
    for (Viter v_it = mesh->vertices_begin(); v_it != mesh->vertices_end(); ++v_it) {
      fprintf(myfile,"%f %f %f \n", mesh->point(*v_it)[0], mesh->point(*v_it)[1], mesh->point(*v_it)[2]);    
    }
  }//end for

  int nfaces = (layers.size() - 1) * layers[0]->n_faces();

  fprintf(myfile,"CELLS %i %i\n",nfaces,nfaces*7);
  //fprintf(myfile,"CELLS %i %i\n",1,7);
  
//  fprintf(myfile,"6  %i %i %i %i %i %i\n",0, 1, 2, verts, verts+1, verts+2);
  for (int ilayers = 1; ilayers < layers.size(); ++ilayers)
  {
    auto mesh = layers[ilayers-1];
    auto mesh1 = layers[ilayers];
    for (auto f_it = mesh->faces_begin(), f_it2 = mesh1->faces_begin(); f_it != mesh->faces_end(); ++f_it, ++f_it2) {
      auto fv_it = mesh->fv_iter(*f_it);

      std::vector<int> face1;

      for (; fv_it.is_valid(); ++fv_it) {
        int idx = (*fv_it).idx();
        face1.push_back(idx);
      }

      std::vector<int> face2;
      fv_it = mesh1->fv_iter(*f_it2);

      for (; fv_it.is_valid(); ++fv_it) {
        int idx = (*fv_it).idx();
        face2.push_back(idx);
      }

      int i0 = face1[0] + (ilayers - 1) * verts;
      int i1 = face1[1] + (ilayers - 1) * verts;
      int i2 = face1[2] + (ilayers - 1) * verts;
      int i3 = face2[0] + (ilayers) * verts;
      int i4 = face2[1] + (ilayers) * verts;
      int i5 = face2[2] + (ilayers) * verts;
      fprintf(myfile, "6  %i %i %i %i %i %i\n", i0, i1, i2, i3, i4, i5);

    }
  }

  fprintf(myfile,"CELL_TYPES %i\n",nfaces);        
  for(i=0;i<nfaces;i++)
  {
    fprintf(myfile,"13\n");          
  }//end for

  //close the file
  fclose (myfile);  

}

void writeOFFMesh(MyMesh& mesh, std::string fileName) {

  try {
    if (!OpenMesh::IO::write_mesh(mesh, fileName)) {
      std::cerr << "Cannot write mesh to file '"<< fileName << "'" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
  catch( std::exception &x) {
    std::cerr << x.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

}

MyMesh readMesh(std::string fileName) {
  MyMesh mesh;

  if (typeid(OpenMesh::vector_traits<MyMesh::Point>::value_type) != typeid(double)) {
    std::cerr << "Data type error" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (typeid(OpenMesh::vector_traits<MyMesh::Normal>::value_type) != typeid(double)) {
    std::cerr << "Data type error" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  mesh.request_vertex_normals();
  mesh.request_face_normals();

  OpenMesh::IO::Options opt;

  if (!OpenMesh::IO::read_mesh(mesh, fileName, opt)) {
    std::cerr << "Error: cannot read from file " << fileName << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!opt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_face_normals() && mesh.has_vertex_normals()) {
    mesh.update_normals();
  }

  return mesh;
}

void scaleAlongNormal(MyMesh& mesh, ScalarType dist) {
  typedef MyMesh::VertexIter Viter;
  for (Viter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
    //std::cout << "Vertex #" << *v_it << ": " << mesh.point(*v_it);
    mesh.set_point(*v_it, mesh.point(*v_it) + dist * mesh.normal(*v_it));
    //std::cout << "moved to " << mesh.point( *v_it ) << std::endl;
  }
    std::cout << "Moving finished" << std::endl;
}

MyMesh getScaledMesh(MyMesh& mesh, ScalarType dist) {

  typedef MyMesh::VertexIter Viter;
  MyMesh extMesh(mesh);
  extMesh.request_vertex_normals();
  extMesh.request_face_normals();
  extMesh.update_normals();

  std::vector<MyMesh::VertexHandle> vhandle;
  std::vector<MyMesh::Point> vertices;
  std::vector<MyMesh::VertexHandle> face_vhandles;

  for (Viter v_it = extMesh.vertices_begin(); v_it != extMesh.vertices_end(); ++v_it) {
    //mesh.set_point(*v_it, mesh.point(*v_it) + dist * mesh.normal(*v_it));
    MyMesh::Point hqNormal = extMesh.normal(*v_it);
    hqNormal[2] = 0.0;
    hqNormal.normalize();
    MyMesh::Point p = extMesh.point(*v_it) + dist * hqNormal;
    extMesh.set_point(*v_it, p);
  }
  std::cout << "Moving finished" << std::endl;


  return extMesh;
}

// Command line arguments:
// 1) method 2) input surface mesh 3) solidifiedMesh 4) prism volume mesh 5) name of surface mesh for next iteration 6) extrusion thickness 
// command2 = "./boundarylayer_generator 4 %s %s %s %s %f" %(os.path.basename(surfaceMeshName), os.path.basename(outputLayerName),  prismName, nextBaseMesh, thickness)
int main(int argc, char * argv[])
{

  int imethod = std::atoi(argv[1]);

  if (imethod == 1) {

    std::cout << "Method 1 not supported for production... exiting" << std::endl;
    std::exit(EXIT_FAILURE);

  } else if (imethod == 2) {

    std::cout << "Method 2 not supported for production... exiting" << std::endl;
    std::exit(EXIT_FAILURE);

  } else if (imethod == 3) {

    std::cout << "Length: " << argc << " arg2 " << argv[2] << " arg3 " << argv[3] << " arg4 " << argv[4] << std::endl;

    std::cout << "Method 3" << std::endl;

    std::string workingDir = std::string(argv[4]);

    std::string baseName = std::string(argv[2]); //offInputName.str(); 

    int iLayers = std::atoi(argv[3]);

    // Get the faces from this file
    std::ostringstream offInputName;
    offInputName << workingDir << "/" << argv[2];

    std::vector<MyMesh> layerMeshes;

    for(int i(1); i < iLayers + 1; ++i) {

      std::ostringstream name;
      name << workingDir << "/" << baseName << i << ".off";

      std::ostringstream outputName;
      outputName << name.str();

      layerMeshes.push_back(readMesh(name.str()));

      writeOFFMesh(layerMeshes[i-1], outputName.str()); 
      //WritePrismMesh(layerMeshes, "mergedLayers.vtk");
    }

  } else if (imethod == 4) {
    // (os.path.basename(surfaceMeshName), workingDir,  prismName, nextBaseMesh, thickness)
    std::cout << "Length: " << argc << " arg2 " << argv[2] << " arg3 " << argv[3] << " arg4 " << argv[4] << " arg5 " << argv[5] << std::endl;

    std::cout << "Method 4" << std::endl;

    // Get the working dir
    std::string workingDir = std::string(argv[3]);

    // Get the faces from this file
    std::ostringstream offInputName;
    offInputName << workingDir << "/" << argv[2];
    MyMesh inputMesh = readMesh(offInputName.str());
    std::cout << "InputMesh: " << offInputName.str() << std::endl;

    ScalarType amount = std::atof(argv[6]);
    std::cout << "Scale: " << amount << std::endl;
    // This mesh is the extruded mesh
    MyMesh extMesh = getScaledMesh(inputMesh, amount);
    //std::cout << "NewVerts: " << extMesh.n_vertices() << std::endl;
    std::cout << "NewBaseMesh: " << argv[5] << std::endl;

    std::ostringstream offOutputName;
    offOutputName << workingDir << "/" << argv[5];

    writeOFFMesh(extMesh, offOutputName.str()); 

    std::cout << "Writing Volume Mesh: " << argv[4] << std::endl;
    std::vector<MyMesh*> meshes;
    meshes.push_back(&inputMesh);
    meshes.push_back(&extMesh);

    std::ostringstream unstrOutputName;
    unstrOutputName << workingDir << "/" << argv[4];
    WriteUnstr(meshes, unstrOutputName.str().c_str());

    std::string argv4(argv[5]);
    
    //std::cout << "String: " <<  argv4.substr(0, argv4.length()- 4) << std::endl;
    std::string visName = argv4.substr(0, argv4.length()- 4);
    visName.append(".stl");

    std::ostringstream visOutputName;
    visOutputName << workingDir << "/" << visName;
    writeOFFMesh(extMesh, visOutputName.str()); 
  }
  else {
    std::cout << "Unknown method parameter... exiting" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return EXIT_SUCCESS;
}
