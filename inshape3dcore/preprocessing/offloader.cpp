#include <offloader.h>
#include <3dmodel.h>
#include <fstream>
#include <sstream>
#include <string.h>

namespace i3d {

OffLoader::OffLoader(void)
{

}//end constructor

OffLoader::~OffLoader(void)
{

}//end deconstructor

void OffLoader::readModelFromFile(Model3D *pModel,const char *strFileName)
{

  std::string n(strFileName);

  std::string fileType;

  // Function: istream &read(char *buf, streamsize num)
  // Read in <num> chars from the invoking stream
  // into the buffer <buf>
  ifstream in(n, ios::in);

  if(!in)
  {
    cout << "Cannot open file: "<< n << endl;
    std::exit(EXIT_FAILURE);
  }

  char buf[1024];

  int iVertices;
  int iFace;

  std::vector<Vec3> vertices;
  std::vector<tObjFace> faces;


  // First line should be the keyword OFF
  in>>fileType;
  in.getline(buf, 1024);
  
  std::cout << "file type: " << fileType << std::endl;

  // Second line contains vertex_count face_count edge_count
  in >> iVertices;
  in >> iFace;

  // Skip the rest of the line
  in.getline(buf, 1024);

  std::cout << "No vertices: " << iVertices << std::endl;
  std::cout << "No faces: " << iFace << std::endl;

  // read the vertices
  for(int i(0); i < iVertices; ++i)
  {

    Vec3 v;
    in >> v.x >> v.y >> v.z;

    vertices.push_back(v);

    std::cout << "Vertices: " << v;

    // Skip the rest of the line
    in.getline(buf, 1024);

  }
  
  // read the faces
  for(int i(0); i < iFace; ++i)
  {
    Vec3 v;
    int nFace;
    in >> nFace;

    tObjFace face;

    //>> v.x >> v.y >> v.z;
    if (nFace != 3)
    {
      std::cout << "Only faces with 3 vertices are supported" << std::endl;  
      std::exit(EXIT_FAILURE);
    }

    for (int i(0); i < nFace; ++i)
    {
      in >> face.VertexIndex[i];
      std::cout << face.VertexIndex[i] << " ";
    }

    std::cout << std::endl;

    faces.push_back(face);

    // Skip the rest of the line
    in.getline(buf, 1024);
  }

  in.close();

  Mesh3D mesh;
  //assign number of vertices
  mesh.vertices_ = vertices;

  mesh.numVerts_ = vertices.size();

  mesh.numFaces_ = faces.size();
  mesh.faces_.reserve(faces.size());

  for (unsigned int i = 0; i<faces.size(); i++)
  {
    mesh.faces_.push_back(TriFace(faces[i].VertexIndex));
  }//end for

  mesh.numTexCoords_ = 0;
  mesh.texCoords_.reserve(mesh.numTexCoords_);

  mesh.calcVertexNormals();
  pModel->meshes_.push_back(mesh);

}//end ReadModelFromFile

}
