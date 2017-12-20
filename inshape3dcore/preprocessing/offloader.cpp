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

  Mesh3D mesh;

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

  int iVertex;
  int iFace;

  // First line should be the keyword OFF
  in>>fileType;
  in.getline(buf, 1024);

  std::cout << "file type: " << fileType << std::endl;

  // Second line contains vertex_count face_count edge_count
  in >> iVertex;
  in >> iFace;

  // Skip the rest of the line
  in.getline(buf, 1024);

  std::cout << "No vertices: " << iVertex << std::endl;
  std::cout << "No faces: " << iFace << std::endl;

  in.close();

//  while(!in.eof())
//  {    
//    in>>first;
//    
//    if(first == string("#"))
//    {
//      in.getline(strLine,256);
//      continue;
//    }
//    else if(first == string(""))
//    {
//      in.getline(strLine,256);
//      continue;
//    }
//    //case: Vertex
//    else if (first == string("v"))
//    {
//      readVertex(in,strLine);
//    }
//    //case: Face
//    else if(first == string("f"))
//    {
//      readFace(in, strLine);
//    }
//    //default
//    else
//      in.getline(strLine,256);
//        
//  }//end while


}//end ReadModelFromFile

}
