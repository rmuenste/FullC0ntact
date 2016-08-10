#include "tubeloader.h"
#include "3dmodel.h"
#include <fstream>
#include <sstream>
#include <string.h>

namespace i3d {

  CTubeLoader::CTubeLoader(void)
  {

    m_bUV = false;	

  }//end constructor

  CTubeLoader::~CTubeLoader(void)
  {

  }//end deconstructor

  void CTubeLoader::ReadModelFromFile(Model3D *pModel,const char *strFileName)
  {

    std::ifstream in(strFileName);

    char strLine[256];
    std::string first;

    m_pModel = pModel;

    while(!in.eof())
    {

      in>>first;

      if(first == std::string("#"))
      {
        in.getline(strLine,256);
        continue;
      }
      //case: Vertex
      else if(first == std::string("v"))
        ReadVertex(in,strLine);
      //case: TexCoord
      else if(first == std::string("vt"))
      {
        ReadTexCoord(in, strLine);
        m_bUV = true;
      }
      //case: Face
      else if(first == std::string("f"))
      {
        if(!m_bUV)
        {
          ReadFace(in, strLine);
        }
        else
          ReadFaceTex(in, strLine);
      }
      //default
      else
        in.getline(strLine,256);
    }//end while

    std::cout <<"Number of vertices: "<<m_pVertices.size()<<std::endl;
    std::cout <<"Number of faces: "<<m_pFaces.size()<<std::endl;
    int ivertsTotal = m_pVertices.size();
    int ifacesTotal = m_pFaces.size();
    int ivertsMesh = 1343;
    int ifacesMesh = 2686;
    int iTubes = 176;
    for(int imesh=0;imesh<iTubes;imesh++)
    {
      Mesh3D mesh;
      mesh.numVerts_=ivertsMesh;
      mesh.numFaces_=ifacesMesh;
      mesh.numTexCoords_=0;
      mesh.vertices_.reserve(ivertsMesh);
      mesh.faces_.reserve(ifacesMesh);
      for(unsigned int i=0;i<ivertsMesh;i++)
      {
        mesh.vertices_[i]=m_pVertices[imesh*ivertsMesh+i];
      }
      for(unsigned int i=0;i<ifacesMesh;i++)
      {
        int vertexIndex[3];
        vertexIndex[0] = m_pFaces[imesh*ifacesMesh+i].VertexIndex[0] - imesh*ivertsMesh;
        vertexIndex[1] = m_pFaces[imesh*ifacesMesh+i].VertexIndex[1] - imesh*ivertsMesh;
        vertexIndex[2] = m_pFaces[imesh*ifacesMesh+i].VertexIndex[2] - imesh*ivertsMesh;
        mesh.faces_[i].InitFace(vertexIndex);
      }//end for
      m_pModel->meshes_.push_back(mesh);
    }// end for imesh
    //assign number of vertices
  }//end ReadModelFromFile

  void CTubeLoader::ReadVertex(std::ifstream &in, char strLine[])
  {

    Vec3 vec;
    in >> vec.x;
    in >> vec.z;
    in >> vec.y;
    in.getline(strLine,256);
    m_pVertices.push_back(vec);

  }//end ReadVertex

  void CTubeLoader::ReadFace(std::ifstream &in, char strLine[])
  {

    tObjFace Face;

    for(int i = 0; i < 3; i++)
    {
      in >> Face.VertexIndex[i];
      Face.VertexIndex[i]--;
    }

    in.getline(strLine, 256);
    m_pFaces.push_back(Face);

  }//end ReadFace

  void CTubeLoader::ReadTexCoord(std::ifstream &in, char strLine[])
  {

    Vec2 vec;
    in >> vec.x;
    in >> vec.y;
    m_pTexCoords.push_back(vec);
    //cout<<m_pTexCoords.size()<<" "<<vec.x<<" "<<vec.y<<endl;
    in.getline(strLine,256);

  }//end ReadTexCoord

  void CTubeLoader::ReadFaceTex(std::ifstream &in, char strLine[])
  {
    tObjFace Face;

    std::string s;

    std::basic_string<char> vertIndex;
    std::basic_string<char> texIndex;
    int vi;
    int ti;

    for(int i = 0; i < 3; i++)
    {

      // Format for a face is vertexIndex/texture index vertexIndex/textureIndex vertexIndex/texture index 
      in >> s;

      // find separator 
      std::basic_string<char>::size_type index = s.find("/");

      // extract indices
      vertIndex = s.substr(0,index);
      texIndex = s.substr(index+1,s.size()-1);

      // convert indices from string to int
      std::istringstream VertStream(vertIndex);
      std::istringstream TexStream(texIndex);

      VertStream >> vi;
      TexStream  >> ti;

      //assign the values to the face structure
      Face.VertexIndex[i] = vi-1;
      Face.TexIndex[i]    = ti-1;		

    }



    //go to next line
    in.getline(strLine, 256);
    m_pFaces.push_back(Face);

  }//end ReadFaceTex

  const VertArray& CTubeLoader::GetVertices() const
  {

    return m_pVertices;

  }//end GetVertices

  const FaceArray& CTubeLoader::GetFaces() const
  {

    return m_pFaces;

  }//end GetVertices


  bool CTubeLoader::HasUV(void) const
  {
    return m_bUV;
  }

  const TexCoordArray& CTubeLoader::GetTexCoords(void) const
  {
    return m_pTexCoords;
  }//end GetTexCoords

}
