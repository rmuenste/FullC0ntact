#ifndef CTUBELOADER_H
#define CTUBELOADER_H


#include <vector>
#include <vector3.h>
#include <vector2.h>
#include <dynamicarray.h>

namespace i3d {

class Model3D;
class Mesh3D;

///@cond HIDDEN_SYMBOLS

typedef struct
{

	int VertexIndex[3];
	int TexIndex[3];

}tObjFace;

typedef std::vector<Vec3> VertArray;
typedef std::vector<tObjFace>  FaceArray;
typedef std::vector<CVector2<Real> > TexCoordArray;
typedef CDynamicArray<Vector3<Real> > Vec3Array;
typedef CDynamicArray<CVector2<Real> > Vec2Array;

class CTubeLoader 
{
  public:
    CTubeLoader(void);
    ~CTubeLoader(void);

    /* reads the .obj file specified in strFileName */
    void ReadModelFromFile(Model3D *pModel,const char *strFileName);
    void ReadModelFromFile(char *strFileName){};

    const VertArray& GetVertices() const;
    const FaceArray& GetFaces() const;
    const Vec3Array& GetNormals() const;

    const TexCoordArray& GetTexCoords(void) const;


    bool HasUV(void) const;

  private:

    void ReadVertex(std::ifstream &in, char strLine[]);

    void ReadFace(std::ifstream &in, char strLine[]);

    void ReadTexCoord(std::ifstream &in, char strLine[]);

    void ReadFaceTex(std::ifstream &in, char strLine[]);

    /* private member variables */

    VertArray m_pVertices;

    TexCoordArray m_pTexCoords;

    FaceArray m_pFaces;

    bool m_bUV;

    Model3D *m_pModel;
};

///@cond 

}

#endif
