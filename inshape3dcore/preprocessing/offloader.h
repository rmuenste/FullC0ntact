#ifndef _OFFLOADER_H_
#define _OFFLOADER_H_

#include <vector>
#include <vector3.h>
#include <vector2.h>

namespace i3d {

class Model3D;
class Mesh3D;

using namespace std;

///@cond HIDDEN_SYMBOLS
typedef struct
{

	int VertexIndex[3];
	int TexIndex[3];

}tObjFace;
///@cond 

/**
* @brief A class that loads .off files
*
* A class that loads .off files and converts them into a C3DModel
* the internally used file format in this library
*
*/
class OffLoader
{
public:
  OffLoader(void);
  ~OffLoader(void);

  /* reads the .obj file specified in strFileName */
  void readModelFromFile(Model3D *pModel, const char *strFileName);

};

}

#endif
