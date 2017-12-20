#include <genericloader.h>
#include <3dsloader.h>
#include <objloader.h>
#include <offloader.h>
#include <string>
#include <iostream>

using namespace std;

namespace i3d {

GenericLoader::GenericLoader(void)
{

}

GenericLoader::~GenericLoader(void)
{

}

/* reads the .obj file specified in strFileName */
void GenericLoader::readModelFromFile(Model3D *pModel,const char *strFileName)
{
	string sFile(strFileName);

	size_t pos = sFile.find(".");

	string sType = sFile.substr(pos);

	if(sType == ".obj")
	{
		ObjLoader Loader;
		Loader.readMultiMeshFromFile(pModel,strFileName);
	}//end if
	else if(sType == ".3DS" || sType ==".3ds")
	{
		C3DSLoader Loader;
		Loader.ReadModelFromFile(pModel,strFileName);
	}//end if
  else if (sType == ".off")
  {
    C3DSLoader Loader;
    Loader.ReadModelFromFile(pModel, strFileName);
  }//end if
	else
	{
    std::cout << "File type " << sType << " not recognized." << std::endl;
    std::exit(EXIT_FAILURE);
	}//end else

}//end ReadModelFromFile

}