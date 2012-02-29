#include "genericloader.h"
#include "3dsloader.h"
#include "objloader.h"
#include <string>
#include <iostream>

using namespace std;

namespace i3d {

CGenericLoader::CGenericLoader(void)
{
}

CGenericLoader::~CGenericLoader(void)
{

}

	/* reads the .obj file specified in strFileName */
void CGenericLoader::ReadModelFromFile(C3DModel *pModel,const char *strFileName)
{
	string sFile(strFileName);

	size_t pos = sFile.find(".");

	string sType = sFile.substr(pos);

	if(sType == ".obj")
	{
		CObjLoader Loader;
		Loader.ReadMultiMeshFromFile(pModel,strFileName);
	}//end if
	else if(sType == ".3DS" || sType ==".3ds")
	{
		C3DSLoader Loader;
		Loader.ReadModelFromFile(pModel,strFileName);
	}//end if
	else
	{
	}//end else

}//end ReadModelFromFile

}