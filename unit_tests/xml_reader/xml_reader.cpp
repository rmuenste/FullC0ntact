#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <worldparameters.h>
#include <reader.h>

using namespace i3d;

int main()
{
  
  FileParserXML myReader;
  WorldParameters dataFileParams;
  std::string fileName("start/sample.xml");

  //Get the name of the mesh file from the
  //configuration data file.
  myReader.parseDataXML(dataFileParams, fileName);

  return EXIT_SUCCESS;
}
