/*
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public
License version 2 as published by the Free Software Foundation.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.

You should have received a copy of the GNU Library General Public License
along with this library; see the file COPYING.LIB.  If not, write to
the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
Boston, MA 02110-1301, USA.
*/

#ifndef CREADER_H
#define CREADER_H

#include <string>
#include <fstream>
#include <worldparameters.h>
#include <mymath.h>
#include <vector3.h>
#include <deformparameters.h>
#include <rapidxml.hpp>


namespace i3d {

class FileParserJson {

public:

  FileParserJson() {};

  ~FileParserJson() {};

  void parseData(WorldParameters &params, const std::string &fileName);

};

class FileParserXML {

public:


  FileParserXML() {};

  ~FileParserXML() {};

  void parseDataXML(WorldParameters &params, const std::string &fileName);

};

/**
* @brief A reader/parser for data files that allow the user set application parameters
*
* A reader/parser for data files that allow the user set application parameters
*/
class Reader
{
public:
/**
* Creates a CReader
*
*/
	Reader(void);
	~Reader(void);
  
/**  
*  Reads the data file of a collision application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
	void readParameters(std::string strFileName, WorldParameters &parameters);
  
/**  
*  Reads the data file of a deform application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
  void readParametersDeform(std::string strFileName, DeformParameters &parameters);  

  std::vector<BodyStorage> read_sol_rb(int idx);

private:

/**
*  Reads an int token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/
	bool readNextTokenInt(std::ifstream &in,std::string token,int &value);
/**
*  Reads a string token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/  
	bool readNextTokenString(std::ifstream &in,std::string token,std::string &value);
/**
*  Reads a real token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/  
  bool readNextTokenReal(std::ifstream &in,std::string token,Real &value);
/**
*  Reads a vector token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param vec The variable that stores the value read from the file
*/  
  bool readNextTokenVector(std::ifstream &in,std::string token,VECTOR3 &vec);

/**
*  Reads the rigid bodies from the rigid body section in the data file
*  @param in The stream from which we read 
*  @param nBodies The number of bodies to be read
*  @param vBodies The vector that stores the rigid body information
*/
  bool readRigidBodySection(std::ifstream &in, int nBodies, std::vector<BodyStorage> &vBodies);
  
/**
*  Reads a rigid body from the rigid body section in the data file
*  @param in The stream from which we read 
*  @param body The rigid body structure that stores the information
*/
  bool readRigidBody(std::ifstream &in, BodyStorage &body);    
  
/**
*  Reads an int token from the file
*
* @param in The stream from which we read
* @param value The variable that stores the value read from the file
*/  
  void readVector(std::ifstream &in,VECTOR3 &vec);

/**
*  Reads a real token from the file
*
* @param in The stream from which we read
* @param value The variable that stores the value read from the file
*/    
  void readReal(std::ifstream &in,Real &value);

/**
*  Reads an int token from the file
*
* @param in The stream from which we read
* @param value The variable that stores the value read from the file
*/  
  void readInt(std::ifstream &in,int &value);

/**
*  Reads a string token from the file
*
* @param in The stream from which we read
* @param value The variable that stores the value read from the file
*/    
  void readString(std::ifstream &in,std::string &value);

  /**
  *  Reads the domain extents from the data file
  *
  * @param in The stream from which we read
  * @param extents The array where the domain extents will be stored
  */
  void readExtents(std::ifstream &in, Real *extents);

  
};

/**
* @brief A reader/parser for e3d input data
*
* A reader/parser for e3d input data
*/
class E3dReader
{
public:
/**
* Creates a E3dReader
*
*/
  E3dReader() = default;
  ~E3dReader() {};
  
/**  
*  Reads the data file of a deform application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
  void readE3dFile(std::string strFileName);  

private:

/**
*  Reads an int token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/
	bool readElement(std::ifstream &in);
  
};

/**
* @brief A reader/parser for e3d input data
*
* A reader/parser for e3d input data
*/
class OffsReader
{
public:
/**
* Creates a E3dReader
*
*/
  OffsReader() = default;
  ~OffsReader() {};
  
/**  
*  Reads the data file of a deform application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
	void readParameters(std::string strFileName, WorldParameters &parameters);

private:
  
};

}

#endif
