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

namespace i3d {

/**
* @brief A reader/parser for data files that allow the user set application parameters
*
* A reader/parser for data files that allow the user set application parameters
*/
class CReader
{
public:
/**
* Creates a CReader
*
*/
	CReader(void);
	~CReader(void);
  
/**  
*  Reads the data file of a collision application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
	void ReadParameters(std::string strFileName, CWorldParameters &parameters);
  
/**  
*  Reads the data file of a deform application and parses the parameters
*  @param strFileName The data file 
*  @param parameters  The class that stores the parameters
*/
  void ReadParametersDeform(std::string strFileName, CDeformParameters &parameters);  

private:

/**
*  Reads an int token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/
	bool ReadNextTokenInt(std::ifstream &in,std::string token,int &value);
/**
*  Reads a string token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/  
	bool ReadNextTokenString(std::ifstream &in,std::string token,std::string &value);
/**
*  Reads a real token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param value The variable that stores the value read from the file
*/  
  bool ReadNextTokenReal(std::ifstream &in,std::string token,Real &value);
/**
*  Reads a vector token from the file
*
* @param in The stream from which we read
* @param token The token we should find
* @param vec The variable that stores the value read from the file
*/  
  bool ReadNextTokenVector(std::ifstream &in,std::string token,VECTOR3 &vec);

/**
*  Reads the rigid bodies from the rigid body section in the data file
*  @param in The stream from which we read 
*  @param nBodies The number of bodies to be read
*  @param vBodies The vector that stores the rigid body information
*/
  bool ReadRigidBodySection(std::ifstream &in, int nBodies, std::vector<sRigidBody> &vBodies);
  
/**
*  Reads a rigid body from the rigid body section in the data file
*  @param in The stream from which we read 
*  @param body The rigid body structure that stores the information
*/
  bool ReadRigidBody(std::ifstream &in, sRigidBody &body);    

};

}

#endif
