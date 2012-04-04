#include "reader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <rigidbodyio.h>


namespace i3d {

CReader::CReader(void)
{
}

CReader::~CReader(void)
{
}
  
void CReader::ReadParametersDeform(std::string strFileName, CDeformParameters &parameters)
{
  using namespace std;
  string first;
  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    exit(0);
  }
  
  if(!ReadNextTokenInt(in,string("nBodies"),parameters.m_iBodies))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"nBodies"<<endl;
    exit(0);
  }  

  ReadRigidBodySection(in, parameters.m_iBodies, parameters.m_vRigidBodies); 
  
  in.close();

}  

void CReader::ReadParameters(std::string strFileName, CWorldParameters &parameters)
{
	using namespace std;
	string first;
	ifstream in(strFileName.c_str());

	if(!in.is_open())
	{
		std::cerr<<"Unable to open file: "<<strFileName<<endl;
		exit(0);
	}

	if(!ReadNextTokenInt(in,string("startType"),parameters.m_iStartType))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"startType"<<endl;
		exit(0);
	}
	

  if(!ReadNextTokenInt(in,string("liquidSolid"),parameters.m_iLiquidSolid))
  {
    std::cerr<<"bad file format"<<strFileName 
             <<" could not find parameter: "<<"liquidSolid"<<endl;
  }
  
	if(!ReadNextTokenString(in,string("solution"),parameters.m_sSolution))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"solution"<<endl;
		exit(0);
	}


	if(!ReadNextTokenInt(in,string("nBodies"),parameters.m_iBodies))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"nBodies"<<endl;
		exit(0);
	}


	if(!ReadNextTokenInt(in,string("bodyInit"),parameters.m_iBodyInit))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"bodyInit"<<endl;
		exit(0);
	}


	if(!ReadNextTokenString(in,string("bodyFile"),parameters.m_sBodyFile))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"bodyFile"<<endl;
		exit(0);
	}

	
	if(!ReadNextTokenReal(in,string("defaultDensity"),parameters.m_dDefaultDensity))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"defaultDensity"<<endl;
		exit(0);
	}

  if(!ReadNextTokenReal(in,string("liquidDensity"),parameters.m_dDensityMedium))
  {
    std::cerr<<"bad file format"<<strFileName 
             <<" could not find parameter: "<<"liquidDensity"<<endl;
  }

	if(!ReadNextTokenReal(in,string("defaultRadius"),parameters.m_dDefaultRadius))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"defaultRadius"<<endl;
		exit(0);
	}


	if(!ReadNextTokenVector(in,string("gravity"),parameters.m_vGrav))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"gravity"<<endl;
		exit(0);
	}


	if(!ReadNextTokenInt(in,string("totalTimesteps"),parameters.m_iTotalTimesteps))
	{
		std::cerr<<"bad file format: "<<strFileName
			<<" could not find parameter: "<<"totalTimesteps"<<endl;
		exit(0);
	}

  if(!ReadNextTokenReal(in,string("timeStep"),parameters.m_dTimeStep))
  {
    std::cerr<<"bad file format"<<strFileName 
             <<" could not find parameter: "<<"timeStep"<<endl;
  }

  if(!ReadNextTokenInt(in,string("solverType"),parameters.m_iSolverType))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"solverType"<<endl;
    exit(0);
  }

  if(!ReadNextTokenInt(in,string("lcpSolverIterations"),parameters.m_iMaxIterations))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"lcpSolverIterations"<<endl;
    exit(0);
  }

  if(!ReadNextTokenInt(in,string("collPipelineIterations"),parameters.m_iPipelineIterations))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"collPipelineIterations"<<endl;
    exit(0);
  }
  //	
	// cout<<"startType = "<<parameters.m_iStartType<<endl;	
	// cout<<"solution = "<<parameters.m_sSolution<<endl;	
	// cout<<"nBodies = "<<parameters.m_iBodies<<endl;	
	// cout<<"bodyInit = "<<parameters.m_iBodyInit<<endl;	
	// cout<<"bodyFile = "<<parameters.m_sBodyFile<<endl;	
	// cout<<"defaultDensity = "<<parameters.m_dDefaultDensity<<endl;	
	// cout<<"defaultRadius = "<<parameters.m_dDefaultRadius<<endl;	
	// cout<<"gravity = "<<parameters.m_vGrav;	
	// cout<<"totalTimesteps = "<<parameters.m_iTotalTimesteps<<endl;
	// cout<<"lcpSolverIterations = "<<parameters.m_iMaxIterations<<endl;
	// cout<<"collPipelineIterations = "<<parameters.m_iPipelineIterations<<endl;

  ReadRigidBodySection(in, parameters.m_iBodies, parameters.m_vRigidBodies);
  
	in.close();

}

bool CReader::ReadNextTokenVector(std::ifstream &in,std::string token,VECTOR3 &vec)
{
	using namespace std;
	bool found=false;
	char strLine[256];
	string equal;
	while(!in.eof())
	{
		string word;
		in>>word;
		if(word==token)
		{
			found=true;
			in >> equal >> vec.x >> vec.y >> vec.z;
			break;
		}
		else 
		{
			in.getline(strLine,256);
		}
	}
	return found;
}

bool CReader::ReadNextTokenReal(std::ifstream &in,std::string token,Real &value)
{
	using namespace std;
	bool found=false;
	char strLine[256];
	string equal;
	while(!in.eof())
	{
		string word;
		in>>word;
		if(word==token)
		{
			found=true;
			in >> equal >> value;
			break;
		}
		else 
		{
			in.getline(strLine,256);
		}
	}
	return found;

}


bool CReader::ReadNextTokenInt(std::ifstream &in,std::string token,int &value)
{
	using namespace std;
	bool found=false;
	char strLine[256];
	string equal;
	while(!in.eof())
	{
		string word;
		in>>word;
		if(word==token)
		{
			found=true;
			in >> equal >> value;
			break;
		}
		else 
		{
			in.getline(strLine,256);
		}
	}
	return found;

}

bool CReader::ReadNextTokenString(std::ifstream &in,std::string token,std::string &value)
{
	using namespace std;
	bool found=false;
	char strLine[256];
	string equal;
	while(!in.eof())
	{
		string word;
		in>>word;
		if(word==token)
		{
			found=true;
			in >> equal >> value;
			break;
		}
		else 
		{
			in.getline(strLine,256);
		}
	}
	return found;

}

bool CReader::ReadRigidBodySection(std::ifstream &in, int nBodies, std::vector<sRigidBody> &vBodies)
{
  
  using namespace std;
  char strLine[256];  
  
  in.getline(strLine,256);
  in.getline(strLine,256);    
  
  for(int i=1;i<=nBodies;i++)
  {
    sRigidBody body;
    ReadRigidBody(in,body);
    vBodies.push_back(body);
  }
  
  return true;
  
}

bool CReader::ReadRigidBody(std::ifstream &in, sRigidBody &body)
{
  
  using namespace std;
  bool found=false;
  char strLine[256];

  std::string fileName;  

  if(in.eof())return false;
  
   in >> body.m_iShape;
   in.getline(strLine,256);
  
   in >> body.m_vCOM.x >>body.m_vCOM.y>>body.m_vCOM.z;
   in.getline(strLine,256);  

   in >> body.m_vVelocity.x >>body.m_vVelocity.y>>body.m_vVelocity.z;
   in.getline(strLine,256);  

   in >> body.m_vAngVel.x >>body.m_vAngVel.y>>body.m_vAngVel.z;
   in.getline(strLine,256);  

   in >> body.m_vAngle.x >>body.m_vAngle.y>>body.m_vAngle.z;
   in.getline(strLine,256);  
   
   in >> body.m_vForce.x >>body.m_vForce.y>>body.m_vForce.z;
   in.getline(strLine,256);  

   in >> body.m_vTorque.x >>body.m_vTorque.y>>body.m_vTorque.z;
   in.getline(strLine,256);  

   in >> body.m_Extends[0]>>body.m_Extends[1]>>body.m_Extends[2];
   in.getline(strLine,256);

   in >> body.m_vUVW[0].x >> body.m_vUVW[0].y >> body.m_vUVW[0].z;
   in.getline(strLine,256);  

   in >> body.m_vUVW[1].x >> body.m_vUVW[1].y >> body.m_vUVW[1].z;
   in.getline(strLine,256);  
   
   in >> body.m_vUVW[2].x >> body.m_vUVW[2].y >> body.m_vUVW[2].z;
   in.getline(strLine,256);  

   in >> body.m_dDensity;
   in.getline(strLine,256);

   in >> body.m_Restitution;
   in.getline(strLine,256);

   in >> body.m_iAffectedByGravity;
   in.getline(strLine,256);

   body.m_bMatrixAvailable = false;
   
   in >> fileName;
   
   strcpy(body.m_strFileName,fileName.c_str());

  return true;  

}

}
