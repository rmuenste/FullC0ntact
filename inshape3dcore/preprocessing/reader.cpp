#include "reader.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <rigidbodyio.h>
#include <rigidbody.h>


namespace i3d {

CReader::CReader(void)
{
}

CReader::~CReader(void)
{
}
  
void CReader::ReadParametersDeform(std::string strFileName, DeformParameters &parameters)
{
  using namespace std;
  string first;
  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    exit(0);
  }
  
  if(!ReadNextTokenInt(in,string("nBodies"),parameters.bodies_))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"nBodies"<<endl;
    exit(0);
  }  

  ReadRigidBodySection(in, parameters.bodies_, parameters.rigidBodies_); 
  
  in.close();

}  

void CReader::ReadParameters(std::string strFileName, WorldParameters &parameters)
{
  using namespace std;
  string first;
  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    exit(0);
  }

  char strLine[1024];
  string equal;
  bool found=false;
  while(!in.eof())
  {
    string word;
    //parse first word in line
    in>>word;
    std::transform(word.begin(), word.end(), word.begin(), ::tolower);
    if(word=="starttype")
    {
      ReadInt(in,parameters.startType_);
      in.getline(strLine,1024);   
    }
    else if(word=="liquidsolid")
    {
      //parse
      ReadInt(in,parameters.liquidSolid_);      
      in.getline(strLine,1024);
    }
    else if(word=="solution")
    {
      //parse
      ReadString(in,parameters.solutionFile_);            
      in.getline(strLine,1024);
    }    
    else if(word=="nbodies")
    {
      //parse
      ReadInt(in,parameters.bodies_);      
      in.getline(strLine,1024);
    }    
    else if(word=="bodyinit")
    {
      //parse
      ReadInt(in,parameters.bodyInit_);            
      in.getline(strLine,1024);
    }
    else if(word=="bodyfile")
    {
      //parse
      ReadString(in,parameters.bodyConfigurationFile_);                  
      in.getline(strLine,1024);
    }        
    else if(word=="defaultdensity")
    {
      //parse
      ReadReal(in,parameters.defaultDensity_);
      in.getline(strLine,1024);
    }        
    else if(word=="liquiddensity")
    {
      //parse
      ReadReal(in,parameters.densityMedium_);      
      in.getline(strLine,1024);
    }        
    else if(word=="defaultradius")
    {
      //parse
      ReadReal(in,parameters.defaultRadius_);      
      in.getline(strLine,1024);
    }
    else if(word=="gravity")
    {
      //parse
      ReadVector(in,parameters.gravity_);      
      in.getline(strLine,1024);
    }            
    else if(word=="totaltimesteps")
    {
      //parse
      ReadInt(in,parameters.nTimesteps_);
      in.getline(strLine,1024);
    }            
    else if(word=="timestep")
    {
      //parse
      ReadReal(in,parameters.timeStep_);            
      in.getline(strLine,1024);
    }            
    else if(word=="solvertype")
    {
      //parse
      ReadInt(in,parameters.solverType_);      
      in.getline(strLine,1024);
    }            
    else if(word=="lcpsolveriterations")
    {
      //parse
      ReadInt(in,parameters.maxIterations_);            
      in.getline(strLine,1024);
    }
    else if(word=="collpipelineiterations")
    {
      //parse
      ReadInt(in,parameters.pipelineIterations_);                  
      in.getline(strLine,1024);
    }
    else if(word=="[rigidbodysection]")
    {
      in.getline(strLine,1024);            
      break;
    }
    else 
    {
      //skip contents
      in.getline(strLine,1024);
    }
  }

//   cout<<"startType = "<<parameters.m_iStartType<<endl; 
//   cout<<"solution = "<<parameters.m_sSolution<<endl; 
//   cout<<"nBodies = "<<parameters.m_iBodies<<endl;  
//   cout<<"bodyInit = "<<parameters.m_iBodyInit<<endl; 
//   cout<<"bodyFile = "<<parameters.m_sBodyFile<<endl; 
//   cout<<"defaultDensity = "<<parameters.m_dDefaultDensity<<endl; 
//   cout<<"defaultRadius = "<<parameters.m_dDefaultRadius<<endl; 
//   cout<<"gravity = "<<parameters.m_vGrav;  
//   cout<<"totalTimesteps = "<<parameters.m_iTotalTimesteps<<endl;
//   cout<<"lcpSolverIterations = "<<parameters.m_iMaxIterations<<endl;
//   cout<<"collPipelineIterations = "<<parameters.m_iPipelineIterations<<endl;
  
  ReadRigidBodySection(in, parameters.bodies_, parameters.rigidBodies_);

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

void CReader::ReadVector(std::ifstream &in,VECTOR3 &vec)
{
  using namespace std;
  bool found=false;
  char strLine[1024];
  string equal; 
  in >> equal >> vec.x >> vec.y >> vec.z;      
}

void CReader::ReadReal(std::ifstream &in,Real &value)
{
  using namespace std;
  bool found=false;
  char strLine[1024];
  string equal;
  in >> equal >> value;
}


void CReader::ReadInt(std::ifstream &in,int &value)
{
  using namespace std;
  bool found=false;
  char strLine[1024];
  string equal;
  in >> equal >> value;
}

void CReader::ReadString(std::ifstream &in,std::string &value)
{
  using namespace std;
  bool found=false;
  char strLine[256];
  string equal;
  in >> equal >> value;
}


bool CReader::ReadRigidBodySection(std::ifstream &in, int nBodies, std::vector<sRigidBody> &vBodies)
{
  
  using namespace std;
  char strLine[256];  
    
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

   in >> body.extents_[0]>>body.extents_[1]>>body.extents_[2];
   in.getline(strLine,256);

   in >> body.m_vUVW[0].x >> body.m_vUVW[0].y >> body.m_vUVW[0].z;
   in.getline(strLine,256);  

   in >> body.m_vUVW[1].x >> body.m_vUVW[1].y >> body.m_vUVW[1].z;
   in.getline(strLine,256);  
   
   in >> body.m_vUVW[2].x >> body.m_vUVW[2].y >> body.m_vUVW[2].z;
   in.getline(strLine,256);  

   in >> body.m_dDensity;
   in.getline(strLine,256);
   
   if(body.m_iShape == RigidBody::MESH)
   {
     in >> body.m_dVolume;
     in.getline(strLine,256);     
     memset(body.m_dTensor,0.0,9*sizeof(Real));
     in >> body.m_dTensor[0] >> body.m_dTensor[4] >> body.m_dTensor[8];     
     in.getline(strLine,256);               
   }

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
