#include "reader.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <rigidbodyio.h>
#include <rigidbody.h>
#include <rapidxml_utils.hpp>


namespace i3d {

void FileParserXML::parseDataXML(WorldParameters &params, const std::string &fileName)
{

  using namespace rapidxml;
  file<> xmlFile(fileName.c_str());

  xml_document<> doc;
  doc.parse<0>(xmlFile.data());

  //std::cout << "The first node is" << doc.first_node()->name() << "\n";
  xml_node<> *root = doc.first_node();

  xml_node<> *n = root->first_node("ApplicationSettings");
  //std::cout << "Name of the current node. " << n->name() << "\n";
  xml_attribute<> *att = n->first_attribute();
  while (att)
  {

    std::string word(att->name());
    std::transform(word.begin(), word.end(), word.begin(), ::tolower);
    //std::cout << "Node has the following attribute: " << std::endl;
    //std::cout << "Name of the attribute: " << att->name() << std::endl;
    //std::cout << "Value of the attribute: " << att->value() << std::endl;

    if (word == "starttype")
    {
      params.startType_ = atoi(att->value());
    }
    else if (word == "liquidsolid")
    {
      params.liquidSolid_ = atoi(att->value());
    }
    else if (word == "solution")
    {
      params.solutionFile_= std::string(att->value());
    }
    else if (word == "nbodies")
    {
      params.bodies_ = atoi(att->value());
    }
    else if (word == "bodyinit")
    {
      params.bodyInit_ = atoi(att->value());
    }
    else if (word == "bodyfile")
    {
      params.bodyConfigurationFile_ = std::string(att->value());
    }
    else if (word == "defaultdensity")
    {
      params.defaultDensity_ = atof(att->value());
    }
    else if (word == "liquiddensity")
    {
      params.densityMedium_ = atof(att->value());
    }
    else if (word == "airfriction")
    {
      params.airFriction_ = atof(att->value());
    }
    else if (word == "defaultradius")
    {
      params.defaultRadius_ = atof(att->value());
    }
    else if (word == "gravity")
    {
      std::stringstream myStream(att->value());
      myStream >> params.gravity_.x >> params.gravity_.y >> params.gravity_.z;
    }
    else if (word == "totaltimesteps")
    {
      params.nTimesteps_ = atoi(att->value());
    }
    else if (word == "timestep")
    {
      params.timeStep_ = atof(att->value());
    }
    else if (word == "solvertype")
    {
      params.solverType_ = atoi(att->value());
    }
    else if (word == "lcpsolveriterations")
    {
      params.maxIterations_ = atoi(att->value());
    }
    else if (word == "collpipelineiterations")
    {
      params.pipelineIterations_ = atoi(att->value());
    }
    else if (word == "extents")
    {
      std::stringstream myStream(att->value());
      myStream >> params.extents_[0] >> params.extents_[1] >> params.extents_[2]
                >> params.extents_[3] >> params.extents_[4] >> params.extents_[5];
      params.setHasExtents(true);
    }

    att = att->next_attribute();

  }

  if(params.bodies_ == 0)
  {
    //std::cout << "number of rigid bodies: " << params.bodies_ << std::endl;
    return;
  }

  n = root->first_node("RigidBodyList");
  //std::cout << "Name of the current node: " << n->name() << "\n";
  
  if(n)
   n = n->first_node();
  else
  {
    return;
    //std::cerr << "Token RigidBodyList not found" << std::endl;    
  }
  
  for (; n; n = n->next_sibling())
  {
    att = n->first_attribute();

    BodyStorage body;

    while (att)
    {
      //std::cout << "Node has the following attribute: " << std::endl;
      //std::cout << "Name of the attribute: " << att->name() << std::endl;

      std::string name(att->name());
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);

      if (name == "type")
      {
        body.shapeId_ = atoi(att->value());
      }
      else if (name == "position")
      {
        std::stringstream myStream(att->value());
        myStream >> body.com_.x >> body.com_.y >> body.com_.z;
      }
      else if (name == "velocity")
      {
        std::stringstream myStream(att->value());
        myStream >> body.velocity_.x >> body.velocity_.y >> body.velocity_.z;
      }
      else if (name == "angularvelocity")
      {
        std::stringstream myStream(att->value());
        myStream >> body.angVel_.x >> body.angVel_.y >> body.angVel_.z;
      }
      else if (name == "orientation")
      {
        std::stringstream myStream(att->value());
        myStream >> body.angle_.x >> body.angle_.y >> body.angle_.z;
      }
      else if (name == "force")
      {
        std::stringstream myStream(att->value());
        myStream >> body.force_.x >> body.force_.y >> body.force_.z;
      }
      else if (name == "torque")
      {
        std::stringstream myStream(att->value());
        myStream >> body.torque_.x >> body.torque_.y >> body.torque_.z;
      }
      else if (name == "boxextents")
      {
        std::stringstream myStream(att->value());
        myStream >> body.extents_[0] >> body.extents_[1] >> body.extents_[2];
      }
      else if (name == "u")
      {
        std::stringstream myStream(att->value());
        myStream >> body.uvw_[0].x >> body.uvw_[0].y >> body.uvw_[0].z;
      }
      else if (name == "v")
      {
        std::stringstream myStream(att->value());
        myStream >> body.uvw_[1].x >> body.uvw_[1].y >> body.uvw_[1].z;
      }
      else if (name == "w")
      {
        std::stringstream myStream(att->value());
        myStream >> body.uvw_[2].x >> body.uvw_[2].y >> body.uvw_[2].z;
      }
      else if (name == "density")
      {
        body.density_ = atof(att->value());
      }
      else if (name == "restitution")
      {
        body.restitution_ = atof(att->value());
      }
      else if (name == "affectedbygravity")
      {
        body.affectedByGravity_ = atoi(att->value());
      }
      else if (name == "meshfile")
      {
        strcpy(body.fileName_, att->value());
      }
      else if (name == "volume")
      {
        body.volume_ = atof(att->value());
      }
      else if (name == "tensor")
      {
        std::stringstream myStream(att->value());
        memset(body.tensor_, 0, 9*sizeof(Real));
        myStream >> body.tensor_[0] >> body.tensor_[4] >> body.tensor_[8];
      }

      att = att->next_attribute();

    }

    params.rigidBodies_.push_back(body);

  }
}

Reader::Reader(void)
{
}

Reader::~Reader(void)
{
}
  
void Reader::readParametersDeform(std::string strFileName, DeformParameters &parameters)
{
  using namespace std;
  string first;
  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    exit(0);
  }
  
  if(!readNextTokenInt(in,string("nBodies"),parameters.bodies_))
  {
    std::cerr<<"bad file format: "<<strFileName
      <<" could not find parameter: "<<"nBodies"<<endl;
    exit(0);
  }  

  readRigidBodySection(in, parameters.bodies_, parameters.rigidBodies_); 
  
  in.close();

}  

void Reader::readParameters(std::string strFileName, WorldParameters &parameters)
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
      readInt(in,parameters.startType_);
      in.getline(strLine,1024);   
    }
    else if(word=="liquidsolid")
    {
      //parse
      readInt(in,parameters.liquidSolid_);      
      in.getline(strLine,1024);
    }
    else if(word=="solution")
    {
      //parse
      readString(in,parameters.solutionFile_);            
      in.getline(strLine,1024);
    }    
    else if(word=="nbodies")
    {
      //parse
      readInt(in,parameters.bodies_);      
      in.getline(strLine,1024);
    }    
    else if(word=="bodyinit")
    {
      //parse
      readInt(in,parameters.bodyInit_);            
      in.getline(strLine,1024);
    }
    else if(word=="bodyfile")
    {
      //parse
      readString(in,parameters.bodyConfigurationFile_);                  
      in.getline(strLine,1024);
    }        
    else if(word=="defaultdensity")
    {
      //parse
      readReal(in,parameters.defaultDensity_);
      in.getline(strLine,1024);
    }        
    else if(word=="liquiddensity")
    {
      //parse
      readReal(in,parameters.densityMedium_);      
      in.getline(strLine,1024);
    }        
    else if(word=="defaultradius")
    {
      //parse
      readReal(in,parameters.defaultRadius_);      
      in.getline(strLine,1024);
    }
    else if(word=="gravity")
    {
      //parse
      readVector(in,parameters.gravity_);      
      in.getline(strLine,1024);
    }            
    else if(word=="totaltimesteps")
    {
      //parse
      readInt(in,parameters.nTimesteps_);
      in.getline(strLine,1024);
    }            
    else if(word=="timestep")
    {
      //parse
      readReal(in,parameters.timeStep_);            
      in.getline(strLine,1024);
    }            
    else if(word=="solvertype")
    {
      //parse
      readInt(in,parameters.solverType_);      
      in.getline(strLine,1024);
    }            
    else if(word=="lcpsolveriterations")
    {
      //parse
      readInt(in,parameters.maxIterations_);            
      in.getline(strLine,1024);
    }
    else if(word=="collpipelineiterations")
    {
      //parse
      readInt(in,parameters.pipelineIterations_);                  
      in.getline(strLine,1024);
    }
    else if (word == "extents")
    {
      readExtents(in, parameters.extents_);
      parameters.hasExtents_ = true;
      in.getline(strLine, 1024);
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
  
  readRigidBodySection(in, parameters.bodies_, parameters.rigidBodies_);

  in.close();

}

void Reader::readExtents(std::ifstream &in, Real *extents)
{

  using namespace std;
  bool found = false;
  string equal;
  in >> equal >> extents[0] >> extents[1] >> extents[2] >> extents[3] >> extents[4] >> extents[5];

}

bool Reader::readNextTokenVector(std::ifstream &in,std::string token,VECTOR3 &vec)
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

bool Reader::readNextTokenReal(std::ifstream &in,std::string token,Real &value)
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


bool Reader::readNextTokenInt(std::ifstream &in,std::string token,int &value)
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

bool Reader::readNextTokenString(std::ifstream &in,std::string token,std::string &value)
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

void Reader::readVector(std::ifstream &in,VECTOR3 &vec)
{
  using namespace std;
  bool found=false;
  string equal; 
  in >> equal >> vec.x >> vec.y >> vec.z;      
}

void Reader::readReal(std::ifstream &in,Real &value)
{
  using namespace std;
  bool found=false;
  string equal;
  in >> equal >> value;
}


void Reader::readInt(std::ifstream &in,int &value)
{
  using namespace std;
  bool found=false;
  string equal;
  in >> equal >> value;
}

void Reader::readString(std::ifstream &in,std::string &value)
{
  using namespace std;
  bool found=false;
  string equal;
  in >> equal >> value;
}


bool Reader::readRigidBodySection(std::ifstream &in, int nBodies, std::vector<BodyStorage> &vBodies)
{
  
  using namespace std;
    
  for(int i=1;i<=nBodies;i++)
  {
    BodyStorage body;
    readRigidBody(in,body);
    vBodies.push_back(body);
  }
  
  return true;
  
}

bool Reader::readRigidBody(std::ifstream &in, BodyStorage &body)
{
  
  using namespace std;
  bool found=false;
  char strLine[256];

  std::string fileName;  
  
  if(in.eof())return false;
  
   in >> body.shapeId_;
   in.getline(strLine,256);
  
   in >> body.com_.x >>body.com_.y>>body.com_.z;
   in.getline(strLine,256);  

   in >> body.velocity_.x >>body.velocity_.y>>body.velocity_.z;
   in.getline(strLine,256);  

   in >> body.angVel_.x >>body.angVel_.y>>body.angVel_.z;
   in.getline(strLine,256);  

   in >> body.angle_.x >>body.angle_.y>>body.angle_.z;
   in.getline(strLine,256);  
   
   in >> body.force_.x >>body.force_.y>>body.force_.z;
   in.getline(strLine,256);  

   in >> body.torque_.x >>body.torque_.y>>body.torque_.z;
   in.getline(strLine,256);  

   in >> body.extents_[0]>>body.extents_[1]>>body.extents_[2];
   in.getline(strLine,256);

   in >> body.uvw_[0].x >> body.uvw_[0].y >> body.uvw_[0].z;
   in.getline(strLine,256);  

   in >> body.uvw_[1].x >> body.uvw_[1].y >> body.uvw_[1].z;
   in.getline(strLine,256);  
   
   in >> body.uvw_[2].x >> body.uvw_[2].y >> body.uvw_[2].z;
   in.getline(strLine,256);  

   in >> body.density_;
   in.getline(strLine,256);
   
   if(body.shapeId_ == RigidBody::MESH)
   {
     in >> body.volume_;
     in.getline(strLine,256); 
     memset(body.tensor_,0.0,9*sizeof(Real));
     in >> body.tensor_[0] >> body.tensor_[4] >> body.tensor_[8];     
     in.getline(strLine,256);               
   }

   in >> body.restitution_;
   in.getline(strLine,256);

   in >> body.affectedByGravity_;
   in.getline(strLine,256);

   body.matrixAvailable_ = false;
   
   in >> fileName;
   
   strcpy(body.fileName_,fileName.c_str());

  return true;  

}

}
