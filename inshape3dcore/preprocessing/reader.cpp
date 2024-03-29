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
#include <regex>
#include <json.hpp>
#include <ode_config.hpp>

namespace i3d {

void FileParserXML::parseDataXML(WorldParameters &params, const std::string &fileName)
{

  using namespace rapidxml;

  using json = nlohmann::json;

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
      params.startType_ = std::atoi(att->value());
    }
    else if (word == "liquidsolid")
    {
      params.liquidSolid_ = std::atoi(att->value());
    }
    else if (word == "solution")
    {
      params.solutionFile_= std::string(att->value());
    }
    else if (word == "nbodies")
    {
      params.bodies_ = std::atoi(att->value());
    }
    else if (word == "bodyinit")
    {
      params.bodyInit_ = std::atoi(att->value());
    }
    else if (word == "bodyfile")
    {
      params.bodyConfigurationFile_ = std::string(att->value());
    }
    else if (word == "odeconfigfile")
    {
      params.odeConfigurationFile_ = std::string(att->value());
    }
    else if (word == "cgalconfigfile")
    {
      params.cgalConfigurationFile_ = std::string(att->value());
    }
    else if (word == "defaultdensity")
    {
      params.defaultDensity_ = std::atof(att->value());
    }
    else if (word == "liquiddensity")
    {
      params.densityMedium_ = std::atof(att->value());
    }
    else if (word == "airfriction")
    {
      params.airFriction_ = std::atof(att->value());
    }
    else if (word == "refinementlevel")
    {
      params.refinementLevel_ = std::atoi(att->value());
      std::cout << "Found refinement level " << params.refinementLevel_ << std::endl;
    }
    else if (word == "defaultradius")
    {
      params.defaultRadius_ = std::atof(att->value());
    }
    else if (word == "gravity")
    {
      std::stringstream myStream(att->value());
      myStream >> params.gravity_.x >> params.gravity_.y >> params.gravity_.z;
    }
    else if (word == "totaltimesteps")
    {
      params.nTimesteps_ = std::atoi(att->value());
    }
    else if (word == "timestep")
    {
      params.timeStep_ = std::atof(att->value());
    }
    else if (word == "solvertype")
    {
      params.solverType_ = std::atoi(att->value());
    }
    else if (word == "lcpsolveriterations")
    {
      params.maxIterations_ = std::atoi(att->value());
    }
    else if (word == "collpipelineiterations")
    {
      params.pipelineIterations_ = std::atoi(att->value());
    }
    else if (word == "dofbm")
    {
      int val = std::atoi(att->value());
      if(val == 0)
        params.doFBM_ = false;
    }
    else if (word == "dodynamics")
    {
      int val = std::atoi(att->value());
      if(val == 0)
        params.doDynamics_ = false;
    }
    else if (word == "outputrigidbodies")
    {
      int val = std::atoi(att->value());
      if(val == 1)
        params.outputRigidBodies_ = true;
    }
    else if (word == "excludedefaultboundary")
    {
      int val = std::atoi(att->value());
      if(val == 1)
        params.excludeDefaultBoundary_ = true;
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
  
  //---------------------------------------------------------------------------------------------------------------------------------------------

  // Read the boundary section
  n = root->first_node("BoundaryDescription");

  if (n)
  {

      att = n->first_attribute();

      while (att)
      {
        //std::cout << "Node has the following attribute: " << std::endl;
        //std::cout << "Name of the attribute: " << att->name() << std::endl;

        std::string name(att->name());
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);

        if (name == "ncomponents")
        {
          params.boundaryComponents_ = std::atoi(att->value());
          std::cout << "components: " << params.boundaryComponents_ << std::endl;
        }

        att = att->next_attribute();
      }

      for (xml_node<> * bndry_node = n->first_node("BoundaryShape"); bndry_node; bndry_node = bndry_node->next_sibling())
      {
        std::cout << "Found another node" << std::endl;
        std::cout << "Name of the current node: " << bndry_node->name() << std::endl;

        bndryShape sh;

        att = bndry_node->first_attribute();

        while (att)
        {
          //std::cout << "Node has the following attribute: " << std::endl;
          //std::cout << "Name of the attribute: " << att->name() << std::endl;

          std::string name(att->name());
          std::transform(name.begin(), name.end(), name.begin(), ::tolower);

          if (name == "type")
          {
            sh.type = std::atoi(att->value());
            std::cout << "itype: " << sh.type << std::endl;
          }
          else if (name == "meshfile")
          {

            std::strcpy(sh.name, att->value());
            std::cout << "mesh name: " << sh.name << std::endl;
          }

          att = att->next_attribute();
        }
        params.boundaries_.push_back(sh);
      }
  }
  
  if (params.cgalConfigurationFile_ != "")
  {
    OffsReader reader;
    reader.readParameters(params.cgalConfigurationFile_, params);
    return;
  }  

  //---------------------------------------------------------------------------------------------------------------------------------------------

  if(params.bodies_ == 0)
  {
    //std::cout << "number of rigid bodies: " << params.bodies_ << std::endl;
    return;
  }

  n = root->first_node("RigidBodyList");
  //std::cout << "Name of the current node: " << n->name() << "\n";
  
  if (!n)
  {
    std::cout << "Warning: xml configuration file does not contain a RigidBodyList section." << std::endl;    
    return;
  }
  
  for (n = n->first_node(); n; n = n->next_sibling())
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
        body.density_ = std::atof(att->value());
      }
      else if (name == "restitution")
      {
        body.restitution_ = std::atof(att->value());
      }
      else if (name == "affectedbygravity")
      {
        body.affectedByGravity_ = std::atoi(att->value());
      }
      else if (name == "meshfile")
      {
        std::strcpy(body.fileName_, att->value());
      }
      else if (name == "dynamicstype")
      {
        body.dynamicsType_ = std::string(att->value());
        std::cout << "Dynamics type: " << body.dynamicsType_ << std::endl;
      }
      else if (name == "volume")
      {
        body.volume_ = std::atof(att->value());
      }
      else if (name == "tensor")
      {
        std::stringstream myStream(att->value());
        std::memset(body.tensor_, 0, sizeof body.tensor_);
        myStream >> body.tensor_[0] >> body.tensor_[4] >> body.tensor_[8];
      }

      att = att->next_attribute();
    }

    if (body.shapeId_ == RigidBody::MESH || body.shapeId_ == RigidBody::CGALMESH)
    {
      for (xml_node<> * meshes_node = n->first_node("Meshes"); meshes_node; meshes_node = meshes_node->next_sibling())
      {
        std::cout << "Found another node" << std::endl;
        std::cout << "Name of the current node: " << meshes_node->name() << std::endl;

        for (xml_node<> * mesh_node = meshes_node->first_node("Mesh"); mesh_node; mesh_node = mesh_node->next_sibling())
        {
          //std::cout << "Name of the current node: " << mesh_node->name() << std::endl;
          char buf[1024];

          std::strcpy(buf, mesh_node->first_attribute("meshFile")->value());
         
          body.meshFiles_.push_back(std::string(buf));

          std::cout << "Name of the submesh: " << buf << std::endl;
        }
      }

      if (!body.meshFiles_.empty())
      {
        body.useMeshFiles_ = true;
        std::strcpy(body.fileName_, body.meshFiles_.front().c_str());
      }
      else
      {
        body.useMeshFiles_ = true;
        body.meshFiles_.push_back(body.fileName_);
      }
    }
    else if(body.shapeId_ == RigidBody::SOFTBODY4)
    {
      for(xml_node<> *sb_node = n->first_node("SoftBodyParams"); sb_node; sb_node = sb_node->next_sibling())
      {
        att = sb_node->first_attribute();
        while (att)
        {

          std::string word(att->name());
          std::transform(word.begin(), word.end(), word.begin(), ::tolower);

          if (word == "numberofparticles")
          {
            body.nSoftBodyParticles_ = std::atoi(att->value());
          }
          else if (word == "ks")
          {
            //std::cout << "Found ks = " << std::atof(att->value()) << std::endl;
            body.ks_ = std::atof(att->value());
          }
          else if (word == "kb")
          {
            //std::cout << "Found kb = " << std::atof(att->value()) << std::endl;
            body.kb_ = std::atof(att->value());
          }
          else if (word == "kd")
          {
            //std::cout << "Found kd = " << std::atof(att->value()) << std::endl;
            body.kd_ = std::atof(att->value());
          }
//          else if (word == "dofbm")
//          {
//            int val = std::atoi(att->value());
//            if(val == 0)
//              params.doFBM_ = false;
//          }

          att = att->next_attribute();
        }
      }
    }

    params.rigidBodies_.push_back(body);

  }

  bool hasMeshConf(false);
  bool hasBBCenter(false);
  bool hasBBExtents(false);
  bool hasMeshCells(false);

  n = root->first_node("MeshingConfiguration");
  if (n) {

    hasMeshConf = true;

    xml_node<> *sb_node = n->first_node("MeshingParameters");
    att = sb_node->first_attribute();

    while (att) {
      std::string word(att->name());
      if (word == "cells") {
          std::stringstream myStream(att->value());

          myStream >> params.meshingCells_[0] >> 
                      params.meshingCells_[1] >> 
                      params.meshingCells_[2]; 


//          for (auto elem: params.meshingCells_) std::cout << ' ' << elem;
//          std::cout << '\n';
          hasMeshCells = true;
      }
      else if (word == "deformationSteps") {
          std::stringstream myStream(att->value());
          myStream >> params.adaptationSteps_; 
      }
      att = att->next_attribute();
    }

    xml_node<> *bb_node = sb_node->first_node("BoundingBox");

    for(xml_node<> *bb_param = bb_node->first_node(); bb_param; bb_param = bb_param->next_sibling())
    {

      std::string nodeName = bb_param->name(); 
      if (nodeName == "Center") {
        att = bb_param->first_attribute();
        while (att)
        {
          std::string word(att->name());
          if (word == "coordinates") {
              std::stringstream myStream(att->value());

              myStream >> params.meshingBoundingBoxCenter_.x >> 
                          params.meshingBoundingBoxCenter_.y >> 
                          params.meshingBoundingBoxCenter_.z;

              hasBBCenter = true;
          }
          att = att->next_attribute();
        }
      } else if (nodeName == "Extents") {
        att = bb_param->first_attribute();
        while (att)
        {
          std::string word(att->name());
          if (word == "boxExtents") {
              std::stringstream myStream(att->value());

              myStream >> params.meshingBoundingBoxExtents_.x >> 
                          params.meshingBoundingBoxExtents_.y >> 
                          params.meshingBoundingBoxExtents_.z;

              params.meshingBoundingBoxExtents_ *= 0.5;
              hasBBExtents = true;
          }
          att = att->next_attribute();
        }
      }
    }

    // Look for decimation parameters 
    xml_node<> *dec_node = n->first_node("DecimationParameters");
    att = dec_node->first_attribute();
    while (att)
    {
      std::string word(att->name());
      if (word == "method") {
          std::stringstream myStream(att->value());
          std::string methodName(myStream.str());
          if (methodName == "intersection") {
            params.meshDecimationMethod_ = 0;
          }
          else if (methodName == "distance") {
            params.meshDecimationMethod_ = 1;
          }
          else if (methodName == "distanceAbsolute") {
            params.meshDecimationMethod_ = 2;
          }
          else if (methodName == "containment") {
            params.meshDecimationMethod_ = 3;
          }
          else {
            params.meshDecimationMethod_ = 0;
          }
      }
      else if (word == "distanceEps") {
        params.meshDecimationEpsilon_ = std::atof(att->value());
      }
      att = att->next_attribute();
    }

    std::cout << "Found user-defined meshing parameters: " << std::endl;

    if(!hasBBCenter) {
    std::cout << " > please provide the mesh bounding box center " << std::endl;
    std::exit(EXIT_FAILURE);
    }
    if(!hasBBExtents) {
    std::cout << " > please provide the mesh bounding box extents " << std::endl;
    std::exit(EXIT_FAILURE);
    }
    if(!hasMeshCells) {
    std::cout << " > please provide the number of mesh cells " << std::endl;
    std::exit(EXIT_FAILURE);
    }

    if(hasBBExtents && hasBBCenter && hasMeshCells) {
      std::cout << " > All parameters were provided " << std::endl;
      params.hasUserMeshingParameters_ = true;
    }
  }//end if(n)
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
    std::exit(EXIT_FAILURE);
  }

  if(!readNextTokenInt(in,string("nBodies"),parameters.bodies_))
  {
    std::cerr<<"bad file format: "<<strFileName
            <<" could not find parameter: "<<"nBodies"<<endl;
    std::exit(EXIT_FAILURE);
  }

  readRigidBodySection(in, parameters.bodies_, parameters.rigidBodies_);

  in.close();

}  

std::vector<BodyStorage> Reader::read_sol_rb(int idx)
{

  std::vector<BodyStorage> dumpSolution;

  std::string folder("_sol_rb");

  std::ostringstream nameRigidBodies;
  nameRigidBodies << folder << "/" << idx << "/rb.dmp";

  std::string n(nameRigidBodies.str());

  std::cout << "Loading dmp file: " << n << std::endl;
  
  // Function: istream &read(char *buf, streamsize num)
  // Read in <num> chars from the invoking stream
  // into the buffer <buf>
  std::ifstream in(n, std::ios::in);
  
  if(!in)
  {
    std::cout << "Cannot open file: "<< n << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::string line;

  std::getline(in,line);

  auto c = line.find(':');
  //std::cout << "NumberOfRigidBodies:" << line.substr(c+1, line.length() - c+1) << std::endl;
  //make_pair(item.substr(0,c),item.substr(c+1, item.length() - c+1));

  std::string stringBodyCount(line.substr(c+1, line.length() - c+1));

  int rigidBodyCount = std::atoi(stringBodyCount.c_str());

  for(int i(0); i < rigidBodyCount; ++i)
  {

    BodyStorage body;

    std::string workString;

    std::getline(in, workString);

    in >> body.com_.x >> body.com_.y >> body.com_.z;

    std::getline(in, workString);

    in >> body.velocity_.x >> body.velocity_.y >> body.velocity_.z;

    std::getline(in, workString);

    in >> body.angVel_.x >> body.angVel_.y >> body.angVel_.z;

    std::getline(in, workString);

    in >> body.force_.x >> body.force_.y >> body.force_.z;

    std::getline(in, workString);

    in >> body.torque_.x >> body.torque_.y >> body.torque_.z;

    std::getline(in, workString);

    double w,x,y,z;

    in >> w >> x >> y >> z;
    
    std::getline(in, workString);

    in >> body.density_;

    std::getline(in, workString);

    in >> body.volume_;

    std::getline(in, workString);

    in >> body.invMass_;

    std::getline(in, workString);

    in >> body.restitution_;

    std::getline(in, workString);

    in >> body.shapeId_;

    std::getline(in, workString);

    in >> body.id_;

    std::getline(in, workString);

    in >> body.tensor_[0] >> 
          body.tensor_[1] >> 
          body.tensor_[2] >>
          body.tensor_[3] >> 
          body.tensor_[4] >> 
          body.tensor_[5] >>
          body.tensor_[6] >> 
          body.tensor_[7] >> 
          body.tensor_[8];

    std::getline(in, workString);

    std::string theFileName;

    in >> theFileName;

    std::getline(in, workString);

    std::strcpy(body.fileName_, theFileName.c_str());

    dumpSolution.push_back(body);

  }

  in.close();

  return dumpSolution;

}

void Reader::readParameters(std::string strFileName, WorldParameters &parameters)
{
  using namespace std;
  string first;
  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    std::exit(EXIT_FAILURE);
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

void E3dReader::readE3dFile(std::string strFileName)
{

  using namespace std;

  string first;

  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    std::exit(EXIT_FAILURE);
  }

  char strLine[1024];
  string equal;
  bool found=false;
  while (!in.eof())
  {
    string word;
    //parse first word in line
    in >> word;
    in.getline(strLine,1024);

    if (regex_match(word, std::regex("(\\[E3DGeometryData/Machine/Element)(.*)")))
    {
      readElement(in);
      //cout << "Matched: " << word << endl;
    }


    //std::transform(word.begin(), word.end(), word.begin(), ::tolower);
  }

  in.close();

  std::exit(EXIT_FAILURE);

}

bool E3dReader::readElement(std::ifstream &in)
{

  char strLine[1024];
  while (!in.eof())
  {
    std::string word;
    //parse first word in line
    std::streampos pos = in.tellg();
    //    std::cout << "Streampos: " << pos << std::endl;
    in >> word;
    in.getline(strLine,1024);
    //    std::cout << "Word: " << word << std::endl;

    std::smatch sm;
    if (regex_match(word, sm, std::regex("(Type)(.*)")))
    {
      std::cout << "Matched: " << word << std::endl;
      std::cout << "Match object: " << sm.str(2) << std::endl;

      std::string s(sm.str(2));
      s.erase(std::remove_if(s.begin(), s.end(), [](char c) {return c == '=' || c == ' ';}), s.end());
      if (s == "STL")
      {
        std::cout << "Value: " << s << std::endl;
      }
      else
      {
        return false;
      }

    }
    else if (regex_match(word, std::regex("(\\[)(.*)")))
    {
      //      std::cout << "Matched: " << word << std::endl;
      //      std::cout << "Rewinding Streampos: " << std::endl;
      in.seekg(pos);
      //      std::cout << "New Streampos: " << in.tellg() << std::endl;
      //      in.close();
      //      std::exit(EXIT_FAILURE);
      return true;
    }

    //std::transform(word.begin(), word.end(), word.begin(), ::tolower);
  }

  return true;
}

void OffsReader::readParameters(std::string strFileName, WorldParameters & parameters)
{
  using namespace std;

  string first;

  ifstream in(strFileName.c_str());

  if(!in.is_open())
  {
    std::cerr<<"Unable to open file: "<<strFileName<<endl;
    std::exit(EXIT_FAILURE);
  }

  char strLine[1024];
  string equal;
  bool found=false;

  string word;

  in >> word;
  in.getline(strLine,1024);

  int nMeshes = std::atoi(word.c_str());

  parameters.bodies_ = std::atoi(word.c_str());

  for(int i(0); i < parameters.bodies_; ++i)
  {
    //parse first word in line
    in >> word;
    in.getline(strLine,1024);
    std::cout << word << std::endl;

    BodyStorage body;

    body.shapeId_ = 15;

    strcpy(body.fileName_,word.c_str());

    parameters.rigidBodies_.push_back(body);

  }

}

}
