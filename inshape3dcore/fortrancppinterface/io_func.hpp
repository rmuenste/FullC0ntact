// In older versions of GCC the filesystem header is "experimental",
// then in later version it has become part of the standard.
// So we decide based on the GCC version where to find the filesystem header.

// The procedure is the same for the Intel compiler because it uses the GCC headers
#ifndef WIN32
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)
#if GCC_VERSION < 80100
#include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
    namespace fs = std::filesystem;
#endif
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif
#include <map>
#include <ctime>

// TODO:
// There is an issue with the function ifstream::getline(buf,size). 
// A problem could occur when a line is read that is larger than
// the buffersize <size>. A solution would be to use std::getline(std::string) 
// instead of the ifstream::getline function. 

struct FileHeaderDump {

  /*
   * The name of the FE-space that the solution field
   * is defined in.
   */
  std::string _space;
  /*
   * Name of the solution field
   */
  std::string _name;
  /*
   * The format of the file: 
   * can be element based of vertex based
   */
  std::string _format;

  /*
   * File version of the dump file
   */  
  int _version;
  /*
   * The file contains an output of coarse mesh
   * elements and the fine mesh dofs contained
   * in the coarse mesh element. This variable
   * tells how many fine mesh dofs are stored inside
   * a coarse mesh element.
   */
  int _dofsInElement; 
  /*
   * The number of components of the particular
   * solution field
   */
  int _components;
  int _dofsTotal;
  /*
   * The output level of the solution
   */
  int _outputLevel;

  FileHeaderDump() = default;

  FileHeaderDump(const std::string &s) {

    std::stringstream ss(s);
    std::string item;
    std::map<std::string, std::string> kv_map;

    while (std::getline(ss, item, ',')) {
        auto c = item.find(':');
        kv_map.insert(make_pair(item.substr(0,c),item.substr(c+1, item.length() - c+1)));
    }

    auto it = kv_map.find("Version"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    int ver = std::atoi((it->second).c_str());
    if(ver != 1)
    {
      std::cout << "Found unsupported dump file version: " << ver << std::endl;
      std::exit(EXIT_FAILURE);
    }

    _version = ver;

    it = kv_map.find("Format"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _format = it->second;
    }
    
    it = kv_map.find("Name"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _name = it->second;
    }

    it = kv_map.find("FeSpace"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _space = it->second;
    }

    it = kv_map.find("DofsInElement"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _dofsInElement = std::atoi(it->second.c_str());
    }

    it = kv_map.find("Components"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _components = std::atoi(it->second.c_str());

    }

    it = kv_map.find("DofsTotal"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _dofsTotal = std::atoi(it->second.c_str());

    }

    it = kv_map.find("OutputLevel"); 

    if(it == kv_map.end())
    {
      std::cout << "Format error while reading dump file." << std::endl;
      std::exit(EXIT_FAILURE);
    } else {

      _outputLevel = std::atoi(it->second.c_str());

    }

  };

  FileHeaderDump(const std::string &space, 
                 const std::string &name,
                 const std::string &format,
                 int version,
                 int dofsInElement,
                 int components,
                 int dofsTotal,
                 int outputLevel) :
    _space(space),
    _name(name),
    _format(format),
    _version(version),
    _dofsInElement(dofsInElement),
    _components(components),
    _dofsTotal(dofsTotal),
    _outputLevel(outputLevel) {};

  void toFile(ostream &os) 
  {
    os << "Version:" << _version;
    os << ",";
    os << "Format:" << _format;
    os << ",";
    os << "Name:" << _name;
    os << ",";
    os << "FeSpace:" << _space;
    os << ",";
    os << "DofsInElement:" << _dofsInElement;
    os << ",";
    os << "Components:" << _components;
    os << ",";
    os << "DofsTotal:" << _dofsTotal;
    os << ",";
    os << "OutputLevel:" << _outputLevel;
    os << endl;
  }

  void output() 
  {
    std::cout << "Version:" << _version;
    std::cout << ",";
    std::cout << "Format:" << _format;
    std::cout << ",";
    std::cout << "Name:" << _name;
    std::cout << ",";
    std::cout << "FeSpace:" << _space;
    std::cout << ",";
    std::cout << "DofsInElement:" << _dofsInElement;
    std::cout << ",";
    std::cout << "Components:" << _components;
    std::cout << ",";
    std::cout << "DofsTotal:" << _dofsTotal;
    std::cout << ",";
    std::cout << "OutputLevel:" << _outputLevel;
    std::cout << endl;
  }

};


/*
 *
 * @param iout index of output folder
 * @param istep simulation step
 * @param simTime time of the simulation
 * 
 */
void write_sol_time(int iout, int istep, double simTime)
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    folder.append("/");
    folder.append(std::to_string(iout));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    std::ostringstream nameField;
    nameField << folder << "/time.dmp";

    std::string n(nameField.str());
    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Writing dmp file: " << n << std::endl;
    }    
    
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ofstream out(n, ios::out);
    
    if(!out)
    {
      
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    out << simTime << "\n";
    out << istep << "\n";

    out.close();

  }

}

void read_sol_time(char startFrom[60], int *istep, double *simTime)
{
  if(myWorld.parInfo_.getId() != 0)
  {

    std::string startName(startFrom);
    int step = std::stoi(startName);

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));
    folder.append("/");

    folder.append(std::to_string(step));

    if(!fs::exists(folder))
    {
      std::cout << "Folder name: " << folder << " does not exist." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::ostringstream nameField;
    nameField << folder << "/time.dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    char buf[1024];
    double time;
    int ist;

    in >> time;

    // this will read until we find the
    // first newline character or 1024 bytes
    in.getline(buf, 1024);

    in >> ist;

    *istep = ist;

    *simTime = time;

    in.close();

  }
  else
  {

    std::string startName(startFrom);
    int step = std::stoi(startName);

    std::string folder("_dump");
    folder.append("/processor_1");
    folder.append("/");

    folder.append(std::to_string(step));

    if(!fs::exists(folder))
    {
      std::cout << "Folder name: " << folder << " does not exist." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::ostringstream nameField;
    nameField << folder << "/time.dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    char buf[1024];
    double time;
    int ist;

    in >> time;

    // this will read until we find the
    // first newline character or 1024 bytes
    in.getline(buf, 1024);

    in >> ist;

    *istep = ist;

    *simTime = time;

    in.close();

  }

}

/*
 *
 * @param iout index of output folder
 * @param lvl level
 * @param comp #components of the q2 field
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 * @param u vel u-component 
 * @param v vel v-component 
 * @param w vel w-component 
 *
 */
void write_sol_vel(int iout, int lvl, int comp, int nel_fine, 
                   int nel_coarse, int dofsInE, 
                   int elemmap[], int *edofs, double *u, double *v, double *w)
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    folder.append("/");
    folder.append(std::to_string(iout));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    std::ostringstream nameField;
    nameField << folder << "/velocity.dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Writing dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ofstream out(n, ios::out);
    
    if(!out)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    FileHeaderDump header(std::string("Q2"),
                          std::string("Velocity"),
                          std::string("V"),
                          1,
                          dofsInE,
                          3,
                          nel_fine,
                          lvl);

    header.toFile(out);

    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      out << std::scientific;
      out << std::setprecision(16);
      out << elemmap[iel] << "\n";  

      // write the x values 
      for(int i(0); i < dofsInE; ++i)
      {
        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        int ind = edofs[iel + i*nel_coarse];
        out << " " << u[ind-1];
      }
      out << "\n";

      // write the y values 
      for(int i(0); i < dofsInE; ++i)
      {
        int ind = edofs[iel + i*nel_coarse];
        out << " " << v[ind-1];
      }
      out << "\n";
      
      // write the z values 
      for(int i(0); i < dofsInE; ++i)
      {
        int ind = edofs[iel + i*nel_coarse];
        out << " " << w[ind-1];
      }
      out << "\n";
    }

    out.close();

  }

}

/*
 *
 * @param startFrom simulation step
 * @param lvl level
 * @param comp #components of the q2 field
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 * @param u vel u-component 
 * @param v vel v-component 
 * @param w vel w-component 
 *
 */
void read_sol_vel(char startFrom[60], int lvl, int comp, int nel_fine, 
                  int nel_coarse, int dofsInE, 
                  int elemmap[], int *edofs, double *u, double *v, double *w)
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string startName(startFrom);
    int istep = std::stoi(startName);

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));
    folder.append("/");

    folder.append(std::to_string(istep));

    if(!fs::exists(folder))
    {
      std::cout << "Folder name: " << folder << " does not exist." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::ostringstream nameField;
    nameField << folder << "/velocity.dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    char buf[1024];

    // this will read until we find the
    // first newline character or 1024 bytes
    in.getline(buf, 1024);

    std::string line(buf);
    FileHeaderDump header(line);

    // in a properly formatted file the stream get-pointer
    // should now be on the first byte after the '\n' 
    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      int global_idx;
      double val;
      int ind;

      in >> global_idx;  
      in.getline(buf, 1024);

      // read u-component 
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        ind = edofs[iel + i*nel_coarse];
        u[(ind-1)] = val;
      }

      in.getline(buf, 1024);

      // read v-component 
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        //int ind = edofs[iel + i*nel_coarse];
        ind = edofs[iel + i*nel_coarse];
        v[(ind-1)] = val;
      }

      in.getline(buf, 1024);
      
      // read w-component 
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        ind = edofs[iel + i*nel_coarse];
        w[(ind-1)] = val;
      }

      in.getline(buf, 1024);

    }

    in.close();

  }

}

/*
 *
 * @param startFrom simulation step
 * @param lvl level
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 * @param pres array of the pressure values 
 *
 */
void read_sol_pres(char startFrom[60], int lvl, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double pres[])
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string folder("_dump");

    std::string startName(startFrom);
    int istep = std::stoi(startName);

    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));
    folder.append("/");
    folder.append(std::to_string(istep));

    if(!fs::exists(folder))
    {
      std::cout << "Folder name: " << folder << " does not exist." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::ostringstream namePres;
    namePres << folder << "/pressure.dmp";

    std::string n(namePres.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    char buf[1024];

    // this will read until we find the
    // first newline character or 1024 bytes
    in.getline(buf, 1024);

    std::string line(buf);
    FileHeaderDump header(line);

    // in a properly formatted file the stream get-pointer
    // should now be on the first byte after the '\n' 
    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      int global_idx;
      double val;
      int ind;

      in >> global_idx;  
      in.getline(buf, 1024);

      // read the mean values 
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        ind = edofs[iel + i*nel_coarse];
        pres[4 * (ind-1)] = val;
      }

      in.getline(buf, 1024);

      // read the d/dx derivative
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        //int ind = edofs[iel + i*nel_coarse];
        ind = edofs[iel + i*nel_coarse];
        pres[4 * (ind-1) + 1] = val;
      }

      in.getline(buf, 1024);
      
      // read the d/dy derivative
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        ind = edofs[iel + i*nel_coarse];
        pres[4 * (ind-1) + 2] = val;
      }

      in.getline(buf, 1024);

      // read the d/dz derivative
      for(int i(0); i < dofsInE; ++i)
      {
        in >> val;

        ind = edofs[iel + i*nel_coarse];
        pres[4 * (ind-1) + 3] = val;
      }
      in.getline(buf, 1024);
    }

    in.close();

  }

}

/*
 *
 * @param iout index of output folder
 * @param lvl level
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 * @param pres array of the pressure values 
 *
 */
void write_sol_pres(int iout, int lvl, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double pres[])
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    folder.append("/");
    folder.append(std::to_string(iout));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    std::ostringstream namePres;
    namePres << folder << "/pressure.dmp";

    std::string n(namePres.str());
    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Writing dmp file: " << n << std::endl;
    }    
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ofstream out(n, ios::out);
    
    if(!out)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    FileHeaderDump header(std::string("P1"),
                          std::string("Pressure"),
                          std::string("E"),
                          1,
                          dofsInE,
                          4,
                          nel_fine,
                          lvl);

    header.toFile(out);

    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      out << std::scientific;
      out << std::setprecision(16);
      out << elemmap[iel] << "\n";  

      // write the mean values 
      for(int i(0); i < dofsInE; ++i)
      {
        // ind is the index of the i-th fine mesh
        // P1 dof in the iel-th coarse mesh element
        int ind = edofs[iel + i*nel_coarse];
        out << " " << pres[4 * (ind-1)];
      }
      out << "\n";
      // write the d/dx derivative
      for(int i(0); i < dofsInE; ++i)
      {
        int ind = edofs[iel + i*nel_coarse];
        out << " " << pres[4 * (ind-1) + 1];
      }
      out << "\n";
      //
      // write the d/dy derivative
      for(int i(0); i < dofsInE; ++i)
      {
        int ind = edofs[iel + i*nel_coarse];
        out << " " << pres[4 * (ind-1) + 2];
      }
      out << "\n";

      // write the d/dz derivative
      for(int i(0); i < dofsInE; ++i)
      {
        int ind = edofs[iel + i*nel_coarse];
        out << " " << pres[4 * (ind-1) + 3];
      }
      out << "\n";
    }

    out.close();

  }

}

/*
 *
 * @param startFrom name of the output field
 *
 */
void read_sol_rb(char startFrom[60])
{

  if(myWorld.parInfo_.getId() == 1)
  {

    std::string folder("_sol_rb");

    std::ostringstream nameRigidBodies;
    nameRigidBodies << folder << "/rb.dmp";

    std::string n(nameRigidBodies.str());

    std::cout << "Loading dmp file: " << n << std::endl;
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    std::string line;

    std::getline(in,line);

    auto c = line.find(':');
    std::cout << "NumberOfRigidBodies:" << line.substr(c+1, line.length() - c+1) << std::endl;
    //make_pair(item.substr(0,c),item.substr(c+1, item.length() - c+1));

    std::string stringBodyCount(line.substr(c+1, line.length() - c+1));

    int rigidBodyCount = std::atoi(stringBodyCount.c_str());
    
    // this will read until we find the
    // first newline character or 1024 bytes
    std::getline(in, line);

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

      body.toString();

    }

    in.close();

  }

}

/*
 *
 * @param startFrom name of the output field
 * @param iout index of output folder
 * @param lvl level
 * @param comp #components of the q2 field
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 *
 */
void write_q2_sol(char startFrom[60], int iout, int lvl, int comp,
                   int nel_fine, int nel_coarse, int dofsInE, 
                   int elemmap[], int *edofs)
{

  if(myWorld.parInfo_.getId() != 0)
  {

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    folder.append("/");
    folder.append(std::to_string(iout));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    std::string customName(startFrom);
    std::ostringstream nameField;
    nameField << folder << "/" << customName << ".dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Writing custom q2 field to: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ofstream out(n, ios::out);
    
    if(!out)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    FileHeaderDump header(std::string("Q2"),
                          customName,
                          std::string("V"),
                          1,
                          dofsInE,
                          comp,
                          nel_fine,
                          lvl);

    header.toFile(out);


    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      out << std::scientific;
      out << std::setprecision(16);
      out << elemmap[iel] << "\n";  

      for(int j(0); j < comp; j++)
      {
        if(j < arrayPointers.size())
        {

          double *dcomp = arrayPointers[j];

          // write the components 
          for(int i(0); i < dofsInE; ++i)
          {
            // ind is the index of the i-th fine mesh
            // P1 dof in the iel-th coarse mesh element
            int ind = edofs[iel + i*nel_coarse];
            out << " " << dcomp[ind-1];
          }
          out << "\n";
        }
      }
    }

    out.close();

  }

}

void read_q2_sol(char userField[60], char startFrom[60], int lvl, int comp,
                 int nel_fine, int nel_coarse, int dofsInE, 
                 int elemmap[], int *edofs)
{
  if(myWorld.parInfo_.getId() != 0)
  {

    std::string startName(startFrom);
    int istep = std::stoi(startName);

    std::string folder("_dump");
    folder.append("/processor_");
    folder.append(std::to_string(myWorld.parInfo_.getId()));
    folder.append("/");

    folder.append(std::to_string(istep));

    if(!fs::exists(folder))
    {
      std::cout << "Folder name: " << folder << " does not exist." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::string customName(userField);
    std::ostringstream nameField;
    nameField << folder << "/" << customName << ".dmp";

    std::string n(nameField.str());

    if(myWorld.parInfo_.getId() == 1)
    {
      std::cout << "Loading dmp file: " << n << std::endl;
    }
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ifstream in(n, ios::in);
    
    if(!in)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    char buf[1024];

    // this will read until we find the
    // first newline character or 1024 bytes
    in.getline(buf, 1024);

    std::string line(buf);
    FileHeaderDump header(line);

    // in a properly formatted file the stream get-pointer
    // should now be on the first byte after the '\n' 
    for(int iel(0); iel < nel_coarse; ++iel)
    {
      // elemmap[iel] is the global element number
      // local element iel 
      int global_idx;
      double val;
      int ind;

      in >> global_idx;  
      in.getline(buf, 1024);

      for(int j(0); j < comp; j++)
      {
        if(j < arrayPointers.size())
        {

          double *dcomp = arrayPointers[j];

          // read the component 
          for(int i(0); i < dofsInE; ++i)
          {
            in >> val;

            ind = edofs[iel + i*nel_coarse];
            dcomp[(ind-1)] = val;
          }

          in.getline(buf, 1024);

        }
      }
    }

    in.close();

  }

}

/*
 *
 * @param startFrom name of the output field
 * @param iout index of output folder
 * @param lvl level
 * @param comp #components of the q2 field
 * @param nel_fine elements on the fine level
 * @param nel_coarse elements on the coarse level
 * @param dofsInE number of dofs in a coarse mesh element 
 * @param elemmap a map from local to global element index 
 * @param edofs an array of the fine level dofs in a coarse mesh element 
 *
 */
void write_q2_comp(ofstream &out, int iout, int lvl, int comp,
                   int nel_fine, int nel_coarse, int dofsInE, 
                   int elemmap[], int *edofs, double *u)
{

  if(myWorld.parInfo_.getId() != 0)
  {



  }
}

/*
 *
 * @param startFrom simulation step
 * @param inel reference to an integer for nel
 *
 */
void parse_header_line(char headerLine[1024],int *inel)
{

  std::string header(headerLine);

  std::vector<std::string> kv;
  std::size_t found = header.find_first_of(",");
  while(found != std::string::npos)
  {

    kv.push_back(header.substr(0, found));
    header.erase(0, found+1);
    found = header.find_first_of(",");
  }  

  for(auto v : kv)
  {
    //std::cout<<"key-val: "<< v << std::endl;
    std::size_t f = v.find_first_of(":");
    //std::cout<<"key: "<< v.substr(0, f) << std::endl;
    if(v.substr(0, f) == "NEL")
    {
      std::string val = v;
      val.erase(0,f+1); 
      int nel = std::stoi(val);
      *inel = nel;
      //std::cout<<"value: "<< nel << std::endl;
    }
  }

  //int nel = std::stoi(headerLine);
  //*inel = nel;
  //std::cout<<"Header: "<< headerLine << ", " << " nel "  <<std::endl;
  //std::cout<<"Header: "<< headerLine << ", " << " nel "  <<std::endl;

}

//----------------------------------------------------------------------------------------------

void write_1d_header(int *iout, int *my1DOut_nol, int n)
{

  int iTimestep=*iout;
  std::ostringstream sExtrud;
  sExtrud<< "_1D/extrud3d_" << std::setfill('0') << std::setw(4) << iTimestep << ".res";

  // Function: istream &read(char *buf, streamsize num)
  // Read in <num> chars from the invoking stream
  // into the buffer <buf>
  ofstream out(sExtrud.str(), ios::out);
  
  if(!out)
  {
    cout << "Cannot open file: "<< sExtrud.str() << endl;
    std::exit(EXIT_FAILURE);
  }


  out << std::scientific;
  out << std::setprecision(16);
  //out << elemmap[iel] << "\n";  

  out << "[SigmaFileInfo]" << "\n";
  out << "FileType=ResultsExtrud3d" << "\n";
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "Date = %d/%m/%Y %H-%M-%S");
  out << oss.str() << "\n";
  out << "Extrud3dVersion=Extrud3d 2.0" << "\n";
  out << "counter_pos=" << *my1DOut_nol << "\n";
  out << "counter_verl=" << n << "\n";
  out << "[InputSigmaFile]" << "\n";
  out << "#-COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY-" << "\n";

  std::ifstream ifs("_data/Extrud3D.dat");
  std::string content( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );

  out << content << "\n";

  out << "#-COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY--COPY-" << "\n";

  out.close();

}

typedef struct {
  int len;
  void *binPos;
  void *binHeight;
  double mean;
  char name[256];
} HistoData;

typedef struct {
  int len;
  void *mean;
  void *amin;
  void *amax;
  void *loc;
  char unit_name[256];
} myctype;

#include <json.hpp>

// Root JSON data object
nlohmann::json mainJSON;

/*
 *
 * @param dataName the name of the data set
 *
 */
extern "C" void c_write_json_output(int *iangle) {

  using namespace nlohmann;

  int angle = *iangle;
  if(myWorld.parInfo_.getId() == 1)
  {
    std::ostringstream jsonName;
    jsonName << "_1D/main." << std::setfill('0') << std::setw(5) << angle << ".json";
    std::ofstream outputStream(jsonName.str());

    outputStream << std::setw(4) << mainJSON << std::endl; 

    outputStream.close();
  }

}

/*
 *
 * @param myctype the data for the json array 
 *
 */
extern "C" void c_init_json_output(myctype *n) {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {

    mainJSON["SSEData"]["1DOutput"]["length"] = n->len;

    double *dloc = (double*) n->loc;
    std::string unitName(n->unit_name); 

    // The array that is added to the root JSON data object
    nlohmann::json array_loc = nlohmann::json::array();

    // The label and the value are grouped together
    for (int i(0); i < n->len; ++i) {

      array_loc.push_back(json::object({ {"loc", dloc[i]} }));

    }

    mainJSON["SSEData"]["1DOutput"]["Location"]["rows"] = array_loc;
    mainJSON["SSEData"]["1DOutput"]["Location"]["unit"] = unitName;

  }

}

/*
 *
 */
extern "C" void c_init_histogram_section() {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {

    nlohmann::json array_histograms = nlohmann::json::object();
    mainJSON["SSEData"]["HistogramOutput"] = array_histograms;

  }

}


/*
 * @param HistoData the data for the histogram for the json output 
 */
extern "C" void c_add_histogram(HistoData *histoData) {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {

    nlohmann::json histo_object = nlohmann::json::object();
    histo_object["Name"] = histoData->name;; 
    histo_object["Mean"] = histoData->mean; 
    histo_object["Length"] = histoData->len; 

    std::string objectName(histoData->name);

    // The array that is added to the root JSON data object
    nlohmann::json array_bin = nlohmann::json::array();
    nlohmann::json array_height = nlohmann::json::array();

    double *dheight = (double*) histoData->binHeight;
    double *dpos = (double*) histoData->binPos;

    // The label and the value are grouped together
    for (int i(0); i < histoData->len; ++i) {
      array_bin.push_back(dpos[i]);
      array_height.push_back(dheight[i]);
    }

    histo_object["BinPosition"] = array_bin;
    histo_object["BinHeight"] = array_height;

    mainJSON["SSEData"]["HistogramOutput"][objectName] = histo_object;

  }

}

/*
 *
 */
extern "C" void c_write_histogram() {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {

    nlohmann::json array_histograms = nlohmann::json::array();
    mainJSON["SSEData"]["HistogramOutput"] = array_histograms;

    nlohmann::json histo_object = nlohmann::json::object();
    histo_object["Name"] = "Eta"; 
    histo_object["Mean"] = 2.0; 

    // The array that is added to the root JSON data object
    nlohmann::json array_bin = nlohmann::json::array();
    array_bin.push_back(1);
    array_bin.push_back(2);
    array_bin.push_back(3);
    array_bin.push_back(4);

    histo_object["BinPosition"] = array_bin;
    histo_object["BinHeight"] = array_bin;

    mainJSON["SSEData"]["HistogramOutput"].push_back(histo_object);

  }

}


/*
 *
 *
 * @param myctype the data for the json array 
 * @param dataName the name of the data set
 *
 */
extern "C" void c_write_location_array(myctype *n) {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {

    std::cout << "Length: " << n->len << std::endl;

    mainJSON["SSEData"]["1DOutput"]["length"] = n->len;

    double *dloc = (double*) n->loc;
    std::string unitName(n->unit_name); 

    // The array that is added to the root JSON data object
    nlohmann::json array_loc = nlohmann::json::array();

    // The label and the value are grouped together
    for (int i(0); i < n->len; ++i) {

      array_loc.push_back(json::object({ {"loc", dloc[i]} }));

    }

    mainJSON["SSEData"]["1DOutput"]["Location"]["rows"] = array_loc;
    mainJSON["SSEData"]["1DOutput"]["Location"]["unit"] = unitName;

  }

}

/*
 *
 *
 * @param myctype the data for the json array 
 * @param dataName the name of the data set
 *
 */
extern "C" void c_add_json_array(myctype *n, char *dataName) {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {
    double *dmean = (double*) n->mean;
    double *dmin = (double*) n->amin;
    double *dmax = (double*) n->amax;

    std::string theDataName(dataName);
    std::string unitName(n->unit_name); 

    // The array that is added to the root JSON data object
    nlohmann::json array_press = nlohmann::json::array();

    nlohmann::json array_col = nlohmann::json::array();

    // The label array would make it possible to add custom labels
    array_col.push_back(json::object({ {"label", "mean"} }));
    array_col.push_back(json::object({ {"label", "min"} }));
    array_col.push_back(json::object({ {"label", "max"} }));


    // The label and the value are grouped together
    for (int i(0); i < n->len; ++i) {

      json rowArr = nlohmann::json::array();
      array_press.push_back(json::object({ {"mean", dmean[i]}, {"min", dmin[i]}, {"max", dmax[i]} }));

    }

    mainJSON["SSEData"]["1DOutput"][theDataName.c_str()]["rows"] = array_press;
    mainJSON["SSEData"]["1DOutput"][theDataName.c_str()]["unit"] = unitName;

  }

}

/*
 *
 * @param myctype the data for the json array 
 * @param dataName the name of the data set
 *
 */
extern "C" void c_write_json(myctype *n, char *dataName) {

  using namespace nlohmann;

  if(myWorld.parInfo_.getId() == 1)
  {
    double *dmean = (double*) n->mean;
    double *dmin = (double*) n->amin;
    double *dmax = (double*) n->amax;
    std::cout << "mean: " << dmean[0] << std::endl;

    std::string theDataName(dataName);

    // Root JSON data object
    nlohmann::json jsonOut;

    jsonOut["SSEData"]["1DOutput"]["length"] = n->len;

    // The array that is added to the root JSON data object
    nlohmann::json array_press = nlohmann::json::array();

    nlohmann::json array_col = nlohmann::json::array();

    // The label array would make it possible to add custom labels
    array_col.push_back(json::object({ {"label", "mean"} }));
    array_col.push_back(json::object({ {"label", "min"} }));
    array_col.push_back(json::object({ {"label", "max"} }));


    // The label and the value are grouped together
    for (int i(0); i < n->len; ++i) {

      json rowArr = nlohmann::json::array();
      rowArr.push_back(json::object({{"mean", dmean[i]}}));
      rowArr.push_back(json::object({{"min", dmin[i]}}));
      rowArr.push_back(json::object({{"max", dmax[i]}}));
      array_press.push_back(json::object({ {"row", rowArr} }));

    }

    jsonOut["SSEData"]["1DOutput"][theDataName.c_str()]["rows"] = array_press;

    std::ofstream outputStream("_1D/extrud3d.json");

    outputStream << std::setw(4) << jsonOut << std::endl; 

    outputStream.close();

  }

}

//----------------------------------------------------------------------------------------------

void add_output_array(double *array)
{
  arrayPointers.push_back(array);
}

//----------------------------------------------------------------------------------------------

void clean_output_array() 
{
  arrayPointers.clear();
}

//----------------------------------------------------------------------------------------------

extern "C" void writesoftbody(int *iout)
{
  int iTimestep=*iout;

  std::ostringstream line;
  line << "_vtk/line." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";

  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Output file: " <<
    termcolor::reset << line.str()  << std::endl;
  
  CVtkWriter writer;

  //Write the grid to a file and measure the time
  writer.WriteParamLine(softBody_->geom_, line.str().c_str());

}

//----------------------------------------------------------------------------------------------

extern "C" void writeparticles(int *iout)
{
  int iTimestep=*iout;
  std::ostringstream sName,sNameParticles,sMesh,line;
  std::string sModel("_vtk/model.vtk");
  std::string sParticle("solution/particles.i3d");
  std::string sMeshFile("_vtk/particle.vtk");
  CVtkWriter writer;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sName<<"."<<std::setfill('0')<<std::setw(5)<<iTimestep;
  sMesh<< "_vtk/fish." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";
  sNameParticles<< "_vtk/model." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";

  sModel.append(sName.str());
  sParticle.append(sNameParticles.str());

  sModel=std::string(sNameParticles.str());
//  writer.WriteParticleFile(myWorld.rigidBodies_,sModel.c_str());

//  std::string meshFile(sMesh.str());
//  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Output file: " <<
//    termcolor::reset << meshFile  << std::endl;

//  line << "_vtk/line." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";
  
  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld.rigidBodies_,sModel.c_str());


  if(iTimestep == 0)
  {
    CUnstrGridr ugrid;
    for (auto &body : myWorld.rigidBodies_)
    {
      if (body->shapeId_ != RigidBody::MESH)
        continue;
    
      body->map_->convertToUnstructuredGrid(ugrid);
      writer.WriteUnstr(ugrid, "_vtk/DistanceMap.vtk");
      break;
    }
  }

  //writer.WriteParamLine(bull.geom_, line.str().c_str());
  //writer.WriteSpringMesh(myApp.fish_,meshFile.c_str());

  //RigidBodyIO rbwriter;
  //myWorld.output_ = iTimestep;
  //rbwriter.write(myWorld,sParticle.c_str(),false);
  
/*  std::ostringstream sNameHGrid;  
  std::string sHGrid("_gmv/hgrid.vtk");
  sNameHGrid<<"."<<std::setfill('0')<<std::setw(4)<<iTimestep;
  sHGrid.append(sNameHGrid.str());
  
  //iterate through the used cells of spatial hash
  CHSpatialHash *pHash = dynamic_cast<CHSpatialHash*>(myPipeline.m_BroadPhase->m_pStrat->m_pImplicitGrid->GetSpatialHash());  
  
  CUnstrGridr hgrid;
  pHash->ConvertToUnstructuredGrid(hgrid);

  writer.WriteUnstr(hgrid,sHGrid.c_str());  */
  
  
}

//----------------------------------------------------------------------------------------------

extern "C" void writeuniformgrid()
{
//   std::ostringstream sName;
//   sName<<"."<<std::setfill('0')<<std::setw(3)<<myWorld.m_myParInfo.GetID();

//   std::string sGrid("_vtk/uniformgrid");
//   sGrid.append(sName.str());
//   sGrid.append(".vtk");
//   CVtkWriter writer;
  
//   //Write the grid to a file and measure the time
//   writer.WriteUniformGrid(myUniformGrid,sGrid.c_str());  
}

//----------------------------------------------------------------------------------------------

extern "C" void writepvtu(int *iNodes,int *iTime)
{
  int nodes=*iNodes;
  int time =*iTime;
  CVtkWriter writer;
  writer.WritePVTU(nodes,time);
}

//----------------------------------------------------------------------------------------------

extern "C" void writetri(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode)
{
  int NEL=*iNEL;
  int inode=*iNode;
  
  if(inode==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;

    std::string strEnding(".tri");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  using namespace std;

	std::ostringstream sName;
	sName<<"node_"<<std::setfill('0')<<std::setw(2)<<inode;
	
	string strEnding(".tri");
	string strFileName("_gmv/res_");
	strFileName.append(sName.str());
	strFileName.append(strEnding);
	
	//open file for writing
	FILE * myfile = fopen(strFileName.c_str(),"w");

  //check
	fprintf(myfile,"Coarse mesh exported by stdQ2P1 \n");
	fprintf(myfile,"Version 0.1a \n");

	fprintf(myfile,"    %i %i 0 8 12 6     NEL,NVT,NBCT,NVE,NEE,NAE\n",NEL,8*NEL);
  //write the points data array to the file
  fprintf(myfile,"DCORVG\n");
  for(int i=0;i<8*NEL;i++)
  {
    fprintf(myfile,"%f %f %f\n",dcorvg[i][0],dcorvg[i][1],dcorvg[i][2]);
  }//end for

  fprintf(myfile,"KVERT\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"%i %i %i %i %i %i %i %i\n",iKVERT[i][0],iKVERT[i][1],iKVERT[i][2],iKVERT[i][3],iKVERT[i][4],iKVERT[i][5],iKVERT[i][6],iKVERT[i][7]);
  }

  fprintf(myfile,"KNPR\n");
  for(int i=0;i<NEL;i++)
  {
    fprintf(myfile,"0\n");
  }

	fclose( myfile );
}

//----------------------------------------------------------------------------------------------

extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime)
{
  int NEL=*iNEL;
  int NVT=*iNVT;
  int node=*iNode;
  int time=*iTime;
  
  if(node==1)
  {
    std::ostringstream sName;
    sName<<"node_"<<std::setfill('0')<<std::setw(2)<<node<<"."<<std::setfill('0')<<std::setw(4)<<time;

    std::string strEnding(".vtu");
    std::string strFileName("_gmv/res_");
    strFileName.append(sName.str());
    strFileName.append(strEnding);
    
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<"Writing vtk file: "<<strFileName<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;    
  } 
  
  CVtkWriter writer;
  writer.WritePUXML(NEL,NVT,iKVERT,dcorvg,vu,vv,vw,vp,dist,node,time);
}

//----------------------------------------------------------------------------------------------

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,*ivl,*imst,*itst,*ismst);
}

//----------------------------------------------------------------------------------------------

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,*ivl,*imst,*itst,*ismst);
}

//----------------------------------------------------------------------------------------------

extern "C" void dumpworld()
{
  std::cout<<myWorld<<std::endl;
}

//----------------------------------------------------------------------------------------------

extern "C" void dumprigidbodies()
{
  RigidBodyIO writer;
  writer.write(myWorld,"particles.i3d");
}

//----------------------------------------------------------------------------------------------

extern "C" void logposition()
{
  std::vector<RigidBody*>::iterator vIter;
  //Check every pair
  mylog.Write("Simulation time: %f",myTimeControl.GetTime());
  int count = 0;
  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++)
  {
    RigidBody *body =*vIter;
    mylog.Write("Position of body %i: %f %f %f:",count,body->com_.x,body->com_.y,body->com_.z);
    count++;
  } 
}

//----------------------------------------------------------------------------------------------

extern "C" void logvelocity()
{
  bull.storeVertices();
}

//----------------------------------------------------------------------------------------------

extern "C" void gettype(int *itype, int *iid)
{
  int i = *iid;
  *itype = myWorld.rigidBodies_[i]->shapeId_;
}
