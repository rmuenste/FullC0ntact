#pragma once

#include <ostream>
#include <fstream>
#include <iostream>

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

int main(int _argc, char** _argv);


void write_sol_length(int iout, int istep, double simTime)
{

    std::string folder("_dump");
    folder.append("/processor_");

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
    
    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    std::ofstream out(n, std::ios::out | std::ofstream::app);
    
    if(!out)
    {
      
      std::cout << "Cannot open file: "<< n << std::endl;
      std::exit(EXIT_FAILURE);
    }

    out << simTime << "\n";
    out << istep << "\n";

    out.close();

}

void calculateLength(HexMesh &mesh, ScalarType time) {

  VertexType v0 = mesh.vertex(OpenVolumeMesh::VertexHandle(0));
  VertexType v1 = mesh.vertex(OpenVolumeMesh::VertexHandle(4));
  ScalarType l = (v1 - v0).length();

  //std::cout << "Vertex0 " << (v1 - v0).length() << std::endl;

  std::string folder("plot");

  if(!fs::exists(folder))
  {
    fs::create_directory(folder);
  }

  std::ostringstream nameField;
  nameField << folder << "/stretch.txt";

  std::string n(nameField.str());
  
  // Function: istream &read(char *buf, streamsize num)
  // Read in <num> chars from the invoking stream
  // into the buffer <buf>
  std::ofstream out;
  if(time == 0.0)
    out = std::ofstream(n, std::ios::out);
  else
    out = std::ofstream(n, std::ios::out | std::ofstream::app);
  
  if(!out)
  {
    
    std::cout << "Cannot open file: "<< n << std::endl;
    std::exit(EXIT_FAILURE);
  }

  out << time << " " << l << "\n";

  out.close();

}

ScalarType computeCellVolume(HexMesh &mesh, const OpenVolumeMesh::CellHandle &cellHandle) {

  OpenVolumeMesh::CellIter c_it = OpenVolumeMesh::CellIter(&mesh, cellHandle); 

  std::vector<OpenVolumeMesh::VertexHandle> vertices;

  OpenVolumeMesh::CellVertexIter cv_it((*c_it), &mesh);
  for (; cv_it.valid(); ++cv_it) {
    vertices.push_back(*cv_it);
  }

  // V = |(a-d)*((b-d)x(c-d))| * (1.0/6.0)
  int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
  ScalarType vol = ScalarType(0.0);
  for(int i(0); i < 5; ++i)
  {
    VertexType a = mesh.vertex(vertices[tetras[i][0]]);
    VertexType b = mesh.vertex(vertices[tetras[i][1]]);
    VertexType c = mesh.vertex(vertices[tetras[i][2]]);
    VertexType d = mesh.vertex(vertices[tetras[i][3]]);

    vol += std::abs(OpenVolumeMesh::dot( a-d, OpenVolumeMesh::cross((b-d), (c-d)))) * (1.0/6.0);

  }
  return vol;
}

void computeAllCellVolumes(HexMesh &mesh) {

  OpenVolumeMesh::CellIter c_it = mesh.cells_begin();

  OpenVolumeMesh::CellPropertyT<ScalarType> volProp = mesh.request_cell_property<ScalarType>("volume");

  for (; c_it != mesh.cells_end(); ++c_it) {
    ScalarType volume = computeCellVolume(mesh, *c_it);
    volProp[*c_it] = volume;
  }

}

void calculateAllParticleMasses(HexMesh &mesh, ScalarType rho) {

  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {

    ScalarType mass = 0.0;

    //ScalarType rho = 913.0; // [kg/m**3];

    OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
    mesh.request_vertex_property<ScalarType>("mass");

    OpenVolumeMesh::VertexPropertyT<std::string> fixedProp =
    mesh.request_vertex_property<std::string>("fixed");

    OpenVolumeMesh::CellPropertyT<ScalarType> volProp =
    mesh.request_cell_property<ScalarType>("volume");

    OpenVolumeMesh::VertexCellIter vc_it(*v_it, &mesh);
    for (; vc_it.valid(); ++vc_it) {
      ScalarType volume = volProp[*vc_it];
      mass += rho * volume;
    }

    mass *= 0.125;
    if (fixedProp[*v_it] != "xyz")
      massProp[*v_it] = mass;
    else
      massProp[*v_it] = 0.0;
  }
}

// std::cout << "xi: " << 2.0 * T1 / area << std::endl;
// std::cout << "eta: " <<  2.0 * T2 / area << std::endl;
ScalarType evaluateFaceBasisFunction(ScalarType xi, ScalarType eta, int vidx) {
  if(vidx == 0) return (1.0 - xi) * (1.0 - eta);
  else if(vidx == 1) return xi * (1.0 - eta);
  else if(vidx == 2) return xi * eta;
  else if(vidx == 3) return (1.0 - xi) * eta;
  else
    return 0.0;
}

ScalarType computeCellVolume(OpenVolumeMesh::CellIter &c_it, HexMesh &mesh) {

    std::vector<OpenVolumeMesh::VertexHandle> vertices;

    OpenVolumeMesh::CellVertexIter cv_it((*c_it), &mesh);
    for (; cv_it.valid(); ++cv_it) {
      vertices.push_back(*cv_it);
    }

    // V = |(a-d)*((b-d)x(c-d))| * (1.0/6.0)
    int tetras[5][4] = {{0,1,3,4},{1,2,3,6},{3,4,6,7},{1,4,5,6},{1,3,4,6}};
    ScalarType vol = ScalarType(0.0);
    for(int i(0); i < 5; ++i)
    {
      VertexType a = mesh.vertex(vertices[tetras[i][0]]);
      VertexType b = mesh.vertex(vertices[tetras[i][1]]);
      VertexType c = mesh.vertex(vertices[tetras[i][2]]);
      VertexType d = mesh.vertex(vertices[tetras[i][3]]);

      vol += std::abs(OpenVolumeMesh::dot( a-d, OpenVolumeMesh::cross((b-d), (c-d)))) * (1.0/6.0);

    }
    return vol;

}

//  std::cout << "Cell Volume = [" << hex.computeCellVolume(myMesh) << "]" << std::endl;
//  
//  std::cout << "----------------------------------------------------------- " << std::endl;
//  std::cout << "------------------Force Computation Example-----------------" << std::endl;
//  //std::cout << "Axis length = [" << zeta[0].parameters[0].first << "]" << std::endl;
//  LinearSpring<ScalarType> spring(ScalarType(1.0), hex.zeta_[0].getDir().length());
//  std::cout << "Linear Spring Force = [" << spring.evaluateForce(hex.zeta_[0].q[0], hex.zeta_[0].q[1]) << "]" << std::endl;
//  
//  ScalarType alphaZero = OpenVolumeMesh::dot(hex.zeta_[0].getDir(), hex.zeta_[1].getDir());
//  std::cout << "Linear Torsion Spring rest angle = [" << std::acos(alphaZero) << "]" << std::endl;
//
//  LinearTorsionSpring<ScalarType> torsionSpring(ScalarType(1.0), alphaZero);
//  std::pair<VertexType, VertexType> torsionForcePair = torsionSpring.evaluateTorsion(hex.zeta_[0].getDir(), hex.zeta_[1].getDir());
//  std::cout << "Linear Torsion Spring Force = [" << torsionForcePair.first << "]" << std::endl;

