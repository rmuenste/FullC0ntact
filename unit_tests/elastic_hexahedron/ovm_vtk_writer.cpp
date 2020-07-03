#include <ovm_vtk_writer.hpp>
#include <ostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

// file manager
#include <OpenVolumeMesh/FileManager/FileManager.hh>
#include <hexahedron_element.hpp>

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

void OpenVolMeshVtkWriter::writeHexMesh(HexMesh &mesh, const std::string &fileName, int iTimestep) {

  std::string folder("ovm");

  if (!fs::exists(folder)) {
    fs::create_directory(folder);
  }

  std::ostringstream sName;

  sName << folder << "/" << fileName << "_" << std::setfill('0') << std::setw(4) << iTimestep << ".ovm";

  OpenVolumeMesh::IO::FileManager fileManager;
  fileManager.writeFile(sName.str(), mesh);

}

void OpenVolMeshVtkWriter::writeUnstructuredVTK(HexMesh & mesh, const std::string & fileName, int iTimestep)
{
    using namespace std;
//    int iRangeMin,iRangeMax;
//    double dRangeMin,dRangeMax;

    std::string folder("vtk");

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    std::ostringstream sName;
    sName << folder << "/" << fileName << "_" << std::setfill('0')<<std::setw(4)<< iTimestep;

    string strEnding(".vtu");
    string strFileName(sName.str());
    strFileName.append(strEnding);

    OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
    mesh.request_vertex_property<ScalarType>("mass");

    OpenVolumeMesh::VertexPropertyT<VertexType> velProp =
    mesh.request_vertex_property<VertexType>("velocity");

    OpenVolumeMesh::VertexPropertyT<VertexType> externalForce =
    mesh.request_vertex_property<VertexType>("externalforce");

    //open file for writing
    FILE * myfile = fopen(strFileName.c_str(),"w");

    //check
    fprintf(myfile,"<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n");

    //Write the vtk header
    fprintf(myfile,"  <UnstructuredGrid>\n");

    //Next item is the number of points and cells
    fprintf(myfile,"    <Piece NumberOfPoints=\"%i\" NumberOfCells=\"%i\">\n", mesh.n_vertices(), mesh.n_cells());

    //write the pointdata 
    fprintf(myfile,"      <PointData Scalars=\"Mass\" Vectors=\"velocity\">\n");

    fprintf(myfile,"        <DataArray type=\"Float32\" Name=\"Mass\" format=\"ascii\" RangeMin=\"\" RangeMax=\"\">\n");

    for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
      v_it != mesh.vertices_end(); ++v_it) {
      fprintf(myfile,"          %f\n", massProp[*v_it]);
    }

    fprintf(myfile,"        </DataArray>\n");

    //write the pointdata velocity array
    fprintf(myfile,"        <DataArray type=\"Float32\" Name=\"velocity\" NumberOfComponents=\"3\" format=\"ascii\" RangeMin=\"\" RangeMax=\"\">\n");

    for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
      v_it != mesh.vertices_end(); ++v_it) {
      fprintf(myfile,"          %f %f %f\n", velProp[*v_it][0], velProp[*v_it][1], velProp[*v_it][2]);
    }

    fprintf(myfile,"        </DataArray>\n");

    fprintf(myfile,"      </PointData>\n");

    //write the points data array to the file
    fprintf(myfile,"      <Points>\n");
    fprintf(myfile,"        <DataArray type=\"Float32\" Name=\"Points\" NumberOfComponents=\"3\" format=\"ascii\" RangeMin=\"0\" RangeMax=\"1.0\">\n");

    for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
      v_it != mesh.vertices_end(); ++v_it) {
      fprintf(myfile,"          %f %f %f\n", mesh.vertex(*v_it)[0], mesh.vertex(*v_it)[1], mesh.vertex(*v_it)[2]);
    }

    fprintf(myfile,"        </DataArray>\n");
    fprintf(myfile,"      </Points>\n");

    //Next item is the cell data array
    fprintf(myfile,"      <Cells>\n");
    fprintf(myfile,"        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\" RangeMin=\"0\" RangeMax=\"%i\">\n", mesh.n_cells()-1);
    fprintf(myfile,"          ");

    OpenVolumeMesh::CellIter c_it = mesh.cells_begin();

    for (; c_it != mesh.cells_end(); ++c_it) {
      std::vector<int> cellIndices;
      OpenVolumeMesh::CellVertexIter cv_it((*c_it), &mesh);
      for (; cv_it.valid(); ++cv_it) {
        cellIndices.push_back(cv_it->idx());
      }

      fprintf(myfile,"          %i %i %i %i %i %i %i %i\n", cellIndices[0], 
                                                            cellIndices[1], 
                                                            cellIndices[2], 
                                                            cellIndices[3], 
                                                            cellIndices[4], 
                                                            cellIndices[5], 
                                                            cellIndices[6], 
                                                            cellIndices[7]);
    }

    fprintf(myfile,"        </DataArray>\n");

    //the offset seems to be mandatory
    fprintf(myfile,"        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\" RangeMin=\"8\" RangeMax=\"%i\">\n", mesh.n_cells()*8);
    fprintf(myfile,"          ");

    for(int i=1; i <= mesh.n_cells(); i++)
    {
      fprintf(myfile,"%i ",i*8);
      if(i%6==0)
      {
        fprintf(myfile,"\n");
        fprintf(myfile,"          ");
      }
    }

    if(mesh.n_cells()%6!=0)fprintf(myfile,"\n");
    fprintf(myfile,"        </DataArray>\n");

    //add a data array with the types of the cells
    fprintf(myfile,"        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\" RangeMin=\"12\" RangeMax=\"12\">\n");
    fprintf(myfile,"          ");

    for(int i=1; i<=mesh.n_cells(); i++)
    {
      fprintf(myfile,"12 ");
      if(i%6==0)
      {
        fprintf(myfile,"\n");
        fprintf(myfile,"          ");
      }
    }

    if(mesh.n_cells()%6!=0)	fprintf(myfile,"\n");
    fprintf(myfile,"        </DataArray>\n");

    //finish the cells and the piece
    fprintf(myfile,"      </Cells>\n");
    fprintf(myfile,"    </Piece>\n");

    //add the finish tags
    fprintf(myfile,"  </UnstructuredGrid>\n");
    fprintf(myfile,"</VTKFile>\n");

    fclose( myfile );

}


void OpenVolMeshVtkWriter::writeUnstructuredCellData(HexMesh & mesh, const std::string & fileName)
{
  using namespace std;
  FILE * myfile = fopen(fileName.c_str(),"w");

  if (myfile == NULL) {
    cout<<"Error opening file: "<<fileName<<endl;
    exit(0);
  }

  int i,j;

  fprintf(myfile,"# vtk DataFile Version 2.0\n");
  fprintf(myfile,"Generated by InShape 2.x\n");
  fprintf(myfile,"ASCII\n");
  fprintf(myfile,"DATASET UNSTRUCTURED_GRID\n");

  //total number of vertices
  int iVerts=0;
  //total number of polygons
  int iPolys=0;

  fprintf(myfile,"POINTS %i double\n", mesh.n_faces());

//  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
//    v_it != mesh.vertices_end(); ++v_it) {
//    fprintf(myfile,"%f %f %f\n", mesh.vertex(*v_it)[0], mesh.vertex(*v_it)[1], mesh.vertex(*v_it)[2]);
//  }

  for (OpenVolumeMesh::CellIter c_it = mesh.cells_begin();
    c_it != mesh.cells_end(); ++c_it) {

    for (OpenVolumeMesh::CellFaceIter cf_it = OpenVolumeMesh::CellFaceIter(*c_it, &mesh); cf_it.valid(); ++cf_it) {

      VertexType faceCenter(0, 0, 0);
      for (OpenVolumeMesh::FaceVertexIter fv_it = OpenVolumeMesh::FaceVertexIter(*cf_it, &mesh); fv_it.valid(); ++fv_it) {
        faceCenter += mesh.vertex(*fv_it);
      }
      faceCenter *= 0.25;
      fprintf(myfile, "%f %f %f\n", faceCenter[0], faceCenter[1], faceCenter[2]);
    }
  }

//  for (OpenVolumeMesh::CellIter c_it = mesh.cells_begin();
//    c_it != mesh.cells_end(); ++c_it) {
//    VertexType baryCenter(0, 0, 0);
//    for (OpenVolumeMesh::CellVertexIter cv_it = OpenVolumeMesh::CellVertexIter(*c_it, &mesh); cv_it.valid(); ++cv_it) {
//      baryCenter += mesh.vertex(*cv_it);
//    }
//    baryCenter *= 0.125;
//    fprintf(myfile, "%f %f %f\n", baryCenter[0], baryCenter[1], baryCenter[2]);
//  }

//  OpenVolumeMesh::VertexPropertyT<ScalarType> massProp =
//  mesh.request_vertex_property<ScalarType>("mass");

  OpenVolumeMesh::CellPropertyT<ScalarType> volProp =
  mesh.request_cell_property<ScalarType>("volume");

  OpenVolumeMesh::CellPropertyT<ElasticHexahedron<ScalarType>> elasticProp = 
  mesh.request_cell_property<ElasticHexahedron<ScalarType>>("elastic");


  fprintf(myfile,"POINT_DATA %i\n",mesh.n_faces());
  fprintf(myfile,"SCALARS volume double 1\n");
  fprintf(myfile,"LOOKUP_TABLE default\n");

  for (OpenVolumeMesh::CellIter c_it = mesh.cells_begin();
    c_it != mesh.cells_end(); ++c_it) {

    for (OpenVolumeMesh::CellHalfFaceIter cf_it = OpenVolumeMesh::CellHalfFaceIter(*c_it, &mesh); cf_it.valid(); ++cf_it) {

      VertexType faceCenter(0, 0, 0);
      for (OpenVolumeMesh::HalfFaceVertexIter fv_it = OpenVolumeMesh::HalfFaceVertexIter(*cf_it, &mesh); fv_it.valid(); ++fv_it) {
        faceCenter += mesh.vertex(*fv_it);
      }
      faceCenter *= 0.25;
      fprintf(myfile, "%f\n", faceCenter[0]);
    }
  }

//  for(OpenVolumeMesh::VertexIter v_it = mesh.vertices_begin();
//    v_it != mesh.vertices_end(); ++v_it) {
//    fprintf(myfile,"%f\n", massProp[*v_it]);
//  }
//  for (OpenVolumeMesh::CellIter c_it = mesh.cells_begin();
//    c_it != mesh.cells_end(); ++c_it) {
//    fprintf(myfile," %f\n", volProp[*c_it]);
//  }

  fprintf(myfile,"VECTORS normal double\n");
//  for()
//  {
//    fprintf(myfile,"%f %f %f\n",body.com_.x, body.com_.y, body.com_.z);
//  }
  for (OpenVolumeMesh::CellIter c_it = mesh.cells_begin();
    c_it != mesh.cells_end(); ++c_it) {

    for (OpenVolumeMesh::CellFaceIter cf_it = OpenVolumeMesh::CellFaceIter(*c_it, &mesh); cf_it.valid(); ++cf_it) {

      VertexType faceNormal = elasticProp[*c_it].computeHalfFaceNormal(*cf_it, mesh); 

      fprintf(myfile, "%f %f %f\n", faceNormal[0], faceNormal[1], faceNormal[2]);
    }
  }

//  fprintf(myfile,"VECTORS velocity double\n");
//  for()
//  {
//    fprintf(myfile,"%f %f %f\n",body.velocity_.x, body.velocity_.y, body.velocity_.z);
//  }

  fclose( myfile );

}


