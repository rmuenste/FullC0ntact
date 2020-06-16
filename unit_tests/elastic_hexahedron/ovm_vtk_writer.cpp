#include <ovm_vtk_writer.hpp>
#include <ostream>
#include <sstream>
#include <iomanip>
#include <filesystem>
namespace fs = std::experimental::filesystem;

void OpenVolMeshVtkWriter::writeUnstructuredVTK(HexMesh & mesh, const std::string & fileName, int iTimestep)
{
    using namespace std;
    int iRangeMin,iRangeMax;
    double dRangeMin,dRangeMax;

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
