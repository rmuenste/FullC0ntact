
//-------------------------------------------------------------------------------------------------------

extern "C" void writesoftbody(int *iout)
{
  int iTimestep=*iout;

  std::ostringstream line;
  line << "_vtk/line." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";

  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Output file: " <<
    termcolor::reset << line.str()  << std::endl;
  
  CVtkWriter writer;

  //Write the grid to a file and measure the time
  writer.WriteParamLine(softBody_.geom_, line.str().c_str());

}

//-------------------------------------------------------------------------------------------------------

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
  writer.WriteParticleFile(myWorld.rigidBodies_,sModel.c_str());

  std::string meshFile(sMesh.str());
  std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  "> Output file: " <<
    termcolor::reset << meshFile  << std::endl;

  line << "_vtk/line." << std::setfill('0') << std::setw(5) << iTimestep << ".vtk";
  
  //Write the grid to a file and measure the time
  writer.WriteRigidBodies(myWorld.rigidBodies_,meshFile.c_str());
  writer.WriteParamLine(bull.geom_, line.str().c_str());
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

//-------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------

extern "C" void writepvtu(int *iNodes,int *iTime)
{
  int nodes=*iNodes;
  int time =*iTime;
  CVtkWriter writer;
  writer.WritePVTU(nodes,time);
}

//-------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  CVtkWriter writer;
  writer.WriteVTK23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,*ivl,*imst,*itst,*ismst);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumpworld()
{
  std::cout<<myWorld<<std::endl;
  mylog.Write(myWorld.toString().c_str());
}

//-------------------------------------------------------------------------------------------------------

extern "C" void dumprigidbodies()
{
  RigidBodyIO writer;
  writer.write(myWorld,"particles.i3d");
}

//-------------------------------------------------------------------------------------------------------

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

//-------------------------------------------------------------------------------------------------------

extern "C" void logvelocity()
{
  bull.storeVertices();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void logangularvelocity()
{

}

//-------------------------------------------------------------------------------------------------------

extern "C" void logdistance()
{

}

//-------------------------------------------------------------------------------------------------------

extern "C" void logcollision()
{
  
}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettype(int *itype, int *iid)
{
  int i = *iid;
  *itype = myWorld.rigidBodies_[i]->shapeId_;
}
