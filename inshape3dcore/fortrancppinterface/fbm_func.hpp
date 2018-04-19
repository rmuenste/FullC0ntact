#include <cppinterface.h>

#ifdef WITH_ODE
extern "C" void ode_get_position_()
{

  int id = 0;
  Vector3<Real> vec(0.0, 0.0, 2.5);
  int in=0;

  RigidBody *body = myWorld.rigidBodies_[id];

  //#ifdef WITH_ODE
  BodyODE &b = myWorld.bodies_[body->odeIndex_];

  const double *pos = dBodyGetPosition(b._bodyId);

  body->com_.x = pos[0];
  body->com_.y = pos[1];
  body->com_.z = pos[2];

  if(myWorld.parInfo_.getId()==1)
  {
    std::cout << body->com_ << std::endl;
  }

  //check if inside, if so then leave the function
  if(body->isInBody(vec))
  {
    in=1;
  }

  if(myWorld.parInfo_.getId()==1)
  {
    std::cout << "Inside: " << in << std::endl;
  }

}

extern "C" void ode_get_velocity_()
{

  int id = 0;

  RigidBody *body = myWorld.rigidBodies_[id];

  BodyODE &b = myWorld.bodies_[body->odeIndex_-1];

  const double *pos = dBodyGetLinearVel(b._bodyId);

  body->velocity_.x = pos[0];
  body->velocity_.y = pos[1];
  body->velocity_.z = pos[2];

//  if(myWorld.parInfo_.getId()==1)
//  {
//    std::cout << body->velocity_ << std::endl;
//  }

}
#endif
//-------------------------------------------------------------------------------------------------------

extern "C" void updateelementsprev(int *ibody)
{
  int i  = *ibody; 
  int e1 = myWorld.rigidBodies_[i]->elements_.size();
  int e2 = myWorld.rigidBodies_[i]->boundaryElements_.size();
  int itotal = e1+e2;
  myWorld.rigidBodies_[i]->elementsPrev_ = itotal;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setdomainbox(double vmin[3], double vmax[3])
{
  boxDomain = AABB3r(VECTOR3(vmin[0],vmin[1],vmin[2]),VECTOR3(vmax[0],vmax[1],vmax[2]));
}

//-------------------------------------------------------------------------------------------------------

extern "C" void elementsize(double element[][3], double *size)
{

  VECTOR3 elementMin(element[0][0],element[0][1],element[0][2]);
  VECTOR3 elementMax(element[0][0],element[0][1],element[0][2]);  

  for(int i=1;i<8;i++)
  {
    if(elementMin.x > element[i][0])
      elementMin.x = element[i][0];

    if(elementMin.y > element[i][1])
      elementMin.y = element[i][1];

    if(elementMin.z > element[i][2])
      elementMin.z = element[i][2];    

    if(elementMax.x < element[i][0])
      elementMax.x = element[i][0];

    if(elementMax.y < element[i][1])
      elementMax.y = element[i][1];

    if(elementMax.z < element[i][2])
      elementMax.z = element[i][2];            
  }

  AABB3r gridElement = AABB3r(elementMin,elementMax);

  //printf("extends %f %f %f \n",gridElement.m_Extends[0],gridElement.m_Extends[1],gridElement.m_Extends[2]); 

  *size = gridElement.getBoundingSphereRadius();

}

//-------------------------------------------------------------------------------------------------------

struct sortSizes {
  bool operator()(const std::pair<Real,int> &a, const std::pair<Real,int> &b)
  {
    return a.first < b.first;
  }
};

extern "C" void setelementarray(double elementsize[], int *iel)
{

  int isize = *iel;

  std::list< std::pair<Real,int> > sizes;

  for(int i=0;i<isize;i++)
  {
    sizes.push_back( std::pair<Real,int>(elementsize[i],i+1));
  }

  sizes.sort(sortSizes());
  std::vector<int> vDistribution;
  std::vector<Real> vGridSizes;
  double factor = 1.75;
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();

  double tsize = factor * ((*liter).first);
  liter++;
  int levels=0;
  int elemPerLevel=1;
  double lastsize=0.0;
  double dsize=0.0;
  for(;liter!=sizes.end();liter++)
  {
    dsize=((*liter).first);
    if(dsize > tsize)
    {
      vGridSizes.push_back(lastsize);
      tsize=factor*dsize;
      lastsize=dsize;
      vDistribution.push_back(elemPerLevel);
      elemPerLevel=1;

    }
    else
    {
      lastsize=dsize;
      elemPerLevel++;
    }
  }

  vGridSizes.push_back(lastsize);
  vDistribution.push_back(elemPerLevel);

  levels=vDistribution.size();

  int totalElements=0;
  for(int j=0;j<vDistribution.size();j++)
  {
    totalElements+=vDistribution[j];
  }

  AABB3r boundingBox = boxDomain;

  myUniformGrid.initGrid(boundingBox,levels);

  for(int j=0;j<vGridSizes.size();j++)
  {
    std::cout<<"Building level: "<<j+1<<" size: "<<vGridSizes[j]<<"\n";
    myUniformGrid.initGridLevel(j,2.0*vGridSizes[j]);
  }

  std::cout<<"Total elements = "<<totalElements<<" = "<<isize<<"\n";

  CVtkWriter writer;
  for(int j=0;j<levels;j++)
  {
    std::ostringstream sGrid;
    std::string sNameGrid("_vtk/uniform_level");
    sGrid<<"."<<std::setfill('0')<<std::setw(2)<<j<<".node."<<std::setfill('0')<<std::setw(2)<<myWorld.parInfo_.getId()<<".vtk";
    sNameGrid.append(sGrid.str());

    //Write the grid to a file and measure the time
    writer.WriteUniformGrid(myUniformGrid.levels_[j],sNameGrid.c_str());
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void setelementarray2(double elementsize[], int *iel)
{

  int isize = *iel;

  std::list< std::pair<Real,int> > sizes;

  for(int i=0;i<isize;i++)
  {
    sizes.push_back( std::pair<Real,int>(elementsize[i],i+1));
  }

  sizes.sort(sortSizes());
  std::vector<int> vDistribution;
  std::vector<Real> vGridSizes;
  double factor = 1.75;
  std::list< std::pair<Real,int> >::iterator liter = sizes.begin();

  double tsize = factor * ((*liter).first);
  liter++;
  int levels=0;
  int elemPerLevel=1;
  double lastsize=0.0;
  double dsize=0.0;
  for(;liter!=sizes.end();liter++)
  {
    dsize=((*liter).first);
    if(dsize > tsize)
    {
      vGridSizes.push_back(lastsize);
      tsize=factor*dsize;
      lastsize=dsize;
      vDistribution.push_back(elemPerLevel);
      elemPerLevel=1;

    }
    else
    {
      lastsize=dsize;
      elemPerLevel++;
    }
  }

  vGridSizes.push_back(lastsize);
  vDistribution.push_back(elemPerLevel);

  levels=vDistribution.size();

  int totalElements=0;
  for(int j=0;j<vDistribution.size();j++)
  {
    //      std::cout<<vDistribution[j]<< " elements on level: "<<j+1<<"\n";
    totalElements+=vDistribution[j];
  }

  AABB3r boundingBox = boxDomain;

  myUniformGrid.initGrid(boundingBox,levels);

  for(int j=0;j<vGridSizes.size();j++)
  {
    std::cout<<"Building level: "<<j+1<<" size: "<<vGridSizes[j]<<"\n";
    myUniformGrid.initGridLevel(j,2.0*vGridSizes[j]);
  }

  std::cout<<"Total elements = "<<totalElements<<" = "<<isize<<"\n";

  CVtkWriter writer;
  for(int j=0;j<levels;j++)
  {
    std::ostringstream sGrid;
    std::string sNameGrid("_vtk/uniform_level");
    sGrid<<"."<<std::setfill('0')<<std::setw(2)<<j<<".node."<<std::setfill('0')<<std::setw(2)<<myWorld.parInfo_.getId()<<".vtk";
    sNameGrid.append(sGrid.str());

    //Write the grid to a file and measure the time
    writer.WriteUniformGrid(myUniformGrid.levels_[j],sNameGrid.c_str());
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_querystatus()
{

  for(int j=0;j<myUniformGrid.nLevels_;j++)
  {
    std::cout<<"Level: "<<j+1<<" Element check: "<<myUniformGrid.levels_[j].getNumEntries()<<"\n";
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_resetuniformgrid()
{
  myUniformGrid.reset();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_insertelement(int *iel, double center[3], double *size)
{

  std::cout << "Function ug_insertelement is deprecated: " << __FILE__ << " line: " << __LINE__<< std::endl; 
  std::exit(EXIT_FAILURE);

//  int myiel = *iel;
//  VECTOR3 ele(center[0],center[1],center[2]);
//  Real mysize = *size;
//
//  myUniformGrid.insertElement(myiel,ele,2.0*mysize);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_pointquery(double center[3], int *iiel)
{

  std::cout << "Function ug_pointquery is deprecated: " << __FILE__ << " line: " << __LINE__<< std::endl; 
  std::exit(EXIT_FAILURE);

//  g_iElements.clear();
//  VECTOR3 q(center[0],center[1],center[2]);  
//  myUniformGrid.pointQuery(q,g_iElements);
//  *iiel=g_iElements.size();

}

//-------------------------------------------------------------------------------------------------------

extern "C" void ug_getelements(int ielem[])
{

  std::cout << "Function ug_getlements is deprecated: " << __FILE__ << " line: " << __LINE__<< std::endl; 
  std::exit(EXIT_FAILURE);

//  std::list<int>::iterator i = g_iElements.begin();
//  for(int j=0;i!=g_iElements.end();i++,j++)
//  {
//    ielem[j]=(*i);
//  }
//  g_iElements.clear();

}

//-------------------------------------------------------------------------------------------------------

extern "C" void inituniformgrid(double vmin[3], double vmax[3], double element[][3])
{
  AABB3r boundingBox = AABB3r(VECTOR3(vmin[0],vmin[1],vmin[2]),VECTOR3(vmax[0],vmax[1],vmax[2]));

  VECTOR3 elementMin(element[0][0],element[0][1],element[0][2]);
  VECTOR3 elementMax(element[0][0],element[0][1],element[0][2]);  

  for(int i=1;i<8;i++)
  {
    if(elementMin.x > element[i][0])
      elementMin.x = element[i][0];

    if(elementMin.y > element[i][1])
      elementMin.y = element[i][1];

    if(elementMin.z > element[i][2])
      elementMin.z = element[i][2];    

    if(elementMax.x < element[i][0])
      elementMax.x = element[i][0];

    if(elementMax.y < element[i][1])
      elementMax.y = element[i][1];

    if(elementMax.z < element[i][2])
      elementMax.z = element[i][2];            
  }

  AABB3r gridElement = AABB3r(elementMin,elementMax);
  //myUniformGrid.InitGrid(boundingBox,gridElement);

}

//-------------------------------------------------------------------------------------------------------

extern "C" void uniformgridinsert(int *iel, double center[3])
{
  int elem = *iel;
  //myUniformGrid.Insert(elem,VECTOR3(center[0],center[1],center[2]));    
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setmyid(int *myid)
{
  int id = *myid;
  myWorld.parInfo_.setId(id);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void intersectdomainbody(int *ibody,int *domain,int *intersection)
{
  // call the intersector for the bounding box of the body and 
  // and the domain bounding box
  int i = *ibody;
  RigidBody *pBody  = myWorld.rigidBodies_[i];
  AABB3r boxBody    = pBody->getAABB();
  CIntersector2AABB<Real> intersector(boxDomain,boxBody);
  bool bIntersect =  intersector.Intersection();
  if(bIntersect)
    *intersection=1;
  else
    *intersection=0;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void vertexorderxyz(int *invt,int iorder[], double dcorvg[][3])
{
  int nvt = *invt;
  std::vector<sPerm> permArray;

  for(int i=0;i<nvt;i++)
  { 
    sPerm perm;
    Real x = dcorvg[0][i];
    Real y = dcorvg[1][i];
    Real z = dcorvg[2][i];
    Vector3<Real> vec(x,y,z);        
    perm.vVec =  vec;
    perm.index  = i;
    permArray.push_back(perm);
  }

  std::stable_sort(permArray.begin(),permArray.end(),funcz());
  std::stable_sort(permArray.begin(),permArray.end(),funcy());
  std::stable_sort(permArray.begin(),permArray.end(),funcx());

  for(int i=0;i<nvt;i++)
  {
    iorder[i]=permArray[i].index;
  }  

}

//-------------------------------------------------------------------------------------------------------

extern "C" void updateMax0(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];
  MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);

  // compute distance point triangle
  int k = resMaxM1.iTriangleID;
  Triangle3<Real> &tri3 = resMaxM1.pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();  

  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent          = &resMax0;
}

//-----------------------------------------------------------------------------------------------

extern "C" void setMaxM1(double *dx,double *dy,double *dz,double *dist)
{ 
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];
  MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  
  resMaxM1.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMaxM1.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMaxM1.pNode       = distMeshPoint.m_Res.pNode;
}
//-------------------------------------------------------------------------------------------------------

extern "C" void setstartbb(double *dx,double *dy,double *dz,double *dist)
{
  // Calculate the distance using RefPoint@nlmax-1
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = 0;
  // Coordinates of the reference point nlmax-1
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];
  MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);
  ddist = distMeshPoint.ComputeDistanceSqr();
  *dist=ddist;  

  resMax0.iTriangleID = distMeshPoint.m_Res.iTriangleID;
  resMax0.m_pBVH      = distMeshPoint.m_Res.m_pBVH;
  resMax0.pNode       = distMeshPoint.m_Res.pNode;
  resCurrent           = &resMax0;
}

//-----------------------------------------------------------------------------------------------

extern "C" void getdistancebbid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];
  MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
  CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);

  // compute the distance to the triangle found for the reference point
  int k = resCurrent->iTriangleID;
  Triangle3<Real> &tri3 = resCurrent->pNode->m_Traits.m_vTriangles[k];
  CDistancePointTriangle<Real> distPointTri(tri3,vec);
  Real distTriangle = distPointTri.ComputeDistance();    

  ddist = distMeshPoint.ComputeDistanceCoSqr(distTriangle);
  *dist=ddist;  
}
//-------------------------------------------------------------------------------------------------------

extern "C" void clearelementlists(int *ibody)
{
  int i = *ibody;
  myWorld.rigidBodies_[i]->elements_.clear();
  myWorld.rigidBodies_[i]->boundaryElements_.clear();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void checkuniformgrid(int *ibody)
{
  int i = *ibody;
  myWorld.rigidBodies_[i]->elements_.clear();

}

//-------------------------------------------------------------------------------------------------------

void addelement2list(int *iel, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  myWorld.rigidBodies_[i]->elements_.push_back(ielc);
}

//-------------------------------------------------------------------------------------------------------

void addelement2bndlist(int *iel, int *idofs, int *ibody)
{
  int i = *ibody;
  int ielc = *iel;
  int idofsc = *idofs;
  myWorld.rigidBodies_[i]->boundaryElements_.push_back(std::pair<int,int>(ielc,idofsc));
}

//-------------------------------------------------------------------------------------------------------

void getelementarray(int* elements, int *idofselement, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.rigidBodies_[i]->boundaryElements_.begin();
  for(int j=0;iter!=myWorld.rigidBodies_[i]->boundaryElements_.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
    idofselement[j]= mypair.second;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void gettotalelements(int* nel, int* ibody)
{
  int i = *ibody;
  int e1 = myWorld.rigidBodies_[i]->elements_.size();
  int e2 = myWorld.rigidBodies_[i]->boundaryElements_.size();
  int itotal = e1+e2;
  *nel = itotal;
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsprev(int* nel, int* ibody)
{
  int i = *ibody;
  *nel  = myWorld.rigidBodies_[i]->elementsPrev_;
}

//-------------------------------------------------------------------------------------------------------

void getallelements(int* elements, int* ibody)
{
  int i = *ibody;
  std::list< std::pair<int,int> >::iterator iter = myWorld.rigidBodies_[i]->boundaryElements_.begin();
  std::list<int>::iterator iter2 = myWorld.rigidBodies_[i]->elements_.begin();
  int j;
  for(j=0;iter!=myWorld.rigidBodies_[i]->boundaryElements_.end();iter++,j++)
  {
    std::pair<int,int> &mypair = *iter;
    elements[j]= mypair.first;
  }

  for(;iter2!=myWorld.rigidBodies_[i]->elements_.end();iter2++,j++)
  {
    int iel = *iter2;
    elements[j]= iel;
  }

}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelements(int* elements, int* ibody)
{
  int i = *ibody;
  std::list<int>::iterator iter = myWorld.rigidBodies_[i]->elements_.begin();
  int j;
  for(j=0;iter!=myWorld.rigidBodies_[i]->elements_.end();iter++,j++)
  {
    int iel = *iter;
    elements[j]= iel;
  }
}
//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsinside(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.rigidBodies_[i]->elements_.size();
}

//-------------------------------------------------------------------------------------------------------

extern "C" void getelementsbndry(int *iel, int *ibody)
{
  int i = *ibody;
  *iel = myWorld.rigidBodies_[i]->boundaryElements_.size();
}
//-------------------------------------------------------------------------------------------------------

void queryuniformgrid(int* ibody)
{
  int id = *ibody;
  myWorld.rigidBodies_[id]->elements_.clear();
  myUniformGrid.query(myWorld.rigidBodies_[id]);
  Real avgElements = 0.0;
}
//-------------------------------------------------------------------------------------------------------

extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist)
{

  CDistOps3 op;
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  Vector3<Real> vec(x,y,z);
  double dist=op.BruteForceDistance(Model, vec);
  *ddist=dist;

}//end getcenter

//-------------------------------------------------------------------------------------------------------

extern "C" void getdistanceid(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];

#ifdef WITH_ODE
  BodyODE &b = myWorld.bodies_[pBody->odeIndex_-1];

  const double *pos = dBodyGetPosition(b._bodyId);

  pBody->com_.x = pos[0];
  pBody->com_.y = pos[1];
  pBody->com_.z = pos[2];
#endif

  if(pBody->shapeId_ == RigidBody::MESH)
  {
    ddist = 0;
    *dist = ddist;
    return;
    MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
    CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);
    ddist = distMeshPoint.ComputeDistance();
    *dist=ddist;
  }
  else if (pBody->shapeId_ == RigidBody::CGALMESH)
  {
    ddist = pBody->getMinimumDistance(vec);
    *dist=ddist;
  }
  else if(pBody->shapeId_ == RigidBody::CYLINDER)
  {
    VECTOR3 vLocal = vec - pBody->com_;
    MATRIX3X3 trans = pBody->getTransformationMatrix();
    trans.TransposeMatrix();
    vLocal = trans * vLocal ;    

    Cylinder<Real> *cylinder = dynamic_cast< Cylinder<Real> *>(pBody->shape_);
    CDistancePointCylinder<Real> distCylMesh(vLocal,*cylinder);
    ddist = distCylMesh.ComputeDistance();
    *dist=ddist;
  }
  else if(pBody->shapeId_ == RigidBody::SPHERE)
  {
    VECTOR3 vLocal = vec - pBody->com_;
    Real ddd = vLocal.mag();

    Sphere<Real> *sphere = dynamic_cast< Sphere<Real> *>(pBody->shape_);
    ddd -= sphere->getRadius();

    *dist=ddd;
  }
  else
  {

  }

}//end getdistance

//-------------------------------------------------------------------------------------------------------

void intersecbodyelement(int *ibody,int *iel,double vertices[][3])
{

  /*    if(body->m_iShape == RigidBody::BOUNDARYBOX)
        {
        return;
        }*/
  int i = *ibody;
  i--;
  RigidBody *body = myWorld.rigidBodies_[i];
  VECTOR3 verts[8];
  for(int i=0;i<8;i++)
  {
    verts[i]=VECTOR3(Real(vertices[i][0]),Real(vertices[i][1]),Real(vertices[i][2]));
  }
  int in = body->nDofsHexa(verts);

  if(in > 0)body->element_ = *iel;

}

//-------------------------------------------------------------------------------------------------------

void isboundarybody(int* isboundary, int* ibodyc)
{
  int i = *ibodyc;
  if(myWorld.rigidBodies_[i]->shapeId_==RigidBody::BOUNDARYBOX)
  {
    *isboundary = 1;
  }
  else
  {
    *isboundary = 0;
  }
}

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;  
  Vector3<Real> vec(x,y,z);

  int in=1;

  *isin=in;

}//end isinelement

//-------------------------------------------------------------------------------------------------------

extern "C" void isinelementperf(double *dx,double *dy,double *dz,int *isin)
{

  CDistOps3 op;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  Vector3<Real> point(x,y,z);
  int in=0;

  //locate the cell in that the point is
  SpatialHashHierarchy *pHash = dynamic_cast<SpatialHashHierarchy*>(myPipeline.broadPhase_->strategy_->implicitGrid_);    

  for(int level=0;level<=pHash->getMaxLevel();level++)
  {
    if(pHash->isBoundaryLevel(level))
      continue;

    //get the cell on the current level
    CellCoords cell = pHash->getCell(point,level);

    //get the entries of the cell
    std::vector<CSpatialHashEntry> *vEntries = pHash->getCellEntries(cell);

    //test for all entries if the point is inside
    //loop through the entries of the hash bucket
    std::vector<CSpatialHashEntry>::iterator viter = vEntries->begin();

    //check cell 
    for(;viter!=vEntries->end();viter++)
    {
      //get the rigid body
      RigidBody *pBody = viter->m_pBody;

      //check if inside, if so then leave the function
      if(pBody->isInBody(point))
      {
        in=1;
        *isin=pBody->iID_;
        return;
      }

    }//end for viter

  }//end for level

}//end isinelementperf

//-------------------------------------------------------------------------------------------------------

extern "C" void isinobstacle(double *dx,double *dy,double *dz,int *isin)
{
  int inside = 0;
  CDistOps3 op;
  int in[3];
  Vector3<float> vec(*dx,*dy,*dz);
  RigidBody *pBody0 = myWorld.rigidBodies_[0];
  MeshObject<i3d::Real> *object0 = dynamic_cast< MeshObject<i3d::Real> *>(pBody0->shape_);
  Model3D &model0 = object0->getModel();

  RigidBody *pBody1 = myWorld.rigidBodies_[1];
  MeshObject<i3d::Real> *object1 = dynamic_cast< MeshObject<i3d::Real> *>(pBody1->shape_);
  Model3D &model1 = object1->getModel();

  RigidBody *pBody2 = myWorld.rigidBodies_[2];
  MeshObject<i3d::Real> *object2 = dynamic_cast< MeshObject<i3d::Real> *>(pBody2->shape_);
  Model3D &model2 = object2->getModel();

  VECTOR3 orig(vec.x,vec.y,vec.z);
  Ray3r ray0(orig,VECTOR3(0,0,1));
  Ray3r ray1(orig,VECTOR3(0,0,-1));
  Ray3r ray2(orig,VECTOR3(0,1,0));

  in[0]=op.BruteForcefbm(model0, orig, ray0);
  in[1]=op.BruteForcefbm(model1, orig, ray1);
  in[2]=op.BruteForcefbm(model2, orig, ray2);

  for(int i=0;i<3;i++)
  {
    if(in[i]==1)
    {
      inside=1;
      break;
    }
  }
  *isin=inside;

}//end isinelement

//-------------------------------------------------------------------------------

extern "C" void writepolygon(int *iout)
{

}

//-------------------------------------------------------------------------------

void get_dynamics_type(int *iid, int *dynType)
{
  int id = *iid;
  RigidBody *b = myWorld.rigidBodies_[id];

  if (b->dType_ == DynamicsType::FULLY_DYNAMIC)
  {
    *dynType = 0;
  }
  else if (b->dType_ == DynamicsType::STATIC)
  {
    *dynType = 1;
  }
  else if (b->dType_ == DynamicsType::STATIC_COMPLEMENT)
  {
    *dynType = 2;
  }
  else if (b->dType_ == DynamicsType::ROTATIONAL)
  {
    *dynType = 3;
  }
  else if (b->dType_ == DynamicsType::ROTATIONAL_COMPLEMENT)
  {
    *dynType = 4;
  }
  else
  {
    *dynType = 5;
  }
 
}

//-------------------------------------------------------------------------------

extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin)
{

  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  Vector3<Real> vec(x,y,z);
  int in=0;

  RigidBody *body = myWorld.rigidBodies_[id];

#ifdef WITH_ODE
  BodyODE &b = myWorld.bodies_[body->odeIndex_-1];

  const double *pos = dBodyGetPosition(b._bodyId);

  body->com_.x = pos[0];
  body->com_.y = pos[1];
  body->com_.z = pos[2];

//  if(myWorld.parInfo_.getId()==1)
//  {
//    std::cout << ">" << body->com_; 
//    std::cout << "> " << body->odeIndex_ << std::endl; 
//  }
#endif
  if (body->shapeId_ == RigidBody::MESH)
  {
    //check if inside, if so then leave the function
    if(body->isInBody(vec))
    {
      in=1;
    }
  }
#ifdef WITH_CGAL
  else if (body->shapeId_ == RigidBody::CGALMESH)
  {
    // Generate a direction vector for the ray
    Vector dir = random_vector();

    Vec3 vDir(dir.x(), dir.y(), dir.z());

    //check if inside, if so then leave the function
    if(body->isInBody(vec, vDir))
    {
      in=1;
    }
  }
#endif 
  else
  {
    //check if inside, if so then leave the function
    if(body->isInBody(vec))
    {
      in=1;
    }
  }

  *isin=in;

}//end isinelement
