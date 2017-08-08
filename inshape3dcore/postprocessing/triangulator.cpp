#include "triangulator.h"
#include <cylinder.h>
#include <plane.h>

namespace i3d {

//template<>
//C3DModel CTriangulator<float, CSphere<float> >::Triangulate(const CSphere<float> &pShape)
//{
//  //-Pi/2 to Pi/2
//  float phi;
//  //0 to 2Pi
//  float theta;
//  //points on the sphere
//  //x=x0+r*cos(theta)*cos(phi)
//  //y=y0+r*cos(theta)*sin(phi)
//  //z=z0+r*sin(theta)
//
//  C3DModel model;
//
//  std::vector<Vector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//  int lat  =12;
//  int longi=12;
//
//  float dphi   = CMath<float>::SYS_PI/(float)longi;
//  float dtheta = CMath<float>::SYS_PI/(float)lat;
//  float halfpi = CMath<float>::SYS_PI/2.0;
//
//  Vector3<float> vTop=pShape.eval(halfpi,0);
//  Vector3<float> vBottom=pShape.eval(-halfpi,0);
//  vVertices.push_back(vTop);
//
//  phi  = halfpi-dphi;
//  for(int j=1;j<longi;j++)
//  {
//
//	theta=0.0f;
//	for(int i=0;i<2*lat;i++)
//	{
//	  Vector3<float> vNext=pShape.eval(phi,theta);
//	  vVertices.push_back(vNext);
//	  theta+=dtheta;
//	}//end for i
//	phi-=dphi;
//  }//end for j
//
//  vVertices.push_back(vBottom);
//
// // for(int i=0;i<vVertices.size();i++)
//	//cout<<vVertices[i]<<endl;
//
//  int lat2=2*lat;
//  //add upper triangle fan
//  for(int i=0;i<lat2;i++)
//  {
//	int verts[3];
//	verts[0]=0;
//	verts[1]=1+i;
//	verts[2]=1+(i+1)%lat2;
//	CTriFace face(verts);
//	vFaces.push_back(face);
//  }
//
//  //add body
//  for(int i=0;i<longi-2;i++)
//  {
//	int index=1+i*lat2;
//	for(int j=0;j<lat2;j++)
//	{
//	  int verts[3];
//	  verts[0]=index+j;
//	  verts[1]=index+lat2+j;
//	  verts[2]=index+(j+1)%lat2;
//
//	  CTriFace face1(verts);
//	  vFaces.push_back(face1);
//	  verts[0]=index+(j+1)%lat2;
//	  verts[1]=index+lat2+j;
//	  verts[2]=index+lat2+(j+1)%lat2;
//	  CTriFace face2(verts);
//	  vFaces.push_back(face2);
//	}
//  }
//  int ilast=vVertices.size()-1;
//  int ilastrow=ilast-lat2;
//  //add lower triangle fan
//  for(int i=0;i<lat2;i++)
//  {
//	int verts[3];
//	verts[0]=ilast;
//	verts[1]=ilastrow+(i+1)%lat2;
//	verts[2]=ilastrow+i;
//	CTriFace face(verts);
//	vFaces.push_back(face);
//  }
//
//  model.CreateFrom(vVertices,vFaces);
//
//  return model;
//}
// 
//template<>
//C3DModel CTriangulator<float, CEllipsoid<float> >::Triangulate(const CEllipsoid<float> &pShape)
//{
//  //-Pi/2 to Pi/2
//  float phi;
//  //0 to 2Pi
//  float theta;
//  //points on the sphere
//  //x=x0+r*cos(theta)*cos(phi)
//  //y=y0+r*cos(theta)*sin(phi)
//  //z=z0+r*sin(theta)
//
//  C3DModel model;
//
//  std::vector<Vector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//  int lat  =8;
//  int longi=8;
//
//  float dphi   = CMath<float>::SYS_PI/(float)longi;
//  float dtheta = CMath<float>::SYS_PI/(float)lat;
//  float halfpi = CMath<float>::SYS_PI/2.0;
//
//  Vector3<float> vTop=pShape.eval(halfpi,0);
//  Vector3<float> vBottom=pShape.eval(-halfpi,0);
//  vVertices.push_back(vTop);
//  
//  phi  = halfpi-dphi;
//  for(int j=1;j<longi;j++)
//  {
//
//	theta=0.0f;
//	for(int i=0;i<2*lat;i++)
//	{
//	  Vector3<float> vNext=pShape.eval(phi,theta);
//	  vVertices.push_back(vNext);
//	  theta+=dtheta;
//	}//end for i
//	phi-=dphi;
//  }//end for j
//
//  vVertices.push_back(vBottom);
//
// // for(int i=0;i<vVertices.size();i++)
//	//cout<<vVertices[i]<<endl;
//
//  int lat2=2*lat;
//  //add upper triangle fan
//  for(int i=0;i<lat2;i++)
//  {
//	int verts[3];
//	verts[0]=0;
//	verts[1]=1+i;
//	verts[2]=1+(i+1)%lat2;
//	CTriFace face(verts);
//	vFaces.push_back(face);
//  }
//
//  //add body
//  for(int i=0;i<longi-2;i++)
//  {
//	int index=1+i*lat2;
//	for(int j=0;j<lat2;j++)
//	{
//	  int verts[3];
//	  verts[0]=index+j;
//	  verts[1]=index+lat2+j;
//	  verts[2]=index+(j+1)%lat2;
//
//	  CTriFace face1(verts);
//	  vFaces.push_back(face1);
//	  verts[0]=index+(j+1)%lat2;
//	  verts[1]=index+lat2+j;
//	  verts[2]=index+lat2+(j+1)%lat2;
//	  CTriFace face2(verts);
//	  vFaces.push_back(face2);
//	}
//  }
//  int ilast=vVertices.size()-1;
//  int ilastrow=ilast-lat2;
//  //add lower triangle fan
//  for(int i=0;i<lat2;i++)
//  {
//	int verts[3];
//	verts[0]=ilast;
//	verts[1]=ilastrow+(i+1)%lat2;
//	verts[2]=ilastrow+i;
//	CTriFace face(verts);
//	vFaces.push_back(face);
//  }
//
//  model.CreateFrom(vVertices,vFaces);
//
//  return model;
//}
//
//template<>
//C3DModel CTriangulator<float, COBB3<float> >::Triangulate(const COBB3<float> &pShape)
//{
//
//  C3DModel model;
//
//  std::vector<Vector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//	Vector3<float> vertices[8];
//	pShape.ComputeVertices(vertices);
//	for(int i=0;i<8;i++)
//		vVertices.push_back(vertices[i]);
//	
//	//front faces
//	int verts[3];
//	verts[0]=0;
//	verts[1]=1;
//	verts[2]=5;
//	CTriFace face(verts);
//	vFaces.push_back(face);	
//	verts[0]=0;
//	verts[1]=5;
//	verts[2]=4;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	//back faces
//	verts[0]=3;
//	verts[1]=6;
//	verts[2]=7;
//	face=CTriFace(verts);
//	vFaces.push_back(face);	
//	verts[0]=3;
//	verts[1]=2;
//	verts[2]=6;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	//bottom faces
//	verts[0]=0;
//	verts[1]=1;
//	verts[2]=2;
//	face=CTriFace(verts);
//	vFaces.push_back(face);	
//	verts[0]=0;
//	verts[1]=2;
//	verts[2]=3;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	//top faces
//	verts[0]=4;
//	verts[1]=5;
//	verts[2]=6;
//	face=CTriFace(verts);
//	vFaces.push_back(face);	
//	verts[0]=4;
//	verts[1]=6;
//	verts[2]=7;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	//left faces
//	verts[0]=3;
//	verts[1]=7;
//	verts[2]=4;
//	face=CTriFace(verts);
//	vFaces.push_back(face);	
//	verts[0]=3;
//	verts[1]=4;
//	verts[2]=0;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	//right faces
//	verts[0]=1;
//	verts[1]=2;
//	verts[2]=6;
//	face=CTriFace(verts);
//	vFaces.push_back(face);	
//	verts[0]=1;
//	verts[1]=6;
//	verts[2]=5;
//	face=CTriFace(verts);
//	vFaces.push_back(face);
//	
//  model.CreateFrom(vVertices,vFaces);
//
//  return model;
//}
//

template<>
Model3D CTriangulator<Real, Sphere<Real> >::Triangulate(const Sphere<Real> &pShape)
{
  //-Pi/2 to Pi/2
  Real phi;
  //0 to 2Pi
  Real theta;
  //points on the sphere
  //x=x0+r*cos(theta)*cos(phi)
  //y=y0+r*cos(theta)*sin(phi)
  //z=z0+r*sin(theta)

  Model3D model;

  std::vector<Vector3<Real> > vVertices;
  std::vector<TriFace>         vFaces;

  int lat  =8;
  int longi=8;

  Real dphi   = CMath<Real>::SYS_PI/(Real)longi;
  Real dtheta = CMath<Real>::SYS_PI/(Real)lat;
  Real halfpi = CMath<Real>::SYS_PI/2.0;

  Vector3<Real> vTop=pShape.eval(halfpi,0);
  Vector3<Real> vBottom=pShape.eval(-halfpi,0);
  vVertices.push_back(vTop);

  phi  = halfpi-dphi;
  for(int j=1;j<longi;j++)
  {

	theta=0.0f;
	for(int i=0;i<2*lat;i++)
	{
	  Vector3<Real> vNext=pShape.eval(phi,theta);
	  vVertices.push_back(vNext);
	  theta+=dtheta;
	}//end for i
	phi-=dphi;
  }//end for j

  vVertices.push_back(vBottom);

 // for(int i=0;i<vVertices.size();i++)
	//cout<<vVertices[i]<<endl;

  int lat2=2*lat;
  //add upper triangle fan
  for(int i=0;i<lat2;i++)
  {
	int verts[3];
	verts[0]=0;
	verts[1]=1+i;
	verts[2]=1+(i+1)%lat2;
	TriFace face(verts);
	vFaces.push_back(face);
  }

  //add body
  for(int i=0;i<longi-2;i++)
  {
	int index=1+i*lat2;
	for(int j=0;j<lat2;j++)
	{
	  int verts[3];
	  verts[0]=index+j;
	  verts[1]=index+lat2+j;
	  verts[2]=index+(j+1)%lat2;

	  TriFace face1(verts);
	  vFaces.push_back(face1);
	  verts[0]=index+(j+1)%lat2;
	  verts[1]=index+lat2+j;
	  verts[2]=index+lat2+(j+1)%lat2;
	  TriFace face2(verts);
	  vFaces.push_back(face2);
	}
  }
  int ilast=vVertices.size()-1;
  int ilastrow=ilast-lat2;
  //add lower triangle fan
  for(int i=0;i<lat2;i++)
  {
	int verts[3];
	verts[0]=ilast;
	verts[1]=ilastrow+(i+1)%lat2;
	verts[2]=ilastrow+i;
	TriFace face(verts);
	vFaces.push_back(face);
  }

  model.createFrom(vVertices,vFaces);

  return model;
}
 
template<>
Model3D CTriangulator<Real, Ellipsoid<Real> >::Triangulate(const Ellipsoid<Real> &pShape)
{
  //-Pi/2 to Pi/2
  Real phi;
  //0 to 2Pi
  Real theta;
  //points on the sphere
  //x=x0+r*cos(theta)*cos(phi)
  //y=y0+r*cos(theta)*sin(phi)
  //z=z0+r*sin(theta)

  Model3D model;

  std::vector<Vector3<Real> > vVertices;
  std::vector<TriFace>         vFaces;

  int lat  =8;
  int longi=8;

  Real dphi   = CMath<Real>::SYS_PI/(Real)longi;
  Real dtheta = CMath<Real>::SYS_PI/(Real)lat;
  Real halfpi = CMath<Real>::SYS_PI/2.0;

  Vector3<Real> vTop=pShape.eval(halfpi,0);
  Vector3<Real> vBottom=pShape.eval(-halfpi,0);
  vVertices.push_back(vTop);
  
  phi  = halfpi-dphi;
  for(int j=1;j<longi;j++)
  {

	theta=0.0f;
	for(int i=0;i<2*lat;i++)
	{
	  Vector3<Real> vNext=pShape.eval(phi,theta);
	  vVertices.push_back(vNext);
	  theta+=dtheta;
	}//end for i
	phi-=dphi;
  }//end for j

  vVertices.push_back(vBottom);

 // for(int i=0;i<vVertices.size();i++)
	//cout<<vVertices[i]<<endl;

  int lat2=2*lat;
  //add upper triangle fan
  for(int i=0;i<lat2;i++)
  {
	int verts[3];
	verts[0]=0;
	verts[1]=1+i;
	verts[2]=1+(i+1)%lat2;
	TriFace face(verts);
	vFaces.push_back(face);
  }

  //add body
  for(int i=0;i<longi-2;i++)
  {
	int index=1+i*lat2;
	for(int j=0;j<lat2;j++)
	{
	  int verts[3];
	  verts[0]=index+j;
	  verts[1]=index+lat2+j;
	  verts[2]=index+(j+1)%lat2;

	  TriFace face1(verts);
	  vFaces.push_back(face1);
	  verts[0]=index+(j+1)%lat2;
	  verts[1]=index+lat2+j;
	  verts[2]=index+lat2+(j+1)%lat2;
	  TriFace face2(verts);
	  vFaces.push_back(face2);
	}
  }
  int ilast=vVertices.size()-1;
  int ilastrow=ilast-lat2;
  //add lower triangle fan
  for(int i=0;i<lat2;i++)
  {
	int verts[3];
	verts[0]=ilast;
	verts[1]=ilastrow+(i+1)%lat2;
	verts[2]=ilastrow+i;
	TriFace face(verts);
	vFaces.push_back(face);
  }

  model.createFrom(vVertices,vFaces);

  return model;
}

template<>
Model3D CTriangulator<Real, OBB3<Real> >::Triangulate(const OBB3<Real> &pShape)
{

  Model3D model;

  std::vector<Vector3<Real> > vVertices;
  std::vector<TriFace>         vFaces;

	Vector3<Real> vertices[8];
	pShape.computeVertices(vertices);
	for(int i=0;i<8;i++)
		vVertices.push_back(vertices[i]);
	
	//front faces
	int verts[3];
	verts[0]=0;
	verts[1]=1;
	verts[2]=5;
	TriFace face(verts);
	vFaces.push_back(face);	
	verts[0]=0;
	verts[1]=5;
	verts[2]=4;
	face=TriFace(verts);
	vFaces.push_back(face);
	//back faces
	verts[0]=3;
	verts[1]=6;
	verts[2]=7;
	face=TriFace(verts);
	vFaces.push_back(face);	
	verts[0]=3;
	verts[1]=2;
	verts[2]=6;
	face=TriFace(verts);
	vFaces.push_back(face);
	//bottom faces
	verts[0]=0;
	verts[1]=1;
	verts[2]=2;
	face=TriFace(verts);
	vFaces.push_back(face);	
	verts[0]=0;
	verts[1]=2;
	verts[2]=3;
	face=TriFace(verts);
	vFaces.push_back(face);
	//top faces
	verts[0]=4;
	verts[1]=5;
	verts[2]=6;
	face=TriFace(verts);
	vFaces.push_back(face);	
	verts[0]=4;
	verts[1]=6;
	verts[2]=7;
	face=TriFace(verts);
	vFaces.push_back(face);
	//left faces
	verts[0]=3;
	verts[1]=7;
	verts[2]=4;
	face=TriFace(verts);
	vFaces.push_back(face);	
	verts[0]=3;
	verts[1]=4;
	verts[2]=0;
	face=TriFace(verts);
	vFaces.push_back(face);
	//right faces
	verts[0]=1;
	verts[1]=2;
	verts[2]=6;
	face=TriFace(verts);
	vFaces.push_back(face);	
	verts[0]=1;
	verts[1]=6;
	verts[2]=5;
	face=TriFace(verts);
	vFaces.push_back(face);
	
  model.createFrom(vVertices,vFaces);

  return model;
}

template<>
Model3D CTriangulator<Real, Cylinder<Real> >::Triangulate(const Cylinder<Real> &pShape)
{

  Vector3<Real> center  = pShape.getCenter();
  Vector3<Real> u       = pShape.getU();
  Real height2           = pShape.getHalfLength();
  Real rad               = pShape.getRadius();

  Model3D model;

  std::vector<Vector3<Real> > vVertices;
  std::vector<TriFace>         vFaces;

  int verticalsegments = 2;
  int pointsoncircle   = 24;

  Real dalpha = 2.0 * CMath<Real>::SYS_PI/(Real)pointsoncircle;

  Vector3<Real> vTop    = center + (height2 * u);
  Vector3<Real> vBottom = center - (height2 * u);
  vVertices.push_back(vTop);

  Real dheight = (2.0 * height2)/Real(verticalsegments+1);
  Real currentheight = center.z + height2;
  Real alpha = 0.0;

  //create the vertices
  for(int j=0;j<(verticalsegments+2);j++)
  {
    for(int i=0;i<pointsoncircle;i++)
    {
      Vector3<Real> vNext=pShape.eval(alpha);
      vNext.z = currentheight;
      vVertices.push_back(vNext);
      alpha+=dalpha;
    }
    alpha=0.0;
    currentheight-=dheight;
  }

  vVertices.push_back(vBottom);

  //add the top triangle fan
  for(int i=0;i<pointsoncircle;i++)
  {
    int verts[3];
    verts[0]=0;
    verts[1]=1+i;
    verts[2]=1+(i+1)%pointsoncircle;
    TriFace face(verts);
    vFaces.push_back(face);
  }

  //add the body of the cylinder
  int index = 1;
  for(int i=0;i<verticalsegments+1;i++)
  {
	  int index=1+i*pointsoncircle;
	  for(int j=0;j<pointsoncircle;j++)
	  {
	    int verts[3];
	    verts[0]=index+j;
	    verts[1]=index+pointsoncircle+j;
	    verts[2]=index+(j+1)%pointsoncircle;

	    TriFace face1(verts);
	    vFaces.push_back(face1);
	    verts[0]=index+(j+1)%pointsoncircle;
	    verts[1]=index+pointsoncircle+j;
	    verts[2]=index+pointsoncircle+(j+1)%pointsoncircle;
	    TriFace face2(verts);
	    vFaces.push_back(face2);
	  }
  }//end for i

  int ilast=vVertices.size()-1;
  int ilastrow=ilast-pointsoncircle;
  //add lower triangle fan
  for(int i=0;i<pointsoncircle;i++)
  {
	int verts[3];
	verts[0]=ilast;
	verts[1]=ilastrow+(i+1)%pointsoncircle;
	verts[2]=ilastrow+i;
	TriFace face(verts);
	vFaces.push_back(face);
  }

  model.createFrom(vVertices,vFaces);

  return model;

}

template<>
Model3D CTriangulator<Real, Plane<Real> >::Triangulate(const Plane<Real> &pShape)
{

//  Vector3<Real> center  = pShape.getCenter();
//  Vector3<Real> u       = pShape.getU();
//  Real height2           = pShape.getHalfLength();
//  Real rad               = pShape.getRadius();
  Vector3<Real> n(pShape._a, pShape._b, pShape._c);
  Vector3<Real> u, v;

  if (std::abs(n.x - 1.0) < CMath<Real>::TOLERANCEZERO)
  {
    std::cout << "x" << std::endl;
  }
  else if (std::abs(n.y - 1.0) < CMath<Real>::TOLERANCEZERO)
  {
    std::cout << "y" << std::endl;
  }
  else
  {
    u = Vector3<Real>(1, 0, 0);
    v = Vector3<Real>(0, 1, 0);
  }

  Vec3 center(0, 0, 0);

  Model3D model;

  std::vector<Vector3<Real> > vVertices;
  std::vector<TriFace>         vFaces;

  vVertices.push_back(center + 10.0 * u + 10.0 * v);
  vVertices.push_back(center - 10.0 * u + 10.0 * v);
  vVertices.push_back(center - 10.0 * u - 10.0 * v);
  vVertices.push_back(center + 10.0 * u - 10.0 * v);

  int verts[3];
  verts[0] = 0;
  verts[1] = 1;
  verts[2] = 2;

  TriFace face(verts);
  vFaces.push_back(face);

  verts[0] = 2;
  verts[1] = 3;
  verts[2] = 0;
  face = TriFace(verts);
  vFaces.push_back(face);

  model.createFrom(vVertices,vFaces);

  return model;

}


}
