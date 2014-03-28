#include "triangulator.h"
#include <cylinder.h>

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
//  std::vector<CVector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//  int lat  =12;
//  int longi=12;
//
//  float dphi   = CMath<float>::SYS_PI/(float)longi;
//  float dtheta = CMath<float>::SYS_PI/(float)lat;
//  float halfpi = CMath<float>::SYS_PI/2.0;
//
//  CVector3<float> vTop=pShape.eval(halfpi,0);
//  CVector3<float> vBottom=pShape.eval(-halfpi,0);
//  vVertices.push_back(vTop);
//
//  phi  = halfpi-dphi;
//  for(int j=1;j<longi;j++)
//  {
//
//	theta=0.0f;
//	for(int i=0;i<2*lat;i++)
//	{
//	  CVector3<float> vNext=pShape.eval(phi,theta);
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
//  std::vector<CVector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//  int lat  =8;
//  int longi=8;
//
//  float dphi   = CMath<float>::SYS_PI/(float)longi;
//  float dtheta = CMath<float>::SYS_PI/(float)lat;
//  float halfpi = CMath<float>::SYS_PI/2.0;
//
//  CVector3<float> vTop=pShape.eval(halfpi,0);
//  CVector3<float> vBottom=pShape.eval(-halfpi,0);
//  vVertices.push_back(vTop);
//  
//  phi  = halfpi-dphi;
//  for(int j=1;j<longi;j++)
//  {
//
//	theta=0.0f;
//	for(int i=0;i<2*lat;i++)
//	{
//	  CVector3<float> vNext=pShape.eval(phi,theta);
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
//  std::vector<CVector3<float> > vVertices;
//  std::vector<CTriFace>         vFaces;
//
//	CVector3<float> vertices[8];
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
C3DModel CTriangulator<Real, CSphere<Real> >::Triangulate(const CSphere<Real> &pShape)
{
  //-Pi/2 to Pi/2
  Real phi;
  //0 to 2Pi
  Real theta;
  //points on the sphere
  //x=x0+r*cos(theta)*cos(phi)
  //y=y0+r*cos(theta)*sin(phi)
  //z=z0+r*sin(theta)

  C3DModel model;

  std::vector<CVector3<double> > vVertices;
  std::vector<CTriFace>         vFaces;

  int lat  =8;
  int longi=8;

  Real dphi   = CMath<Real>::SYS_PI/(Real)longi;
  Real dtheta = CMath<Real>::SYS_PI/(Real)lat;
  Real halfpi = CMath<Real>::SYS_PI/2.0;

  CVector3<Real> vTop=pShape.eval(halfpi,0);
  CVector3<Real> vBottom=pShape.eval(-halfpi,0);
  vVertices.push_back(vTop);

  phi  = halfpi-dphi;
  for(int j=1;j<longi;j++)
  {

	theta=0.0f;
	for(int i=0;i<2*lat;i++)
	{
	  CVector3<Real> vNext=pShape.eval(phi,theta);
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
	CTriFace face(verts);
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

	  CTriFace face1(verts);
	  vFaces.push_back(face1);
	  verts[0]=index+(j+1)%lat2;
	  verts[1]=index+lat2+j;
	  verts[2]=index+lat2+(j+1)%lat2;
	  CTriFace face2(verts);
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
	CTriFace face(verts);
	vFaces.push_back(face);
  }

  model.CreateFrom(vVertices,vFaces);

  return model;
}
 
template<>
C3DModel CTriangulator<Real, Ellipsoid<Real> >::Triangulate(const Ellipsoid<Real> &pShape)
{
  //-Pi/2 to Pi/2
  Real phi;
  //0 to 2Pi
  Real theta;
  //points on the sphere
  //x=x0+r*cos(theta)*cos(phi)
  //y=y0+r*cos(theta)*sin(phi)
  //z=z0+r*sin(theta)

  C3DModel model;

  std::vector<CVector3<Real> > vVertices;
  std::vector<CTriFace>         vFaces;

  int lat  =8;
  int longi=8;

  Real dphi   = CMath<Real>::SYS_PI/(Real)longi;
  Real dtheta = CMath<Real>::SYS_PI/(Real)lat;
  Real halfpi = CMath<Real>::SYS_PI/2.0;

  CVector3<Real> vTop=pShape.eval(halfpi,0);
  CVector3<Real> vBottom=pShape.eval(-halfpi,0);
  vVertices.push_back(vTop);
  
  phi  = halfpi-dphi;
  for(int j=1;j<longi;j++)
  {

	theta=0.0f;
	for(int i=0;i<2*lat;i++)
	{
	  CVector3<Real> vNext=pShape.eval(phi,theta);
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
	CTriFace face(verts);
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

	  CTriFace face1(verts);
	  vFaces.push_back(face1);
	  verts[0]=index+(j+1)%lat2;
	  verts[1]=index+lat2+j;
	  verts[2]=index+lat2+(j+1)%lat2;
	  CTriFace face2(verts);
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
	CTriFace face(verts);
	vFaces.push_back(face);
  }

  model.CreateFrom(vVertices,vFaces);

  return model;
}

template<>
C3DModel CTriangulator<Real, COBB3<Real> >::Triangulate(const COBB3<Real> &pShape)
{

  C3DModel model;

  std::vector<CVector3<Real> > vVertices;
  std::vector<CTriFace>         vFaces;

	CVector3<Real> vertices[8];
	pShape.ComputeVertices(vertices);
	for(int i=0;i<8;i++)
		vVertices.push_back(vertices[i]);
	
	//front faces
	int verts[3];
	verts[0]=0;
	verts[1]=1;
	verts[2]=5;
	CTriFace face(verts);
	vFaces.push_back(face);	
	verts[0]=0;
	verts[1]=5;
	verts[2]=4;
	face=CTriFace(verts);
	vFaces.push_back(face);
	//back faces
	verts[0]=3;
	verts[1]=6;
	verts[2]=7;
	face=CTriFace(verts);
	vFaces.push_back(face);	
	verts[0]=3;
	verts[1]=2;
	verts[2]=6;
	face=CTriFace(verts);
	vFaces.push_back(face);
	//bottom faces
	verts[0]=0;
	verts[1]=1;
	verts[2]=2;
	face=CTriFace(verts);
	vFaces.push_back(face);	
	verts[0]=0;
	verts[1]=2;
	verts[2]=3;
	face=CTriFace(verts);
	vFaces.push_back(face);
	//top faces
	verts[0]=4;
	verts[1]=5;
	verts[2]=6;
	face=CTriFace(verts);
	vFaces.push_back(face);	
	verts[0]=4;
	verts[1]=6;
	verts[2]=7;
	face=CTriFace(verts);
	vFaces.push_back(face);
	//left faces
	verts[0]=3;
	verts[1]=7;
	verts[2]=4;
	face=CTriFace(verts);
	vFaces.push_back(face);	
	verts[0]=3;
	verts[1]=4;
	verts[2]=0;
	face=CTriFace(verts);
	vFaces.push_back(face);
	//right faces
	verts[0]=1;
	verts[1]=2;
	verts[2]=6;
	face=CTriFace(verts);
	vFaces.push_back(face);	
	verts[0]=1;
	verts[1]=6;
	verts[2]=5;
	face=CTriFace(verts);
	vFaces.push_back(face);
	
  model.CreateFrom(vVertices,vFaces);

  return model;
}

template<>
C3DModel CTriangulator<Real, Cylinder<Real> >::Triangulate(const Cylinder<Real> &pShape)
{

  CVector3<Real> center  = pShape.GetCenter();
  CVector3<Real> u       = pShape.GetU();
  Real height2           = pShape.GetHalfLength();
  Real rad               = pShape.GetRadius();

  C3DModel model;

  std::vector<CVector3<Real> > vVertices;
  std::vector<CTriFace>         vFaces;

  int verticalsegments = 1;
  int pointsoncircle   = 14;

  Real dalpha = 2.0 * CMath<Real>::SYS_PI/(Real)pointsoncircle;

  CVector3<Real> vTop    = center + (height2 * u);
  CVector3<Real> vBottom = center - (height2 * u);
  vVertices.push_back(vTop);

  Real dheight = (2.0 * height2)/Real(verticalsegments+1);
  Real currentheight = center.z + height2;
  Real alpha = 0.0;

  //create the vertices
  for(int j=0;j<(verticalsegments+2);j++)
  {
    for(int i=0;i<pointsoncircle;i++)
    {
      CVector3<Real> vNext=pShape.eval(alpha);
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
    CTriFace face(verts);
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

	    CTriFace face1(verts);
	    vFaces.push_back(face1);
	    verts[0]=index+(j+1)%pointsoncircle;
	    verts[1]=index+pointsoncircle+j;
	    verts[2]=index+pointsoncircle+(j+1)%pointsoncircle;
	    CTriFace face2(verts);
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
	CTriFace face(verts);
	vFaces.push_back(face);
  }

  model.CreateFrom(vVertices,vFaces);

  return model;

}


}