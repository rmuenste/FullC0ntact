
#include <limits>
#include <matrix3x3.h>
#include <iostream>
#include <distancesegseg.h>
#include <distancepointseg.h>
#include <distancesegrec.h>
#include <distancepointrec.h>
#include <algorithm>
#include <obb3.h>
#include "distanceobb3obb3.h"
#include <intersectortools.h>

namespace i3d {

template <typename T>
CDistanceOBB3OBB3<T>::CDistanceOBB3OBB3(void)
{
}
template <typename T>
CDistanceOBB3OBB3<T>::~CDistanceOBB3OBB3(void)
{
}

template <typename T>
CDistanceOBB3OBB3<T>::CDistanceOBB3OBB3(OBB3<T> &pBox0,OBB3<T> &pBox1)
{
	m_pBox0=&pBox0;
	m_pBox1=&pBox1;
}

template <typename T>
unsigned int CDistanceOBB3OBB3<T>::ClassifyVertex(const CVector3<T> &pVertex,int iwhich)
{

	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;

	//classify every vertex
		unsigned int iRegion=0;
		if(pVertex.x < -pBox->extents_[0])
		{
			iRegion |= 0x01;
		}
		if(pVertex.x >= pBox->extents_[0])
		{
			iRegion |= 0x02;
		}
		if(pVertex.y < -pBox->extents_[1])
		{
			iRegion |= 0x04;
		}
		if(pVertex.y >= pBox->extents_[1])
		{
			iRegion |= 0x08;
		}
		if(pVertex.z < -pBox->extents_[2])
		{
			iRegion |= 0x10;
		}
		if(pVertex.z >= pBox->extents_[2])
		{
			iRegion |= 0x20;
		}
		return iRegion;
}


template <typename T>
void CDistanceOBB3OBB3<T>::ClassifyVertices(unsigned int iRegions[8], CVector3<T> pVertices[8],int iwhich)
{

	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;

	//classify every vertex
	for(int i=0;i<8;i++)
	{
		unsigned int iRegion=0;
		if(pVertices[i].x < -pBox->extents_[0])
		{
			iRegion |= 0x01;
		}
		if(pVertices[i].x >= pBox->extents_[0])
		{
			iRegion |= 0x02;
		}
		if(pVertices[i].y < -pBox->extents_[1])
		{
			iRegion |= 0x04;
		}
		if(pVertices[i].y >= pBox->extents_[1])
		{
			iRegion |= 0x08;
		}
		if(pVertices[i].z < -pBox->extents_[2])
		{
			iRegion |= 0x10;
		}
		if(pVertices[i].z >= pBox->extents_[2])
		{
			iRegion |= 0x20;
		}
		iRegions[i]=iRegion;
	}

}

template <typename T>
void CDistanceOBB3OBB3<T>::ComputeHalfPlaneIntersec(unsigned int s, int bcount, std::vector< std::pair<T,unsigned int> > &vCk, const CVector3<T> &vDir, const CVector3<T> &vA)
{

	unsigned int m[6] = {1,2,4,8,16,32};

	//loop through all half planes and check
	for(unsigned int i=0;i<6;i++)
	{
		//check whether the edge crosses the i-th halfplane
		if((s & m[i])==m[i])
			vCk.push_back(std::pair<T,unsigned int>(T(0),i));
	}

	//loop through all the halfplane crossings
	for(int k=0;k<vCk.size();k++)
	{
		T dj;
		//calculate the j=1,2,3 associated with halfplane normals (1,0,0), (0,1,0), (0,0,1)
		unsigned int i = vCk[k].second;
		unsigned int j = (i >> 1) + 1;
		if((i % 2) == 0)
			//distance of the plane to the origin
			dj = m_pBox1->extents_[j-1];
		else
			//distance of the plane to the origin
			dj = -m_pBox1->extents_[j-1];
		T mu = -(vA.m_dCoords[j-1] + dj)/vDir.m_dCoords[j-1];
		vCk[k].first = -(vA.m_dCoords[j-1] + dj)/vDir.m_dCoords[j-1];
	}
}//ComputeHalfPlaneIntersec

template <typename T>
void CDistanceOBB3OBB3<T>::EdgeSplit(const CVector3<T> &vA, const CVector3<T> &vB, unsigned int ca, unsigned int cb, std::vector<unsigned int> &vRegions)
{
	unsigned int s = ca ^ cb;
	int k = GetBitcount(s);
	unsigned int m[6] = {1,2,4,8,16,32};
	//std::cout<<"Traversed Regions by edge: "<<k+1<<std::endl;

	if(k==0)
	{
		vRegions.push_back(ca);
	}
	else if(k==1)
	{
		vRegions.push_back(ca);
		vRegions.push_back(cb);
	}
	else
	{
		//split the edge, compute the regions
		std::vector< std::pair<T,unsigned int> > vCk;
		CVector3<T> vDir = vB - vA;
		vDir.Normalize();
		ComputeHalfPlaneIntersec(s,k,vCk,vDir,vA);
		CmpPairs compare;
		std::sort(vCk.begin(),vCk.end(),compare);

		vRegions.push_back(ca);
		for(int j=0;j<k;j++)
		{
			unsigned int c_k_0 = vRegions[j];
			unsigned int mjk = m[vCk[j].second];
			unsigned int c_k=vRegions[j] ^ m[vCk[j].second];
			vRegions.push_back(c_k);
		}
	}

}//end function

template <typename T>
T CDistanceOBB3OBB3<T>::ComputeDistance()
{
 return (sqrt(ComputeDistanceSqr()));
}

template <typename T>
T CDistanceOBB3OBB3<T>::ComputeDistanceSqr()
{
	unsigned int iRegions[8];
	int iEdges[12][2]={{0,1}, {1,2}, {2,3}, {3,0}, {0,4}, {1,5}, {2,6}, {3,7}, {4,5}, {5,6}, {6,7}, {7,4}};
	int iConnect[8][3]={{1,3,4}, {0,2,5}, {1,3,6}, {0,2,7}, {0,5,7}, {1,4,6}, {2,5,7}, {3,6,4}};
	T minDistSqr = std::numeric_limits<T>::max();
	T dist;
	m_pBox1->computeVertices(m_pVertices1);
	m_pBox0->computeVertices(m_pVertices0);

	//
	int result=0;

	//set up a transformation for box0
	CMatrix3x3<T> mrotMat;
	mrotMat = CMatrix3x3<T>(m_pBox0->uvw_[0],m_pBox0->uvw_[1],m_pBox0->uvw_[2]);
	CMatrix3x3<T> mrotMatT = mrotMat.GetTransposedMatrix();

	//transform box1 to be centered at the origin with
	//the box axes parallel to the coordinate axes
	CMatrix3x3<T> mrotMat1;
	mrotMat1 = CMatrix3x3<T>(m_pBox1->uvw_[0],m_pBox1->uvw_[1],m_pBox1->uvw_[2]);
	CMatrix3x3<T> mrotMat1T = mrotMat1.GetTransposedMatrix();

	Transform<T> modelWorld;
	modelWorld.m_pMatrix = mrotMat1;
	modelWorld.m_pOrigin = m_pBox1->center_;

	//transform the vertices of box0 into the coordinate system of box1
	for(int k=0;k<8;k++)
	{
		CVector3<T> vecT = m_pVertices0[k];
		vecT = vecT - m_pBox1->center_;
		m_pVertices0[k] = mrotMat1T * vecT;
	}

	//begin first phase of algorithm
  //test the edges of box0
	for(int k=0;k<12;k++)
	{
		CVector3<T> vA = m_pVertices0[iEdges[k][0]];
		CVector3<T> vB = m_pVertices0[iEdges[k][1]];

		//
		T distEdgeB0;
		//std::cout<<"Edge: "<<k<<std::endl;
	//classify the vertices with respect to box1
		unsigned int iRegionA=ClassifyVertex(vA,1);
		unsigned int iRegionB=ClassifyVertex(vB,1);

		//if the enpoints are in different regions, we have to do an edge-splitting
		//take care about edge splitting
		std::vector<unsigned int> vRegions;
		EdgeSplit(vA,vB,iRegionA,iRegionB,vRegions);
		
		//for every region the edge passes
		for(int i=0;i< vRegions.size();i++)
		{
			unsigned int iCurrentRegion=vRegions[i];
			unsigned int iCode = GetRegionType(iCurrentRegion);
			if(iCode==VERTEX)
			{
				//std::cout<<"Found vertex region: "<<std::endl;

				//get the B1 vertex
				CVector3<T> vVertexB = GetRegionVertex(iCurrentRegion,1);
				Segment3<T>seg(vA,vB);
				//calculate point seg distance
				CDistancePointSeg<T> distPointSeg(vVertexB,seg);
				distEdgeB0=distPointSeg.ComputeDistanceSqr();
				//std::cout<<"DistancePointSeg: "<<distEdgeB0<<std::endl;
				if(distEdgeB0 < minDistSqr)
				{
					minDistSqr = distEdgeB0;
					result = 1;
					//m_vClosestPoint0 is the point on the segment of Box0
					m_vClosestPoint0 = distPointSeg.m_vClosestPoint1;
					//m_vClosestPoint1 is the point of the vertex region of Box1
					m_vClosestPoint1 = distPointSeg.m_vClosestPoint0;
				}//end if
			}
			else if(iCode==EDGE)
			{
				//std::cout<<"Found edge region: "<<std::endl;
				//get the B1_0centered edge
				Segment3<T>seg0(vA,vB);
				Segment3<T>seg1 = GetRegionEdge(iCurrentRegion,1);
				//calculate seg seg distance
				CDistanceSegSeg<T> distSegSeg(seg0,seg1);
				distEdgeB0=distSegSeg.ComputeDistanceSqr();
				//std::cout<<"DistanceSegSeg: "<<distEdgeB0 <<std::endl;
				if(distEdgeB0 < minDistSqr)
				{
					minDistSqr = distEdgeB0;
					result = 2;
					//m_vClosestPoint0 is the point on the segment of Box0
					m_vClosestPoint0 = distSegSeg.m_vClosestPoint0;
					//m_vClosestPoint1 is the point on the edge region of Box1
					m_vClosestPoint1 = distSegSeg.m_vClosestPoint1;
				}//end if
			}
			else if(iCode==FACE)
			{
				//std::cout<<"Found face region: "<<std::endl;
				//get the B1_0centered face
				Rectangle3<T> rec = GetRegionFace(iCurrentRegion,1);
				//calculate face seg distance
				Segment3<T>seg(vA,vB);
				CDistanceSegRec<T> distSegRec(seg,rec);
				distEdgeB0=distSegRec.ComputeDistanceSqr();
				//std::cout<<"DistanceSegRec: "<<distEdgeB0<<std::endl;
				if(distEdgeB0 < minDistSqr)
				{
					minDistSqr = distEdgeB0;
					result = 3;
          std::cout<<iCurrentRegion<<std::endl;
					//m_vClosestPoint0 is the point on the segment of Box0
					m_vClosestPoint0 = distSegRec.m_vClosestPoint0;
					//m_vClosestPoint1 is the point on the face(rectangle) of Box1
					m_vClosestPoint1 = distSegRec.m_vClosestPoint1;

          //update the configuration
          m_ocConf.m_iConf            = result;
          m_ocConf.m_vNormal          = GetFaceNormal(iCurrentRegion,1);
          m_ocConf.m_iFeature[0]      = EDGE;
          m_ocConf.m_iFeature[1]      = FACE;
          m_ocConf.m_iFeatureIndex[0] = k;
          m_ocConf.m_iFeatureIndex[1] = iCurrentRegion;
				}//end if
			}
		}//end for i
		//std::cout<<"------------------------------------------------------"<<std::endl;
	}//end for k

	//-------------------------------
	//begin second phase of algorithm

	//transform box0 to be centered at the origin with
	//the box axes parallel to the coordinate axes

	//transform the vertices of box1 to the same coordinate system
	for(int k=0;k<8;k++)
	{
		CVector3<T> vecT = m_pVertices1[k];
		vecT = vecT - m_pBox0->center_;
		m_pVertices1[k] = mrotMatT * vecT;
	}

	//classify the vertices with respect to box0
	ClassifyVertices(iRegions,m_pVertices1,0);

	//for each vertex test if it is in a face region
	for(int j=0;j<8;j++)
	{
		if(GetRegionType(iRegions[j])==FACE)
		{
			//get the B0_0centered face
			Rectangle3<T> rec = GetRegionFace(iRegions[j],0);
			CDistancePointRec<T> distPR(m_pVertices1[j],rec);
			dist = distPR.ComputeDistanceSqr();
			if(dist < minDistSqr)
			{
				minDistSqr = dist;
				result = 4;
				//closest point on the face(rectangle) of box0
				m_vClosestPoint0=distPR.m_vClosestPoint1;
				//closest point of box1 is the vertex
				m_vClosestPoint1=m_pVertices1[j];
				//reset the transformation
				modelWorld.m_pMatrix = mrotMat;
				modelWorld.m_pOrigin = m_pBox0->center_;

        //update the configuration
        m_ocConf.m_iConf       = result;
        m_ocConf.m_vNormal     = GetFaceNormal(iRegions[j],0);
        m_ocConf.m_iFeature[0] = FACE;
        m_ocConf.m_iFeature[1] = VERTEX;
        m_ocConf.m_iFeatureIndex[0] = iRegions[j];
        m_ocConf.m_iFeatureIndex[1] = j;
			}
		}
	}
  std::vector<CVector3<T> > vContacts;
	if(result == 4)
  {
		std::cout<<"Distance VERTEX-FACE: "<<std::endl;
//     CVector3<T> vertices1[8];
//     int index = m_ocConf.m_iFeatureIndex[1];
//     CVector3<T> vVertex1 = vertices1[index];
// 
//     m_pBox1->ComputeVertices(vertices1);
//     CVector3<T> vSegments[3]={vertices1[iConnect[index][0]]-vVertex1,vertices1[iConnect[index][1]]-vVertex1,vertices1[iConnect[index][2]]-vVertex1};
//     CVector3<T> vDir[3]={vSegments[0],vSegments[1],vSegments[2]};
//     vDir[0].Normalize();
//     vDir[1].Normalize();
//     vDir[2].Normalize();
// 
//     T dot[3] = {m_ocConf.m_vNormal*vDir[0],m_ocConf.m_vNormal*vDir[1],m_ocConf.m_vNormal*vDir[2]};
//     if(fabs(dot[0]) < E3)
//     {
//       if(fabs(dot[1]) < E3)
//       {
//         //face-face
//         CVector3<T> rec0[4];//GetFace(id,rec);
//         CVector3<T> rec1[4];//GetFace(id,rec);
// 
//         GetFace(m_ocConf.m_iFeatureIndex[0],0,rec0);
// 
//         int nContacts;
//         for(int i=0;i<4;i++)
//         {
//           rec0[i]+=minDistSqr*m_ocConf.m_vNormal;
//         }
//         CIntersectorTools<T>::RectangleRectanglePlanar(rec0,rec1,nContacts,vContacts);
//       }
//       else if(fabs(dot[2]) < E3)
//       {
//         //face-face
//         CVector3<T> rec0[4];//={vVertex1,vVertex1+vSegments[0]};
//         CVector3<T> rec1[4];//GetFace(id,rec);
//         GetFace(m_ocConf.m_iFeatureIndex[0],0,rec0);
//         int nContacts;
//         for(int i=0;i<4;i++)
//         {
//           rec0[i]+=minDistSqr*m_ocConf.m_vNormal;
//         }
//         CIntersectorTools<T>::RectangleRectanglePlanar(rec0,rec1,nContacts,vContacts);
//       }
//       else
//       {
//         //face-edge
//         CVector3<T> seg[2]={vVertex1,vVertex1+vSegments[0]};
//         CVector3<T> rec[4];
//         GetFace(m_ocConf.m_iFeatureIndex[0],0,rec);
//         int nContacts;
//         for(int i=0;i<4;i++)
//         {
//           rec[i]+=minDistSqr*m_ocConf.m_vNormal;
//         }
//         CIntersectorTools<T>::SegmentRectanglePlanar(seg,rec,nContacts,vContacts);
//       }
//     }
//     else if(fabs(dot[1]) < E3)
//     {
//       if(fabs(dot[2]) < E3)
//       {
//         //face-face
//         CVector3<T> rec0[4];//GetFace(id,rec);
//         CVector3<T> rec1[4];//GetFace(id,rec);
//         GetFace(m_ocConf.m_iFeatureIndex[0],0,rec0);
//         int nContacts;
//         for(int i=0;i<4;i++)
//         {
//           rec0[i]+=minDistSqr*m_ocConf.m_vNormal;
//         }
//         CIntersectorTools<T>::RectangleRectanglePlanar(rec0,rec1,nContacts,vContacts);
//       }
//       else
//       {
//         //face-edge
//         CVector3<T> seg[2]={vVertex1,vVertex1+vSegments[1]};
//         CVector3<T> rec[4];//GetFace(id,rec);
//         GetFace(m_ocConf.m_iFeatureIndex[0],0,rec);
//         int nContacts;
//         for(int i=0;i<4;i++)
//         {
//           rec[i]+=minDistSqr*m_ocConf.m_vNormal;
//         }
//         CIntersectorTools<T>::SegmentRectanglePlanar(seg,rec,nContacts,vContacts);
//       }
//     }
//     else if(fabs(dot[2]) < E3)
//     {
//       //face-edge
//       CVector3<T> seg[2]={vVertex1,vVertex1+vSegments[2]};
//       CVector3<T> rec[4];//GetFace(id,rec);
//       GetFace(m_ocConf.m_iFeatureIndex[0],0,rec);
//       int nContacts;
//       for(int i=0;i<4;i++)
//       {
//         rec[i]+=minDistSqr*m_ocConf.m_vNormal;
//       }
//       CIntersectorTools<T>::SegmentRectanglePlanar(seg,rec,nContacts,vContacts);
//     }
//     else
//     {
//       //vertex-face
// 	    CVector3<T> vCP = m_vClosestPoint0;
// 	    vCP = modelWorld.m_pMatrix * vCP;
// 	    vCP += modelWorld.m_pOrigin;
// 	    m_vClosestPoint0 = vCP;
// 
// 	    vCP = m_vClosestPoint1;
// 	    vCP = modelWorld.m_pMatrix * vCP;
// 	    vCP += modelWorld.m_pOrigin;
// 	    m_vClosestPoint1 = vCP;
//     }

    //m_ocConf.
    //test dot(edge0,faceNormal)
    //test dot(edge1,faceNormal)
    //test dot(edge2,faceNormal)
    //how may smaller constant
    //if(1) edge-face
      //transform face in normal direction
      //clip
    //elseif(2) face-face
      //transform face in normal direction
      //clip

  }
	else if(result == 1)
  {
		std::cout<<"Distance EDGE-VERTEX: "<<std::endl;

	  //transform the closest points
	  //to world coordinates
	  CVector3<T> vCP = m_vClosestPoint0;
	  vCP = modelWorld.m_pMatrix * vCP;
	  vCP += modelWorld.m_pOrigin;
	  m_vClosestPoint0 = vCP;

	  vCP = m_vClosestPoint1;
	  vCP = modelWorld.m_pMatrix * vCP;
	  vCP += modelWorld.m_pOrigin;
	  m_vClosestPoint1 = vCP;
  }
	else if(result == 2)
  {
		std::cout<<"Distance EDGE-EDGE: "<<std::endl;

	  //transform the closest points
	  //to world coordinates
	  CVector3<T> vCP = m_vClosestPoint0;
	  vCP = modelWorld.m_pMatrix * vCP;
	  vCP += modelWorld.m_pOrigin;
	  m_vClosestPoint0 = vCP;

	  vCP = m_vClosestPoint1;
	  vCP = modelWorld.m_pMatrix * vCP;
	  vCP += modelWorld.m_pOrigin;
	  m_vClosestPoint1 = vCP;
  }
	else if(result == 3)
  {
		std::cout<<"Distance EDGE-FACE: "<<std::endl;
    //
  }

	return minDistSqr;
}

template <typename T>
CVector3<T> CDistanceOBB3OBB3<T>::GetRegionVertex(unsigned int iRegion, int iwhich)
{
	CVector3<T> vVertex;
	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;
	for(unsigned int i=1;i<=3;i++)
	{
		unsigned int m = 1;
		//left shift
		m <<= 2*(i-1);
		if((iRegion & m) != 0)
			vVertex.m_dCoords[i-1]=-pBox->extents_[i-1];
		else
			vVertex.m_dCoords[i-1]=pBox->extents_[i-1];
	}
	return vVertex;
}

template <typename T>
Segment3<T> CDistanceOBB3OBB3<T>::GetRegionEdge(unsigned int iRegion, int iwhich)
{
	//the vertex region of the first vertex of the edge
	unsigned int c1;
	//the vertex region of the 2nd vertex of the edge
	unsigned int c2;
	int j=0;

	unsigned int m1 = 1;

	CVector3<T> vDir;
	//identify the double zero pattern 00****,**00**,****00
	for(unsigned int i=1;i<=3;i++)
	{
		unsigned int m3 = 3;
		m3 <<= 2*(i-1);
		if((iRegion & m3) == 0)
		{
			vDir.m_dCoords[i-1] = 1;
			j=i;
		}
		else
			vDir.m_dCoords[i-1] = 0;
	}

	m1 = 1;
	m1 <<=2*(j-1);
	c1 = iRegion ^ m1;
	m1 = 1;
	m1 <<=2*(j-1)+1;
	c2 = iRegion ^ m1;

	//get the vertex corresponding to code c1 of Box iwhich
	CVector3<T> vA = GetRegionVertex(c1,iwhich);
	//get the vertex corresponding to code c2 of Box iwhich
	CVector3<T> vB = GetRegionVertex(c2,iwhich);
	Segment3<T> seg(vA,vB);
	return seg;
}

template <typename T>
Rectangle3<T> CDistanceOBB3OBB3<T>::GetRegionFace(unsigned int iRegion, int iwhich)
{
	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;
	Rectangle3<T> rec;
	CVector3<T> vAxes[3] = {CVector3<T>(1,0,0),CVector3<T>(0,1,0),CVector3<T>(0,0,1)};
	CVector3<T> extAxis0 = pBox->extents_[0] * vAxes[0];
	CVector3<T> extAxis1 = pBox->extents_[1] * vAxes[1];
	CVector3<T> extAxis2 = pBox->extents_[2] * vAxes[2];

	switch(iRegion)
	{
		case 1:
			rec.center_= - extAxis0;
			rec.uv_[0]=vAxes[1];
			rec.uv_[1]=vAxes[2];
			rec.extents_[0]=pBox->extents_[1];
			rec.extents_[1]=pBox->extents_[2];
			break;
		case 2:
			rec.center_= extAxis0;
			rec.uv_[0]=vAxes[1];
			rec.uv_[1]=vAxes[2];
			rec.extents_[0]=pBox->extents_[1];
			rec.extents_[1]=pBox->extents_[2];
			break;
		case 4:
			rec.center_= - extAxis1;
			rec.uv_[0]=vAxes[0];
			rec.uv_[1]=vAxes[2];
			rec.extents_[0]=pBox->extents_[0];
			rec.extents_[1]=pBox->extents_[2];
			break;
		case 8:
			rec.center_= extAxis1;
			rec.uv_[0]=vAxes[0];
			rec.uv_[1]=vAxes[2];
			rec.extents_[0]=pBox->extents_[0];
			rec.extents_[1]=pBox->extents_[2];
			break;
		case 16:
			rec.center_= - extAxis2;
			rec.uv_[0]=vAxes[0];
			rec.uv_[1]=vAxes[1];
			rec.extents_[0]=pBox->extents_[0];
			rec.extents_[1]=pBox->extents_[1];
			break;
		case 32:
			rec.center_= extAxis2;
			rec.uv_[0]=vAxes[0];
			rec.uv_[1]=vAxes[1];
			rec.extents_[0]=pBox->extents_[0];
			rec.extents_[1]=pBox->extents_[1];
			break;
	}
	return rec;
}

template <typename T>
void CDistanceOBB3OBB3<T>::GetFace(unsigned int iRegion, int iwhich, CVector3<T> vVerts[4])
{
	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;

  CVector3<T> vVertices[8];
  pBox->computeVertices(vVertices);
	switch(iRegion)
	{
		case 1:
      vVerts[0]=vVertices[0];
      vVerts[1]=vVertices[3];
      vVerts[2]=vVertices[7];
      vVerts[3]=vVertices[4];
			break;
		case 2:
      vVerts[0]=vVertices[1];
      vVerts[1]=vVertices[5];
      vVerts[2]=vVertices[6];
      vVerts[3]=vVertices[2];
			break;
		case 4:
      vVerts[0]=vVertices[3];
      vVerts[1]=vVertices[2];
      vVerts[2]=vVertices[6];
      vVerts[3]=vVertices[7];
			break;
		case 8:
      vVerts[0]=vVertices[0];
      vVerts[1]=vVertices[1];
      vVerts[2]=vVertices[5];
      vVerts[3]=vVertices[4];
			break;
		case 16:
      vVerts[0]=vVertices[0];
      vVerts[1]=vVertices[1];
      vVerts[2]=vVertices[2];
      vVerts[3]=vVertices[3];
			break;
		case 32:
      vVerts[0]=vVertices[4];
      vVerts[1]=vVertices[5];
      vVerts[2]=vVertices[6];
      vVerts[3]=vVertices[7];
			break;
	}
}


template <typename T>
CVector3<T> CDistanceOBB3OBB3<T>::GetFaceNormal(unsigned int iRegion, int iwhich)
{
	OBB3<T> *pBox = (iwhich==0) ? m_pBox0 : m_pBox1;
  CVector3<T> vNormal;
	switch(iRegion)
	{
		case 1:
      vNormal =  -pBox->uvw_[0];
			break;
		case 2:
      vNormal =  pBox->uvw_[0];
			break;
		case 4:
      vNormal =  -pBox->uvw_[1];
			break;
		case 8:
      vNormal =  pBox->uvw_[1];
			break;
		case 16:
      vNormal =  -pBox->uvw_[2];
			break;
		case 32:
      vNormal =  pBox->uvw_[2];
			break;
	}
  return vNormal;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceOBB3OBB3<Real>;

//template class CDistanceOBB3OBB3<double>;
//----------------------------------------------------------------------------

}