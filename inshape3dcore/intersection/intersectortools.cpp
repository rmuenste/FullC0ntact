#include "intersectortools.h"
#include <mymath.h>
#include <iostream>
#include <transform.h>
#include <intersectorspheresegment.h>
#include <matrix3x3.h>
#include <vector2.h>
#include <intersectorcirclecircle.h>

namespace i3d {

inline int getEqualBits(unsigned int i0,unsigned int i1)
{
  unsigned int code = i0 & i1;
  int count = 0;
  while(code)
  {
    //AND with 0x01 and add up
    count += code & 0x1u;
    //shift the bit away
    code >>= 1;
  }
  return count;
}

template <class T>
CIntersectorTools<T>::CIntersectorTools(void)
{
}

template <class T>
CIntersectorTools<T>::~CIntersectorTools(void)
{
}

template <class T>
bool CIntersectorTools<T>::Find(const Vector3<T> &testAxis, const OBB3<T> &box0, const OBB3<T> &box1, const Vector3<T> relVelocity,
                                T &tmax, T &tfirst, T &tlast,int &side, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1)
{

  CProjCfg<T> cfgStart0;
  CProjCfg<T> cfgStart1;

  //compute the relative orientation for the axis
  getProjCfg(testAxis,box0,cfgStart0);
  getProjCfg(testAxis,box1,cfgStart1);

  //compute the first and last intersection with regard to this axis and
  //update the final configuration if neccessary
  return Find(testAxis,relVelocity,tmax,tfirst,tlast,side,cfgStart0,cfgStart1,cfgFinal0,cfgFinal1);
}

template <class T>
bool CIntersectorTools<T>::Find(const Vector3<T> &testAxis, const Vector3<T> relVelocity, T &tmax, T &tfirst, T &tlast,int &side,
                                CProjCfg<T> &cfgCurr0, CProjCfg<T> &cfgCurr1, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1)
{

  //calculate the first and last time of intersection relative to testAxis
  //this routine is called in the context of an intersection search for all possible separating axes
  //we try to find the biggest tfirst for all axes, so if we find a bigger tfirst, we update this value

  //time of impact
  T toi;

  //calculate the normal velocity
  T velocity = testAxis * relVelocity;
  if((cfgCurr1.m_dMax - cfgCurr0.m_dMin) < -CMath<T>::TOLERANCEZERO)//(cfgCurr1.m_dMax < cfgCurr0.m_dMin)
  {
    //object1 is moving away from object0
    if(velocity <=T(0))
      return false;

    //calculate time of impact
    toi = (cfgCurr0.m_dMin - cfgCurr1.m_dMax)/velocity;

    T separation = (cfgCurr0.m_dMin - cfgCurr1.m_dMax);

    if(separation < cfgCurr0.m_dMinSeparation)
    {
      cfgFinal0.m_dMinSeparation = separation;
      cfgFinal0.m_iMinSepFeature = cfgCurr0.m_iFeature;
      cfgFinal0.m_vMinSepNormal  = testAxis;
      cfgFinal0.m_iMinSepOrientation = CProjCfg<T>::RIGHT;
      memcpy(&cfgFinal0.m_iMinSepIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));

      cfgFinal1.m_dMinSeparation = separation;
      cfgFinal1.m_iMinSepFeature = cfgCurr1.m_iFeature;
      cfgFinal1.m_vMinSepNormal  = testAxis;
      cfgFinal1.m_iMinSepOrientation = CProjCfg<T>::LEFT;
      memcpy(&cfgFinal1.m_iMinSepIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
    }

    //check if the first time of impact for this axis
    //is bigger than the current one, if so then update
    if(toi > tfirst)
    {
      tfirst = toi;
      side   = CProjCfg<T>::LEFT;
      cfgFinal0 = cfgCurr0;
      cfgFinal1 = cfgCurr1;
      
    }

    //check if the first time of impact is outside of
    //the query interval
    if(tfirst > tmax)
      return false;

    //compute the last time of impact with respect to the test axis
    toi = (cfgCurr0.m_dMax - cfgCurr1.m_dMin)/velocity;

    //if we found a smaller last time of impact
    if(toi < tlast)
      tlast = toi;

    if(tfirst > tlast)
      return false;

  }//end if(cfgCurr1.m_dMax < cfgCurr0.m_dMin)
  else if((cfgCurr0.m_dMax - cfgCurr1.m_dMin) < -CMath<T>::TOLERANCEZERO) //(cfgCurr0.m_dMax < cfgCurr1.m_dMin)
  {
    //obj1 is on the right of axis
    //so if the velocity is >=0 there won't be an intersection
    if(velocity >= T(0))
      return false;

    //compute the first time of impact
    toi = (cfgCurr0.m_dMax - cfgCurr1.m_dMin)/velocity;

    T separation = (cfgCurr0.m_dMax - cfgCurr1.m_dMin);
    if(separation < cfgCurr0.m_dMinSeparation)
    {
      cfgFinal0.m_dMinSeparation = separation;
      cfgFinal0.m_iMinSepFeature = cfgCurr0.m_iFeature;
      cfgFinal0.m_vMinSepNormal  = testAxis;
      cfgFinal0.m_iMinSepOrientation = CProjCfg<T>::RIGHT;
      memcpy(&cfgFinal0.m_iMinSepIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));

      cfgFinal1.m_dMinSeparation = separation;
      cfgFinal1.m_iMinSepFeature = cfgCurr1.m_iFeature;
      cfgFinal1.m_vMinSepNormal  = testAxis;
      cfgFinal1.m_iMinSepOrientation = CProjCfg<T>::LEFT;
      memcpy(&cfgFinal1.m_iMinSepIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
    }


    //check if the first time of impact for this axis
    //is bigger than the current one, if so then update
    if(tfirst < toi)
    {
      tfirst = toi;
      side   = CProjCfg<T>::RIGHT;
      cfgFinal0 = cfgCurr0;
      cfgFinal1 = cfgCurr1;
    }

    //check if the first time of impact is outside of
    //the query interval
    if(tfirst > tmax)
      return false;


    //compute the last time of impact with respect to the test axis
    toi = (cfgCurr0.m_dMin - cfgCurr1.m_dMax)/velocity;
    
    //if we found a smaller last time of impact
    if(toi < tlast)
      tlast = toi;

    if(tfirst > tlast)
      return false;

  }//end else if(cfgCurr0.m_dMax < cfgCurr1.m_dMin)
  else
  {
    //the objects are initially overlapping on this axis
    //if(velocity > T(0)) //CMath<T>::EPSILON5)
    if(velocity > CMath<T>::EPSILON5)
    {
      //compute the last time of impact with respect to the test axis
      toi = (cfgCurr0.m_dMax - cfgCurr1.m_dMin)/velocity;

      //if we found a smaller last time of impact
      if(toi < tlast)
        tlast = toi;

      T minOverlap;
      //compute the actual overlap
      if(cfgCurr1.m_dMax > cfgCurr0.m_dMax)
      {
        if((minOverlap=(cfgCurr0.m_dMax - cfgCurr1.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 2;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 1;
        }
      }
      else
      {
        if((minOverlap=(cfgCurr1.m_dMax - cfgCurr0.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 1;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 2;
        }
      }

      //check against tfirst
      if(tfirst > tlast)
        return false;
    }
    //else if(velocity < T(0))//-CMath<T>::EPSILON5)
    else if(velocity < -CMath<T>::EPSILON5)
    {
      toi = (cfgCurr0.m_dMin - cfgCurr1.m_dMax)/velocity;

      //if we found a smaller last time of impact
      if(toi < tlast)
        tlast = toi;

      T minOverlap;
      //compute the actual overlap
      if(cfgCurr1.m_dMax > cfgCurr0.m_dMax)
      {
        if((minOverlap=(cfgCurr0.m_dMax - cfgCurr1.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 2;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 1;
        }
      }
      else
      {
        if((minOverlap=(cfgCurr1.m_dMax - cfgCurr0.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 1;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 2;
        }
      }
      //check against tfirst
      if(tfirst > tlast)
        return false;
    }
    else
    {
      //here we basically detected a resting contact
      toi = std::numeric_limits<T>::max();

      //if we found a smaller last time of impact
      if(toi < tlast)
        tlast = toi;

      T minOverlap;
      //compute the actual overlap
      if(cfgCurr1.m_dMax > cfgCurr0.m_dMax)
      {
        if((minOverlap=(cfgCurr0.m_dMax - cfgCurr1.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 2;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 1;
        }
      }
      else
      {
        if((minOverlap=(cfgCurr1.m_dMax - cfgCurr0.m_dMin)) < cfgFinal0.m_dMinOverlap)
        {
          cfgFinal0.m_dMinOverlap = minOverlap;
          cfgFinal0.m_vMinNormal  = testAxis;
          cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
          memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
          cfgFinal0.m_iMinOrientation = 1;
          cfgFinal1.m_dMinOverlap = minOverlap;
          cfgFinal1.m_vMinNormal  = testAxis;
          cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
          memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
          cfgFinal1.m_iMinOrientation = 2;
        }
      }
      //check against tfirst
      if(tfirst > tlast)
        return false;

    }
  }

  return true;
}

template <class T>
void CIntersectorTools<T>::getProjCfg(const Vector3<T> &testAxis, const OBB3<T> &box, CProjCfg<T> &cfg)
{
  //Checks the orientation of the box relative to the axis
  //if the axis is perpendicular to
  //two axes of the other box, then the feature for this box is face 

  //if the axis is perpendicular to
  //just axis of the other box, then the feature for this box is edge

  //if the axis is not perpendicular to any of the other boxes axes,
  //then the feature for this box is vertex

  //compute the dot products of the axis with the box normals
  T dotAxes[3] ={testAxis*box.uvw_[0], testAxis*box.uvw_[1],testAxis*box.uvw_[2]};

  //compute the abs of the projection
  T dotAxesAbs[3] = {fabs(dotAxes[0]),fabs(dotAxes[1]),fabs(dotAxes[2])};

  T maximumProjection = T(0);

  cfg.m_vNormal = testAxis;

  if(dotAxesAbs[0] < E1)
  {
    if(dotAxesAbs[1] < E1)
    {
      //found a face feature for this axis
      cfg.SetFeature(CProjCfg<T>::BOX_FACE);

      maximumProjection = dotAxesAbs[2] * box.extents_[2];

      //determine which faces are the max-min
      if(dotAxes[2] > T(0))
      {
        //+ z-axis face is max
        cfg.SetMaxFace(4,5,6,7);

        //- z-axis face is min
        cfg.SetMinFace(0,1,2,3);
      }
      else
      {
        //- z-axis face is max
        cfg.SetMaxFace(0,1,2,3);

        //+ z-axis face is min
        cfg.SetMinFace(4,5,6,7);
      }

    }//end if(dotAxesAbs[1] < E1)

    else if(dotAxesAbs[2] < E1)
    {
      //found a face feature for this axis
      cfg.SetFeature(CProjCfg<T>::BOX_FACE);

      maximumProjection = dotAxesAbs[1] * box.extents_[1];

      //determine which faces are the max-min
      if(dotAxes[1] > T(0))
      {
        //+ y-axis face is max
        cfg.SetMaxFace(3,2,6,7);

        //- y-axis face is min
        cfg.SetMinFace(0,1,5,4);
      }
      else
      {
        //- y-axis face is max
        cfg.SetMaxFace(0,1,5,4);

        //+ y-axis face is min
        cfg.SetMinFace(3,2,6,7);
      }

    }//end if(dotAxesAbs[1] < E1)
    else
    {
      //found an edge feature
      cfg.SetFeature(CProjCfg<T>::BOX_EDGE);

      maximumProjection = dotAxesAbs[1] * box.extents_[1] + dotAxesAbs[2] * box.extents_[2];

      //check the relative orientation of this axis to normal[1]
      if(dotAxes[1] > T(0))
      {
        if(dotAxes[2] > T(0))
        {
          //the edge in +normal[1] and +normal[2] direction is max
          cfg.SetMaxEdge(6,7);
          //the opposite edge is min
          cfg.SetMinEdge(0,1);
        }
        else
        {
          //the edge in +normal[1] and -normal[2] direction is max
          cfg.SetMaxEdge(2,3);
          //the opposite edge is min
          cfg.SetMinEdge(4,5);
        }
      }
      else
      {
        if(dotAxes[2] > T(0))
        {
          //the edge in -normal[1] and +normal[2] direction is max
          cfg.SetMaxEdge(4,5);
          //the opposite edge is min
          cfg.SetMinEdge(2,3);
        }
        else
        {
          //the edge in -normal[1] and -normal[2] direction is max
          cfg.SetMaxEdge(0,1);
          //the opposite edge is min
          cfg.SetMinEdge(6,7);
        }
      }

    }//end else

  }//end if (dotAxesAbs[0] < E1)
  else if(dotAxesAbs[1] < E1)
  {
    if(dotAxesAbs[2] < E1)
    {
      //found a face feature for this axis
      cfg.SetFeature(CProjCfg<T>::BOX_FACE);

      maximumProjection = dotAxesAbs[0] * box.extents_[0];

      //determine which faces are the max-min
      if(dotAxes[0] > T(0))
      {
        //+ x-axis face is max
        cfg.SetMaxFace(1,5,6,2);

        //- x-axis face is min
        cfg.SetMinFace(0,3,7,4);
      }
      else
      {
        //- x-axis face is max
        cfg.SetMaxFace(0,3,7,4);

        //+ x-axis face is min
        cfg.SetMinFace(1,5,6,2);
      }

    }//end if(dotAxesAbs[2] < E1)
    else
    {
      //found an edge feature
      cfg.SetFeature(CProjCfg<T>::BOX_EDGE);

      maximumProjection = dotAxesAbs[0] * box.extents_[0] + dotAxesAbs[2] * box.extents_[2];

      //check the relative orientation of this axis to normal[0]
      if(dotAxes[0] > T(0))
      {
        if(dotAxes[2] > T(0))
        {
          //the edge in +normal[0] and +normal[2] direction is max
          cfg.SetMaxEdge(5,6);
          //the opposite edge is min
          cfg.SetMinEdge(0,3);
        }
        else
        {
          //the edge in +normal[0] and -normal[2] direction is max
          cfg.SetMaxEdge(1,2);
          //the opposite edge is min
          cfg.SetMinEdge(4,7);
        }
      }
      else
      {
        if(dotAxes[2] > T(0))
        {
          //the edge in -normal[0] and +normal[2] direction is max
          cfg.SetMaxEdge(4,7);
          //the opposite edge is min
          cfg.SetMinEdge(1,2);
        }
        else
        {
          //the edge in -normal[0] and -normal[2] direction is max
          cfg.SetMaxEdge(0,3);
          //the opposite edge is min
          cfg.SetMinEdge(5,6);
        }
      }

    }//end else

  }//end if (dotAxesAbs[0] < E1)
  else if(dotAxesAbs[2] < E1)
  {

    //found an edge feature
    cfg.SetFeature(CProjCfg<T>::BOX_EDGE);

    maximumProjection = dotAxesAbs[0] * box.extents_[0] + dotAxesAbs[1] * box.extents_[1];

    //check the relative orientation of this axis to normal[0]
    if(dotAxes[0] > T(0))
    {
      if(dotAxes[1] > T(0))
      {
        //the edge in +normal[0] and +normal[1] direction is max
        cfg.SetMaxEdge(2,6);
        //the opposite edge is min
        cfg.SetMinEdge(0,4);
      }
      else
      {
        //the edge in +normal[0] and -normal[1] direction is max
        cfg.SetMaxEdge(1,5);
        //the opposite edge is min
        cfg.SetMinEdge(3,7);
      }
    }
    else
    {
      if(dotAxes[1] > T(0))
      {
        //the edge in -normal[0] and +normal[1] direction is max
        cfg.SetMaxEdge(3,7);
        //the opposite edge is min
        cfg.SetMinEdge(1,5);
      }
      else
      {
        //the edge in -normal[0] and -normal[1] direction is max
        cfg.SetMaxEdge(0,4);
        //the opposite edge is min
        cfg.SetMinEdge(2,6);
      }
    }

  }//end if (dotAxesAbs[2] < E1)
  else
  {
    //found a vertex feature
    cfg.SetFeature(CProjCfg<T>::BOX_VERTEX);

    maximumProjection = dotAxesAbs[0] * box.extents_[0] + dotAxesAbs[1] * box.extents_[1] + dotAxesAbs[2] * box.extents_[2];
    
    //set the min vertex
    int index = (dotAxes[0] > T(0) ? 0 : 1) + (dotAxes[1] > T(0) ? 0 : 2) + (dotAxes[2] > T(0) ? 0 : 4);
    cfg.SetMinVertex(map(index));

    //set the min vertex
    cfg.SetMaxVertex(map(7-index));
  }

  //projection onto the line through the center
  T center = testAxis * box.center_;
  cfg.m_dMax = center + maximumProjection;
  cfg.m_dMin = center - maximumProjection;

}

template <class T>
void CIntersectorTools<T>::ComputeContactSet(const OBB3<T> &box0, const OBB3<T> &box1, int iside, const CProjCfg<T> &cfg0,
                           const CProjCfg<T> &cfg1, const Vector3<T> &vel0, const Vector3<T> &vel1,
                           T tfirst, int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  CPredictionTransform<T,OBB3<T> > transform;
  //get the boxes at time of impact
  OBB3<T> newBox0 = transform.PredictLinearMotion(box0,vel0*tfirst);
  OBB3<T> newBox1 = transform.PredictLinearMotion(box1,vel1*tfirst);

  Vector3<T> vertices0[8];
  Vector3<T> vertices1[8];

  newBox0.computeVertices(vertices0);
  newBox1.computeVertices(vertices1);

  //
  if(iside == CProjCfg<T>::LEFT)
  {
    //here box1 is on the left side of box0 with
    //respect to an axis
    //check the box0 closest feature
    if(cfg0.getFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices0[cfg0.m_iFeatureIndex[0]]);
    }
    else if(cfg1.getFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices1[cfg1.m_iFeatureIndex[4]]);
    }
    //the only remaining cases are now EE,EF,FF
    else if(cfg0.getFeature() == CProjCfg<T>::BOX_EDGE)
    {
      //check configuration of box1 for edge or face
      if(cfg1.getFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-edge
        Vector3<T> edge0[2];
        Vector3<T> edge1[2];
        edge0[0] = getPoint(cfg0.m_iFeatureIndex[0],box0);
        edge0[1] = getPoint(cfg0.m_iFeatureIndex[1],box0);
        edge1[0] = getPoint(cfg1.m_iFeatureIndex[4],box1);
        edge1[1] = getPoint(cfg1.m_iFeatureIndex[5],box1);

        //compute intersection segment-segment
        FindSegmentSegment(edge0,edge1,nContacts,vContacts);
      }
      else
      {
        //edge-face
        Vector3<T> edge0[2];
        Vector3<T> face1[4];
        edge0[0] = getPoint(cfg0.m_iFeatureIndex[0],box0);
        edge0[1] = getPoint(cfg0.m_iFeatureIndex[1],box0);
        face1[0] = getPoint(cfg1.m_iFeatureIndex[4],box1);
        face1[1] = getPoint(cfg1.m_iFeatureIndex[5],box1);
        face1[2] = getPoint(cfg1.m_iFeatureIndex[6],box1);
        face1[3] = getPoint(cfg1.m_iFeatureIndex[7],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge0,face1,nContacts,vContacts);
      }
    }
    else
    {
      //the box0 feature is a face
      //now check for face-edge or face-face
      if(cfg1.getFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-face
        Vector3<T> face0[4];
        Vector3<T> edge1[2];
        face0[0] = getPoint(cfg0.m_iFeatureIndex[0],box0);
        face0[1] = getPoint(cfg0.m_iFeatureIndex[1],box0);
        face0[2] = getPoint(cfg0.m_iFeatureIndex[2],box0);
        face0[3] = getPoint(cfg0.m_iFeatureIndex[3],box0);
        edge1[0] = getPoint(cfg1.m_iFeatureIndex[4],box1);
        edge1[1] = getPoint(cfg1.m_iFeatureIndex[5],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge1,face0,nContacts,vContacts);
      }
      else
      {
        //face-face
        Vector3<T> face0[4];
        Vector3<T> face1[4];
        face0[0] = getPoint(cfg0.m_iFeatureIndex[0],box0);
        face0[1] = getPoint(cfg0.m_iFeatureIndex[1],box0);
        face0[2] = getPoint(cfg0.m_iFeatureIndex[2],box0);
        face0[3] = getPoint(cfg0.m_iFeatureIndex[3],box0);
        face1[0] = getPoint(cfg1.m_iFeatureIndex[4],box1);
        face1[1] = getPoint(cfg1.m_iFeatureIndex[5],box1);
        face1[2] = getPoint(cfg1.m_iFeatureIndex[6],box1);
        face1[3] = getPoint(cfg1.m_iFeatureIndex[7],box1);

        //compute intersection coplanar face-face
        RectangleRectanglePlanar(face0,face1,nContacts,vContacts);
      }
    }

  }//end if(iside == CProjCfg<T>::LEFT)
  else
  {
    //box1 is on the right of box0
    if(cfg0.getFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices0[cfg0.m_iFeatureIndex[4]]);
    }
    else if(cfg1.getFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices1[cfg1.m_iFeatureIndex[0]]);
    }
    //the only remaining cases are now EE,EF,FF
    else if(cfg0.getFeature() == CProjCfg<T>::BOX_EDGE)
    {
      //check configuration of box1 for edge or face
      if(cfg1.getFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-edge
        //compute intersection segment-segment
        Vector3<T> edge0[2];
        Vector3<T> edge1[2];
        edge0[0] = getPoint(cfg0.m_iFeatureIndex[4],box0);
        edge0[1] = getPoint(cfg0.m_iFeatureIndex[5],box0);
        edge1[0] = getPoint(cfg1.m_iFeatureIndex[0],box1);
        edge1[1] = getPoint(cfg1.m_iFeatureIndex[1],box1);

        FindSegmentSegment(edge0,edge1,nContacts,vContacts);

      }
      else
      {
        //edge-face
        Vector3<T> edge0[2];
        Vector3<T> face1[4];
        edge0[0] = getPoint(cfg0.m_iFeatureIndex[4],box0);
        edge0[1] = getPoint(cfg0.m_iFeatureIndex[5],box0);
        face1[0] = getPoint(cfg1.m_iFeatureIndex[0],box1);
        face1[1] = getPoint(cfg1.m_iFeatureIndex[1],box1);
        face1[2] = getPoint(cfg1.m_iFeatureIndex[2],box1);
        face1[3] = getPoint(cfg1.m_iFeatureIndex[3],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge0,face1,nContacts,vContacts);
      }
    }
    else
    {
      //the box0 feature is a face
      //now check for face-edge or face-face
      if(cfg1.getFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-face
        Vector3<T> face0[4];
        Vector3<T> edge1[2];
        face0[0] = getPoint(cfg0.m_iFeatureIndex[4],box0);
        face0[1] = getPoint(cfg0.m_iFeatureIndex[5],box0);
        face0[2] = getPoint(cfg0.m_iFeatureIndex[6],box0);
        face0[3] = getPoint(cfg0.m_iFeatureIndex[7],box0);
        edge1[0] = getPoint(cfg1.m_iFeatureIndex[0],box1);
        edge1[1] = getPoint(cfg1.m_iFeatureIndex[1],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge1,face0,nContacts,vContacts);
      }
      else
      {
        //face-face
        Vector3<T> face0[4];
        Vector3<T> face1[4];
        face0[0] = getPoint(cfg0.m_iFeatureIndex[4],box0);
        face0[1] = getPoint(cfg0.m_iFeatureIndex[5],box0);
        face0[2] = getPoint(cfg0.m_iFeatureIndex[6],box0);
        face0[3] = getPoint(cfg0.m_iFeatureIndex[7],box0);
        face1[0] = getPoint(cfg1.m_iFeatureIndex[0],box1);
        face1[1] = getPoint(cfg1.m_iFeatureIndex[1],box1);
        face1[2] = getPoint(cfg1.m_iFeatureIndex[2],box1);
        face1[3] = getPoint(cfg1.m_iFeatureIndex[3],box1);

        //compute intersection coplanar face-face
        RectangleRectanglePlanar(face0,face1,nContacts,vContacts);
      }
    }
  }//end else
}

template <class T>
void CIntersectorTools<T>::ComputeContactSet(const Cylinder<T> &cylinder0, const Cylinder<T> &cylinder1,
                              CSimplexDescriptorGjk<T> &simplex,
                              const Transformation<T> &transform0, const Transformation<T> &transform1,
                              const Vector3<T> &closestPoint0, const Vector3<T> &closestPoint1,
                              int &nContacts, std::vector<Vector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  const T parallelTolerance = T(1.0)-0.05;
  const T perpendicularTolerance = 0.05;

  //transform cylinder to box coordinate system
  //get the closest feature of the box
  Matrix3x3<T> matBasis0 = transform0.getMatrix().GetTransposedMatrix();
  Vector3<T> v0Local = closestPoint1 - transform0.getOrigin();
  v0Local = matBasis0 * v0Local; 

  Vector3<T> c1 = matBasis0 * (transform1.getOrigin() - transform0.getOrigin());
  Vector3<T> delta = c1-cylinder0.getCenter();

  Vector3<T> ulocal1 = matBasis0 * (transform1.getMatrix() * cylinder1.getU());

  T projDelta = cylinder0.getU() * v0Local;
  if(fabs(projDelta) >= cylinder0.getHalfLength())
  {
    //top section
    T dotUU = cylinder0.getU() * ulocal1;
    //check for face-face
    if(dotUU > parallelTolerance)
    {
      //intersection circle circle
      //project center of cylinder onto plane
      CVector2<T> circleCenter0 = CVector2<T>(cylinder0.getCenter().x,cylinder0.getCenter().y);
      CVector2<T> circleCenter1 = CVector2<T>(c1.x,c1.y);
      CIntersectorCircleCircle<T> intersector(circleCenter0,circleCenter1,
                                           cylinder0.getRadius(),cylinder1.getRadius());
      if(intersector.Find())
      {
        //transform the contact points to world coordinates
        for(int i=0;i<intersector.m_iNumIntersections;i++)
        {
          Vector3<T> v3d(intersector.m_vPoints[i].x,intersector.m_vPoints[i].y,v0Local.z);
          vContacts.push_back(transform0.getMatrix() * v3d + transform0.getOrigin());
        }
      }
    }
    //check for face-edge
    else if(dotUU < perpendicularTolerance)
    {
      //intersection sphere segment
      Sphere<T> sphere(cylinder0.getCenter() + projDelta*cylinder0.getU(),cylinder0.getRadius());
      Vector3<T> centerSegment(c1.x,c1.y,v0Local.z);
      Segment3<T> segment(centerSegment,ulocal1,cylinder1.getHalfLength());
      IntersectorSphereSegment<T> sphereSegment(sphere,segment);
      sphereSegment.intersection();
      //transform the contact points to world coordinates
      for(int k=0;k<sphereSegment.numIntersections_;k++)
      {
        vContacts.push_back(transform0.getMatrix() * sphereSegment.points_[k] + transform0.getOrigin());
      }
    }
    //face-vertex
    else
    {
      //translate contact point along normal
      vContacts.push_back(closestPoint0);
    }
  }
  else
  {
    //middle section
    T dotUU = cylinder0.getU() * ulocal1;
    //edge-edge
    if(dotUU > parallelTolerance)
    {
      //Vector3<T> closestLocal0 = closestPoint0 - transform0.getOrigin();
      //closestLocal0 = matBasis0 * closestLocal0;
      //Segment3<T> seg0(Vector3<T>(closestLocal0.x,closestLocal0.y,0),cylinder0.getU(),cylinder0.getHalfLength());
      //Segment3<T> seg1(Vector3<T>(closestLocal0.x,closestLocal0.y,c1.z),ulocal1,cylinder1.getHalfLength());
      //SegmentSegmentPlanar(
      vContacts.push_back(closestPoint0);
    }
    //face-edge
    //perpendicular and closest0 in top section of
    else if(dotUU < perpendicularTolerance)
    {
      Vector3<T> closestLocal0 = closestPoint0 - transform0.getOrigin();
      closestLocal0 = matBasis0 * closestLocal0; 
      Vector3<T> vNormal = closestLocal0 - v0Local;
      T dist = vNormal.mag();
      vNormal /= dist;
      if(fabs(vNormal * ulocal1) > parallelTolerance)
      {
        //intersection sphere segment
        Vector3<T> sphereCenter = c1 + (dist+cylinder1.getHalfLength()) * vNormal;
        Sphere<T> sphere(sphereCenter,cylinder1.getRadius());
        Vector3<T> centerSegment(closestLocal0.x,closestLocal0.y,0);
        Segment3<T> segment(centerSegment,cylinder0.getU(),cylinder0.getHalfLength());
        IntersectorSphereSegment<T> sphereSegment(sphere,segment);
        sphereSegment.intersection();
        //transform the contact points to world coordinates
        for(int k=0;k<sphereSegment.numIntersections_;k++)
        {
          vContacts.push_back(transform0.getMatrix() * sphereSegment.points_[k] + transform0.getOrigin());
        }
      }
      else
      {
        vContacts.push_back(closestPoint0);
      }
    }
    //vertex-vertex
    else
    {
      //translate contact point along normal
      vContacts.push_back(closestPoint0);
    }
  }
}

template <class T>
void CIntersectorTools<T>::ComputeContactSet(const Cylinder<T> &cylinder, const OBB3<T> &box,
                                             CSimplexDescriptorGjk<T> &simplex,
                                             const Transformation<T> &transform0, const Transformation<T> &transform1,
                                             const Vector3<T> &closestPoint0, const Vector3<T> &closestPoint1,
                                             int &nContacts, std::vector<Vector3<T> > &vContacts)
{
  int i;
  unsigned int iregion,iRegionType;
  const T parallelTolerance = T(1.0)-E1;
  const T perpendicularTolerance = 0.05;

  //transform cylinder to box coordinate system
  //get the closest feature of the box
  Matrix3x3<T> matBasis1 = transform1.getMatrix().GetTransposedMatrix();
  Vector3<T> v1Local = closestPoint1 - transform1.getOrigin();
  v1Local = matBasis1 * v1Local; 
  Vector3<T> v1SuppLocal[3];
  unsigned int region1[3];

  int vertexCount = simplex.GetVertexCount();

  for(i=0;i<vertexCount;i++)
  {
    v1SuppLocal[i] = simplex.getSupportB(i) - transform1.getOrigin();
    v1SuppLocal[i] = matBasis1 * v1SuppLocal[i];
    region1[i] = box.classifyVertexOnSurface(v1SuppLocal[i]);
  }

  //1=-xface s
  iregion = region1[0];
  for(i=1;i<vertexCount;i++)
    iregion&=region1[i];

  //get the region of the box that the final simplex vertices
  //are located in
  iRegionType = box.getRegionType(iregion);

  //the vertices of the simplex are a face
  if(iRegionType==FACE)
  {
    Vector3<T> vNormal = box.getFaceNormal(iregion);
    //orientation of the cylinder to the box

    //test if u*face normal == 0 or ==1
    Vector3<T> centerLocal = transform0.getOrigin() - transform1.getOrigin();
    centerLocal = matBasis1 * centerLocal;
    Vector3<T> ulocal = (matBasis1 * transform0.getMatrix()) * cylinder.getU();
    T dotUF = ulocal * vNormal;

    //if the u-axis of the cylinder and the face normal
    //are parallel then we have a face-face configuration
    if(fabs(dotUF) > parallelTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);
      Vector3<T> circleCenter;

      //project the center of the circle onto the face defined by the
      //face of the box
      ProjectPointOnBoxPlane(box,iregion,centerLocal,circleCenter);

      //clip the circular face and the rectangular face to get
      //the contact manifold
      ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    //if the u-axis of the cylinder and face noramel are
    //perpendicular then we have a face-edge configuration
    else if(dotUF < perpendicularTolerance)
    {
      //get the face of the box
      Rectangle3<T> rec = box.getRegionFace(iregion);

      Vector3<T> seg[2];
      Vector3<T> rectangle[4];
      rec.computeVertices(rectangle);

      //project the end points of the segment onto the plane defined
      //by the face of the box
      ProjectLineOnBoxPlane(box,iregion,
                            centerLocal+cylinder.getHalfLength()*ulocal,
                            centerLocal-cylinder.getHalfLength()*ulocal,
                            seg);

      //compute the intersection of the edge and the face
      //to get the contact points
      SegmentRectanglePlanar(seg,rectangle,nContacts, vContacts);

      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    //in all other cases one contact point is sufficient
    else
    {
      //only one contact point and exit
      vContacts.push_back(closestPoint1);
    }
  }
  //the vertices of the simplex are an edge
  else if(iRegionType==EDGE)
  {
    Segment3<T> seg = box.getRegionEdge(iregion);
    //test if u*edge normal == 0 or ==1
    Vector3<T> centerLocal = transform0.getOrigin() - transform1.getOrigin();
    centerLocal = matBasis1 * centerLocal;
    Vector3<T> ulocal = matBasis1 * cylinder.getU();
    T dotUF = ulocal * seg.dir_;
    //if the edge is perpendicular to the u-axis
    if(dotUF < perpendicularTolerance)
    {
      unsigned int faces[2];
      int index=-1;
      box.getFacesAtEdge(iregion,faces);
      Rectangle3<T> rec;

      //check if the u-axis is parallel to the
      //faces connected to the edge
      for(int j=0;j<2;j++)
      {
        Vector3<T> vNormal = box.getFaceNormal(faces[j]);
        if(ulocal * vNormal > parallelTolerance)
        {
          index=j;
        }
      }

      //if we found a parallel face then
      //we have a face-face configuration
      if(index >= 0)
        rec=box.getRegionFace(faces[index]);
      
      Vector3<T> circleCenter;
      ProjectPointOnBoxPlane(box,faces[index],centerLocal,circleCenter);
      ClipCircleRectangle(rec,circleCenter,cylinder.getRadius(),vContacts);
      //transform the contact points to world coordinates
      for(int i=0;i<vContacts.size();i++)
      {
        vContacts[i] = transform1.getMatrix() * vContacts[i] + transform1.getOrigin();
      }
    }
    else
    {
      vContacts.push_back(closestPoint1);
    }
  }
  //the vertices of the simplex are a vertex
  else if(iRegionType==VERTEX)
  {
    std::cout<<"vertex region"<<std::endl;

    //in this configuration there can be
    //only one contact point
    vContacts.push_back(closestPoint1);
  }
}

template <class T>
void CIntersectorTools<T>::ComputeContactSetGjk(const ConvexShape<T> &shape0, const ConvexShape<T> &shape1, int shapeId0, int shapeId1, 
                                                CSimplexDescriptorGjk<T> &simplex,
                                                const Transformation<T> &transform0, const Transformation<T> &transform1,
                                                const Vector3<T> &closestPoint0, const Vector3<T> &closestPoint1,
                                                int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  if(shapeId0==7)
  {
    if(shapeId1==0)
    {
      //cylinder box
      const Cylinder<T> &cylinder = dynamic_cast<const Cylinder<T>& >(shape0);
      const Sphere<T> &sphere     = dynamic_cast<const Sphere<T>& >(shape1);
      //ComputeContactSet(cylinder0,cylinder1,simplex,transform0,transform1,closestPoint0,closestPoint1,nContacts,vContacts);
    }
    else if(shapeId1==1)
    {
      //cylinder box
      const Cylinder<T> &cylinder = dynamic_cast<const Cylinder<T>& >(shape0);
      const OBB3<T> &box = dynamic_cast<const OBB3<T>& >(shape1);
      ComputeContactSet(cylinder,box,simplex,transform0,transform1,closestPoint0,closestPoint1,nContacts,vContacts);
    }
    else if(shapeId1==7)
    {
      //cylinder box
      const Cylinder<T> &cylinder0 = dynamic_cast<const Cylinder<T>& >(shape0);
      const Cylinder<T> &cylinder1 = dynamic_cast<const Cylinder<T>& >(shape1);
      ComputeContactSet(cylinder0,cylinder1,simplex,transform0,transform1,closestPoint0,closestPoint1,nContacts,vContacts);
    }
  }
}

template <class T>
void CIntersectorTools<T>::ComputeContactSet(const OBB3<T> &box0, const OBB3<T> &box1, int iside, const CProjCfg<T> &cfg0,
                           const CProjCfg<T> &cfg1, const Vector3<T> &vel0, const Vector3<T> &vel1,
                           int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  Vector3<T> relVel  = vel0-vel1;
  Real        nRelVel = cfg0.m_vMinNormal * relVel;

  CPredictionTransform<T,OBB3<T> > transform;

  OBB3<T> newBox0;
  OBB3<T> newBox1;

  Vector3<T> vertices0[8];
  Vector3<T> vertices1[8];

  
  if(cfg0.m_iMinOrientation==1)
  {

    if(nRelVel < 0)
    {
      //get the boxes at time of impact
      newBox0 = transform.PredictLinearMotion(box0,T(0.5)*cfg0.m_dMinOverlap*cfg0.m_vMinNormal);
      newBox1 = transform.PredictLinearMotion(box1,T(-0.5)*cfg1.m_dMinOverlap*cfg1.m_vMinNormal);
    }
    else
    {
      //get the boxes at time of impact
      newBox0 = transform.PredictLinearMotion(box0,T(-0.5)*cfg0.m_dMinOverlap*cfg0.m_vMinNormal);
      newBox1 = transform.PredictLinearMotion(box1,T(0.5)*cfg1.m_dMinOverlap*cfg1.m_vMinNormal);
    }

    newBox0.computeVertices(vertices0);
    newBox1.computeVertices(vertices1);

    //check the box0 closest feature
    if(cfg0.getMinFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices0[cfg0.m_iMinFeatureIndex[0]]);
    }
    else if(cfg1.getMinFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices1[cfg1.m_iMinFeatureIndex[4]]);
    }
    //the only remaining cases are now EE,EF,FF
    else if(cfg0.getMinFeature() == CProjCfg<T>::BOX_EDGE)
    {
      //check configuration of box1 for edge or face
      if(cfg1.getMinFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-edge
        Vector3<T> edge0[2];
        Vector3<T> edge1[2];
        edge0[0] = getPoint(cfg0.m_iMinFeatureIndex[0],box0);
        edge0[1] = getPoint(cfg0.m_iMinFeatureIndex[1],box0);
        edge1[0] = getPoint(cfg1.m_iMinFeatureIndex[4],box1);
        edge1[1] = getPoint(cfg1.m_iMinFeatureIndex[5],box1);

        //compute intersection segment-segment
        FindSegmentSegment(edge0,edge1,nContacts,vContacts);
      }
      else
      {
        //edge-face
        Vector3<T> edge0[2];
        Vector3<T> face1[4];
        edge0[0] = getPoint(cfg0.m_iMinFeatureIndex[0],box0);
        edge0[1] = getPoint(cfg0.m_iMinFeatureIndex[1],box0);
        face1[0] = getPoint(cfg1.m_iMinFeatureIndex[4],box1);
        face1[1] = getPoint(cfg1.m_iMinFeatureIndex[5],box1);
        face1[2] = getPoint(cfg1.m_iMinFeatureIndex[6],box1);
        face1[3] = getPoint(cfg1.m_iMinFeatureIndex[7],box1);

        //project edge to face
        Vector3<T> v0 = face1[1]-face1[0];
        Vector3<T> v1 = face1[2]-face1[0];
        Vector3<T> norm = Vector3<T>::Cross(v0,v1);
        norm.Normalize();
        Vector3<T> middle = 0.25 * (face1[0]+face1[1]+face1[2]+face1[3]);
        Vector3<T> vE0 = edge0[0] - middle;
        Vector3<T> vE1 = edge0[1] - middle;
        T d0 = vE0 * norm;
        T d1 = vE1 * norm;
        edge0[0] -= d0 * norm;
        edge0[1] -= d1 * norm;

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge0,face1,nContacts,vContacts);
      }
    }
    else
    {
      //the box0 feature is a face
      //now check for face-edge or face-face
      if(cfg1.getMinFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-face
        Vector3<T> face0[4];
        Vector3<T> edge1[2];
        face0[0] = getPoint(cfg0.m_iMinFeatureIndex[0],box0);
        face0[1] = getPoint(cfg0.m_iMinFeatureIndex[1],box0);
        face0[2] = getPoint(cfg0.m_iMinFeatureIndex[2],box0);
        face0[3] = getPoint(cfg0.m_iMinFeatureIndex[3],box0);
        edge1[0] = getPoint(cfg1.m_iMinFeatureIndex[4],box1);
        edge1[1] = getPoint(cfg1.m_iMinFeatureIndex[5],box1);

        //project edge to face
        Vector3<T> v0 = face0[1]-face0[0];
        Vector3<T> v1 = face0[2]-face0[0];
        Vector3<T> norm = Vector3<T>::Cross(v0,v1);
        norm.Normalize();
        Vector3<T> middle = 0.25 * (face0[0]+face0[1]+face0[2]+face0[3]);
        Vector3<T> vE0 = edge1[0] - middle;
        Vector3<T> vE1 = edge1[1] - middle;
        T d0 = vE0 * norm;
        T d1 = vE1 * norm;
        edge1[0] -= d0 * norm;
        edge1[1] -= d1 * norm;

        //test loop
        for(int i0=3,i1=0;i1<4;i0=i1++)
        {
          Vector3<T> vN = face0[i1]-face0[i0];
          Vector3<T> vP = (middle - face0[i0]);
          Vector3<T> vV0 = (edge1[0] - face0[i0]);
          Vector3<T> vV1 = (edge1[1] - face0[i0]);
          T mydot0 = vP * vV0;
          T mydot1 = vP * vV1;

        }

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge1,face0,nContacts,vContacts);
      }
      else
      {
        //face-face
        Vector3<T> face0[4];
        Vector3<T> face1[4];
        face0[0] = getPoint(cfg0.m_iMinFeatureIndex[0],box0);
        face0[1] = getPoint(cfg0.m_iMinFeatureIndex[1],box0);
        face0[2] = getPoint(cfg0.m_iMinFeatureIndex[2],box0);
        face0[3] = getPoint(cfg0.m_iMinFeatureIndex[3],box0);
        face1[0] = getPoint(cfg1.m_iMinFeatureIndex[4],box1);
        face1[1] = getPoint(cfg1.m_iMinFeatureIndex[5],box1);
        face1[2] = getPoint(cfg1.m_iMinFeatureIndex[6],box1);
        face1[3] = getPoint(cfg1.m_iMinFeatureIndex[7],box1);

        //compute intersection coplanar face-face
        RectangleRectanglePlanar(face0,face1,nContacts,vContacts);
      }
    }

  }//end if(iside == CProjCfg<T>::LEFT)
  else
  {

    if(nRelVel < 0)
    {
      //get the boxes at time of impact
      newBox0 = transform.PredictLinearMotion(box0,T(0.5)*cfg0.m_dMinOverlap*cfg0.m_vMinNormal);
      newBox1 = transform.PredictLinearMotion(box1,T(-0.5)*cfg1.m_dMinOverlap*cfg1.m_vMinNormal);
    }
    else
    {
      //get the boxes at time of impact
      newBox0 = transform.PredictLinearMotion(box0,T(-0.5)*cfg0.m_dMinOverlap*cfg0.m_vMinNormal);
      newBox1 = transform.PredictLinearMotion(box1,T(0.5)*cfg1.m_dMinOverlap*cfg1.m_vMinNormal);
    }

    newBox0.computeVertices(vertices0);
    newBox1.computeVertices(vertices1);


    //box1 is on the right of box0
    if(cfg0.getMinFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices0[cfg0.m_iMinFeatureIndex[4]]);
    }
    else if(cfg1.getMinFeature() == CProjCfg<T>::BOX_VERTEX)
    {
      //in this case the contact set is a point
      nContacts = 1;
      vContacts.push_back(vertices1[cfg1.m_iMinFeatureIndex[0]]);
    }
    //the only remaining cases are now EE,EF,FF
    else if(cfg0.getMinFeature() == CProjCfg<T>::BOX_EDGE)
    {
      //check configuration of box1 for edge or face
      if(cfg1.getMinFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-edge
        //compute intersection segment-segment
        Vector3<T> edge0[2];
        Vector3<T> edge1[2];
        edge0[0] = getPoint(cfg0.m_iMinFeatureIndex[4],box0);
        edge0[1] = getPoint(cfg0.m_iMinFeatureIndex[5],box0);
        edge1[0] = getPoint(cfg1.m_iMinFeatureIndex[0],box1);
        edge1[1] = getPoint(cfg1.m_iMinFeatureIndex[1],box1);

        FindSegmentSegment(edge0,edge1,nContacts,vContacts);

      }
      else
      {
        //edge-face
        Vector3<T> edge0[2];
        Vector3<T> face1[4];
        edge0[0] = getPoint(cfg0.m_iMinFeatureIndex[4],box0);
        edge0[1] = getPoint(cfg0.m_iMinFeatureIndex[5],box0);
        face1[0] = getPoint(cfg1.m_iMinFeatureIndex[0],box1);
        face1[1] = getPoint(cfg1.m_iMinFeatureIndex[1],box1);
        face1[2] = getPoint(cfg1.m_iMinFeatureIndex[2],box1);
        face1[3] = getPoint(cfg1.m_iMinFeatureIndex[3],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge0,face1,nContacts,vContacts);
      }
    }
    else
    {
      //the box0 feature is a face
      //now check for face-edge or face-face
      if(cfg1.getMinFeature() == CProjCfg<T>::BOX_EDGE)
      {
        //edge-face
        Vector3<T> face0[4];
        Vector3<T> edge1[2];
        face0[0] = getPoint(cfg0.m_iMinFeatureIndex[4],box0);
        face0[1] = getPoint(cfg0.m_iMinFeatureIndex[5],box0);
        face0[2] = getPoint(cfg0.m_iMinFeatureIndex[6],box0);
        face0[3] = getPoint(cfg0.m_iMinFeatureIndex[7],box0);
        edge1[0] = getPoint(cfg1.m_iMinFeatureIndex[0],box1);
        edge1[1] = getPoint(cfg1.m_iMinFeatureIndex[1],box1);

        //compute intersection coplanar segment-face
        SegmentRectanglePlanar(edge1,face0,nContacts,vContacts);
      }
      else
      {
        //face-face
        Vector3<T> face0[4];
        Vector3<T> face1[4];
        face0[0] = getPoint(cfg0.m_iMinFeatureIndex[4],box0);
        face0[1] = getPoint(cfg0.m_iMinFeatureIndex[5],box0);
        face0[2] = getPoint(cfg0.m_iMinFeatureIndex[6],box0);
        face0[3] = getPoint(cfg0.m_iMinFeatureIndex[7],box0);
        face1[0] = getPoint(cfg1.m_iMinFeatureIndex[0],box1);
        face1[1] = getPoint(cfg1.m_iMinFeatureIndex[1],box1);
        face1[2] = getPoint(cfg1.m_iMinFeatureIndex[2],box1);
        face1[3] = getPoint(cfg1.m_iMinFeatureIndex[3],box1);

        //compute intersection coplanar face-face
        RectangleRectanglePlanar(face0,face1,nContacts,vContacts);
      }
    }
  }//end else

}


template <class T>
void CIntersectorTools<T>::FindSegmentSegment(Vector3<T> seg0[2],Vector3<T> seg1[2],int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  const T tolerance = (T)1 - E5;

  //compute direction vectors
  Vector3<T> vD0 = seg0[1] - seg0[0];
  Vector3<T> vD1 = seg1[1] - seg1[0];
  Vector3<T> vN  = Vector3<T>::Cross(vD0,vD1);
  vD0.Normalize();
  vD1.Normalize();
  //check if the segments are colinear
  if(vD0*vD1 > tolerance)
  {
    ColinearSegments(seg0,seg1,nContacts,vContacts);
  }
  else
  {
    SegmentSegmentPlanar(seg1,seg0[0],Vector3<T>::Cross(vN,(seg0[1]-seg0[0])),nContacts,vContacts);
  }
  
}

template <class T>
void CIntersectorTools<T>::SegmentSegmentPlanar(Vector3<T> seg[2],const Vector3<T> &origin, const Vector3<T> &normal, int &nContacts, std::vector<Vector3<T> > &vContacts)
{
  //in this case the number of contact points is 1
  nContacts = 1;

  //reparameterize the segment along the other segments normal
  T u0 = normal * origin;
  T v0 = normal * seg[0];
  T v1 = normal * seg[1];

  //compute the parameter of the intersection point
  T s0 = (u0-v0)/(v1-v0);
  vContacts.push_back(seg[0] + s0 * (seg[1]-seg[0]));

}

template <class T>
void CIntersectorTools<T>::ColinearSegments(Vector3<T> seg0[2],Vector3<T> seg1[2],int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  Vector3<T> P[8];
  //clip segment0 against segment1
  nContacts = 2;
  int i;
  for(i=0;i<2;i++)
    P[i] = seg0[i];

  Vector3<T> vDir = seg1[1]-seg1[0];
  T c = vDir * seg1[0];
  ClipConvexPolygonAgainstPlane(vDir,c,nContacts,P);

  vDir = -vDir;
  c = vDir * seg1[1];
  ClipConvexPolygonAgainstPlane(vDir,c,nContacts,P);

  for(i=0;i<nContacts;i++)
    vContacts.push_back(P[i]);
  
}

template <class T>
void CIntersectorTools<T>::SegmentRectanglePlanar(Vector3<T> seg[2],Vector3<T> rec[4],int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  Vector3<T> P[8];
  //the line segment is clipped against the sides of the rectangle
  nContacts = 2;
  int i;
  for(i=0;i<2;i++)
    P[i] = seg[i];

  for(int i0=3,i1=0;i1<4;i0=i1++)
  {
    Vector3<T> vN = rec[i1]-rec[i0];
    T c = vN * rec[i0];
    ClipConvexPolygonAgainstPlane(vN,c,nContacts,P);
  }

  for(i=0;i<nContacts;i++)
    vContacts.push_back(P[i]);
  
}

template <class T>
void CIntersectorTools<T>::RectangleRectanglePlanar(Vector3<T> rec0[4],Vector3<T> rec1[4],int &nContacts, std::vector<Vector3<T> > &vContacts)
{

  Vector3<T> P[8];
  //the line segment is clipped against the sides of the rectangle
  nContacts = 4;
  int i;
  for(i=0;i<4;i++)
    P[i] = rec0[i];

  for(int i0=3,i1=0;i1<4;i0=i1++)
  {
    Vector3<T> vN = rec1[i1]-rec1[i0];
    T c = vN * rec1[i0];
    ClipConvexPolygonAgainstPlane(vN,c,nContacts,P);
  }

  for(i=0;i<nContacts;i++)
    vContacts.push_back(P[i]);

}

template <class T>
void CIntersectorTools<T>::ClipCircleRectangle(const Rectangle3<T> &rec, const Vector3<T> &circleCenter, T radius, std::vector<Vector3<T> > &vContacts)
{

  //no intersection if distbetween centers > extent+radius

  //make an aabb for the circle
  Rectangle3<T> aabbCircle(circleCenter,rec.uv_[0],rec.uv_[1],radius,radius);
  int nContacts=0;
  //get vertices
  Vector3<T> rectangle0[4];
  rec.computeVertices(rectangle0);

  //get vertices
  Vector3<T> rectangle1[4];
  aabbCircle.computeVertices(rectangle1);
  bool inside = true;

  //test if the center of the circle
  //is located in the rectangle
  //(calculate distance?here)
  for(int i=0;i<4;i++)
  {
    Vector3<T> vDir = rectangle0[(i+1)%4] - rectangle0[i];
    Vector3<T> vIC  = circleCenter -  rectangle0[i];
    T dot = vDir * vIC;
    if(vDir * vIC < 0)
    {
      inside=false;
      break;
    }
  }
  if(inside)
  {
    //case circle center inside rectangle
    Sphere<T> sphere(circleCenter,radius);
    for(int j=0;j<4;j++)
    {
      Segment3<T> segment(rectangle0[j],rectangle0[(j+1)%4]);
      IntersectorSphereSegment<T> sphereSegment(sphere,segment);
      sphereSegment.intersection();
      for(int k=0;k<sphereSegment.numIntersections_;k++)
      {
        vContacts.push_back(sphereSegment.points_[k]);
      }
    }

    //TODO: this may not be correct if the rectancle is smaller
    //than the circle
    if(vContacts.size()==0)
    {
      vContacts.push_back(circleCenter + radius * rec.uv_[0]);
      vContacts.push_back(circleCenter - radius * rec.uv_[0]);
      vContacts.push_back(circleCenter + radius * rec.uv_[1]);
      vContacts.push_back(circleCenter - radius * rec.uv_[1]);
    }
    else if(vContacts.size()==1)
    {
      vContacts.push_back(circleCenter);
    }
    else if(vContacts.size()==2)
    {
      for(int i=0;i<4;i++)
      {
        Vector3<T> vDir = rectangle0[i] - circleCenter;
        if(vDir * vDir < radius * radius)
        {
          vContacts.push_back(rectangle0[i]);
        }
      }
      Vector3<T> dir =  rec.center_ - circleCenter;
      dir.Normalize();
      vContacts.push_back(circleCenter + dir * radius);
      
    }
    else if(vContacts.size()==3)
    {
      for(int i=0;i<4;i++)
      {
        Vector3<T> vDir = rectangle0[i] - circleCenter;
        if(vDir * vDir < radius * radius)
        {
          vContacts.push_back(rectangle0[i]);
        }
      }
      Vector3<T> dir =  rec.center_ - circleCenter;
      dir.Normalize();
      vContacts.push_back(circleCenter + dir * radius);
    }
    else if(vContacts.size()==4)
    {
      vContacts.push_back(circleCenter);
    }
  }
  else
  {
    //circle center not inside the rectangle
    //add the rectangle vertices that are inside
    //the circle
    for(int i=0;i<4;i++)
    {
      Vector3<T> vDir = rectangle0[i] - circleCenter;
      if(vDir * vDir < radius * radius)
      {
        vContacts.push_back(rectangle0[i]);
      }
    }

    //case circle center inside rectangle
    Sphere<T> sphere(circleCenter,radius);
    for(int j=0;j<4;j++)
    {
      Segment3<T> segment(rectangle0[j],rectangle0[(j+1)%4]);
      IntersectorSphereSegment<T> sphereSegment(sphere,segment);
      sphereSegment.intersection();
      for(int k=0;k<sphereSegment.numIntersections_;k++)
      {
        vContacts.push_back(sphereSegment.points_[k]);
      }
    }

    if(vContacts.size() > 0)
    {
      Vector3<T> dir =  rec.center_ - circleCenter;
      dir.Normalize();
      vContacts.push_back(circleCenter + dir * radius);
    }
  }
}

template <class T>
void CIntersectorTools<T>::ClipConvexPolygonAgainstPlane(const Vector3<T>& normal,T constant, int& quantity, Vector3<T> *P)
{
    // The input vertices are assumed to be in counterclockwise order.  The
    // ordering is an invariant of this function.  The size of array P is
    // assumed to be large enough to store the clipped polygon vertices.

    // test on which side of line are the vertices
    int positive = 0, negative = 0, pIndex = -1;
    int currQuantity = quantity;

    T test[8];
    int i;
    for (i = 0; i < quantity; ++i)
    {

        // An epsilon is used here because it is possible for the dot product
        // and 'constant' to be exactly equal to each other (in theory), but
        // differ slightly because of doubleing point problems.  Thus, add a
        // little to the test number to push actually equal numbers over the
        // edge towards the positive.

        // TODO: This should probably be a relative tolerance.  Multiplying
        // by the constant is probably not the best way to do this.
        test[i] = normal * P[i] - constant +
            fabs(constant)*CMath<T>::TOLERANCEZERO;

        if (test[i] >= (T)0)
        {
            ++positive;
            if (pIndex < 0)
            {
                pIndex = i;
            }
        }
        else
        {
            ++negative;
        }
    }

    if (quantity == 2)
    {
        // Lines are a little different, in that clipping the segment
        // cannot create a new segment, as clipping a polygon would
        if (positive > 0)
        {
            if (negative > 0) 
            {
                int clip;

                if (pIndex == 0)
                {
                    // vertex0 positive, vertex1 is clipped
                    clip = 1;
                }
                else // pIndex == 1
                {
                    // vertex1 positive, vertex0 clipped
                    clip = 0;
                }

                T t = test[pIndex]/(test[pIndex] - test[clip]);
                P[clip] = P[pIndex] + t*(P[clip] - P[pIndex]);
            }
            // otherwise both positive, no clipping
        }
        else
        {
            // Assert:  The entire line is clipped, but we should not
            // get here.
            quantity = 0;
        }
    }
    else
    {
        if (positive > 0)
        {
            if (negative > 0)
            {
                // plane transversely intersects polygon
                Vector3<T> CV[8];
                int cQuantity = 0, cur, prv;
                T t;

                if (pIndex > 0)
                {
                    // first clip vertex on line
                    cur = pIndex;
                    prv = cur - 1;
                    t = test[cur]/(test[cur] - test[prv]);
                    CV[cQuantity++] = P[cur] + t*(P[prv] - P[cur]);

                    // vertices on positive side of line
                    while (cur < currQuantity && test[cur] >= (T)0)
                    {
                        CV[cQuantity++] = P[cur++];
                    }

                    // last clip vertex on line
                    if (cur < currQuantity)
                    {
                        prv = cur - 1;
                    }
                    else
                    {
                        cur = 0;
                        prv = currQuantity - 1;
                    }
                    t = test[cur]/(test[cur] - test[prv]);
                    CV[cQuantity++] = P[cur] + t*(P[prv] - P[cur]);
                }
                else  // pIndex is 0
                {
                    // vertices on positive side of line
                    cur = 0;
                    while (cur < currQuantity && test[cur] >= (T)0)
                    {
                        CV[cQuantity++] = P[cur++];
                    }

                    // last clip vertex on line
                    prv = cur - 1;
                    t = test[cur]/(test[cur] - test[prv]);
                    CV[cQuantity++] = P[cur] + t*(P[prv] - P[cur]);

                    // skip vertices on negative side
                    while (cur < currQuantity && test[cur] < (T)0)
                    {
                        cur++;
                    }

                    // first clip vertex on line
                    if (cur < currQuantity)
                    {
                        prv = cur - 1;
                        t = test[cur]/(test[cur] - test[prv]);
                        CV[cQuantity++] = P[cur] + t*(P[prv] - P[cur]);

                        // vertices on positive side of line
                        while (cur < currQuantity && test[cur] >= (T)0)
                        {
                            CV[cQuantity++] = P[cur++];
                        }
                    }
                    else
                    {
                        // cur = 0
                        prv = currQuantity - 1;
                        t = test[0]/(test[0] - test[prv]);
                        CV[cQuantity++] = P[0] + t*(P[prv] - P[0]);
                    }
                }

                currQuantity = cQuantity;

                memcpy(P, CV, cQuantity*sizeof(Vector3<T>));
            }
            // else polygon fully on positive side of plane, nothing to do

            quantity = currQuantity;
        }
        else
        {
            // Polygon does not intersect positive side of plane, clip all.
            // This should not ever happen if called by the findintersect
            // routines after an intersection has been determined.
            quantity = 0;
        }    
    }
}


template <class T>
bool CIntersectorTools<T>::Find(const Vector3<T> &testAxis, const OBB3<T> &box0, const OBB3<T> &box1,
                                CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1)
{

  CProjCfg<T> cfgStart0;
  CProjCfg<T> cfgStart1;

  //compute the relative orientation for the axis
  getProjCfg(testAxis,box0,cfgStart0);
  getProjCfg(testAxis,box1,cfgStart1);

  //compute the first and last intersection with regard to this axis and
  //update the final configuration if neccessary
  return Find(testAxis,cfgStart0,cfgStart1,cfgFinal0,cfgFinal1);
}

template <class T>
bool CIntersectorTools<T>::Find(const Vector3<T> &testAxis,CProjCfg<T> &cfgCurr0,
                                CProjCfg<T> &cfgCurr1, CProjCfg<T> &cfgFinal0, CProjCfg<T> &cfgFinal1)
{

  //we know the objects overlap on this axis
  //let us compute the overlap

    T minOverlap;
    //compute the actual overlap
    if(cfgCurr1.m_dMax > cfgCurr0.m_dMax)
    {
      if((minOverlap=(cfgCurr0.m_dMax - cfgCurr1.m_dMin)/testAxis.mag()) < cfgFinal0.m_dMinOverlap)
      {
        cfgFinal0.m_dMinOverlap = minOverlap;
        cfgFinal0.m_vMinNormal  = testAxis;
        cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
        memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
        cfgFinal0.m_iMinOrientation = 2;
        cfgFinal1.m_dMinOverlap = minOverlap;
        cfgFinal1.m_vMinNormal  = testAxis;
        cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
        memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
        cfgFinal1.m_iMinOrientation = 1;
      }
    }
    else
    {
      if((minOverlap=(cfgCurr1.m_dMax - cfgCurr0.m_dMin)/testAxis.mag()) < cfgFinal0.m_dMinOverlap)
      {
        cfgFinal0.m_dMinOverlap = minOverlap;
        cfgFinal0.m_vMinNormal  = testAxis;
        cfgFinal0.m_iMinFeature = cfgCurr0.m_iFeature;
        memcpy(&cfgFinal0.m_iMinFeatureIndex,&cfgCurr0.m_iFeatureIndex,8*sizeof(int));
        cfgFinal0.m_iMinOrientation = 1;
        cfgFinal1.m_dMinOverlap = minOverlap;
        cfgFinal1.m_vMinNormal  = testAxis;
        cfgFinal1.m_iMinFeature = cfgCurr1.m_iFeature;
        memcpy(&cfgFinal1.m_iMinFeatureIndex,&cfgCurr1.m_iFeatureIndex,8*sizeof(int));
        cfgFinal1.m_iMinOrientation = 2;
      }
    }

  return true;
}

template <class T>
void CIntersectorTools<T>::ProjectLineOnBoxPlane(const OBB3<T> &box, unsigned int plane, const Vector3<T> &v0, const Vector3<T> &v1, Vector3<T> seg[2])
{
  switch(plane)
  {
    case 1:
      seg[0] = Vector3<T>(-box.extents_[0],v0.y,v0.z);
      seg[1] = Vector3<T>(-box.extents_[0],v1.y,v1.z);
      break;
    case 2:
      seg[0] = Vector3<T>(box.extents_[0],v0.y,v0.z);
      seg[1] = Vector3<T>(box.extents_[0],v1.y,v1.z);
      break;
    case 4:
      seg[0] = Vector3<T>(v0.x,-box.extents_[1],v0.z);
      seg[1] = Vector3<T>(v1.x,-box.extents_[1],v1.z);
      break;
    case 8:
      seg[0] = Vector3<T>(v0.x,box.extents_[1],v0.z);
      seg[1] = Vector3<T>(v1.x,box.extents_[1],v1.z);
      break;
    case 16:
      seg[0] = Vector3<T>(v0.x,v0.y,-box.extents_[2]);
      seg[1] = Vector3<T>(v1.x,v1.y,-box.extents_[2]);
      break;
    case 32:
      seg[0] = Vector3<T>(v0.x,v0.y,box.extents_[2]);
      seg[1] = Vector3<T>(v1.x,v1.y,box.extents_[2]);
      break;
  }
}

template <class T>
void CIntersectorTools<T>::ProjectPointOnBoxPlane(const OBB3<T> &box, unsigned int plane, const Vector3<T> &v0, Vector3<T> &point)
{
  switch(plane)
  {
    case 1:
      point = Vector3<T>(-box.extents_[0],v0.y,v0.z);
      break;
    case 2:
      point = Vector3<T>(box.extents_[0],v0.y,v0.z);
      break;
    case 4:
      point = Vector3<T>(v0.x,-box.extents_[1],v0.z);
      break;
    case 8:
      point = Vector3<T>(v0.x,box.extents_[1],v0.z);
      break;
    case 16:
      point = Vector3<T>(v0.x,v0.y,-box.extents_[2]);
      break;
    case 32:
      point = Vector3<T>(v0.x,v0.y,box.extents_[2]);
      break;
  }
}


//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class CIntersectorTools<Real>;

//----------------------------------------------------------------------------

}
