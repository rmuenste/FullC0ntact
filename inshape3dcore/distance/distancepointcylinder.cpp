#include "distancepointcylinder.h"

namespace i3d {

template <typename T>
CDistancePointCylinder<T>::CDistancePointCylinder(void)
{
}

template <typename T>
CDistancePointCylinder<T>::CDistancePointCylinder(const Vector3<T>& point, const Cylinder<T>& cylinder)
{
  m_Cylinder = cylinder;
  m_vPoint = point;
}



template <typename T>
CDistancePointCylinder<T>::~CDistancePointCylinder(void)
{
  
}

template <typename T>
T CDistancePointCylinder<T>::ComputeDistanceSqr()
{
  T sqrDistance;
    
  return sqrDistance;
}

template <typename T>
T CDistancePointCylinder<T>::ComputeDistance()
{

  if(!m_Cylinder.isPointInside(m_vPoint))
  { 
    //The point is located over the cylinder
    if(m_vPoint.z >= m_Cylinder.getHalfLength())
    {
      //compute distance to the top disk
      if( (m_vPoint.x*m_vPoint.x) + (m_vPoint.y*m_vPoint.y) < m_Cylinder.getRadius() * m_Cylinder.getRadius() )
      {
        T distance = m_vPoint.z - m_Cylinder.getHalfLength();
        return distance;        
      }
      else
      {
        Vector3<T> diskCenter(0,0,m_Cylinder.getHalfLength());
        Vector3<T> dir = Vector3<T>(m_vPoint.x,m_vPoint.y,0);
        dir.Normalize();
        Vector3<T> pointOnRing = diskCenter + m_Cylinder.getRadius() * dir;
        Vector3<T> diff = m_vPoint - pointOnRing;
        T distance = diff.mag();        
        return distance;        
      }
    }
    //point is located under the cylinder
    else if(m_vPoint.z <= -m_Cylinder.getHalfLength())
    {
      //compute distance to the top disk
      if( (m_vPoint.x*m_vPoint.x) + (m_vPoint.y*m_vPoint.y) < m_Cylinder.getRadius() * m_Cylinder.getRadius() )
      {
        T distance = fabs(m_vPoint.z) - m_Cylinder.getHalfLength();
        return distance;        
      }
      else
      {
        Vector3<T> diskCenter(0,0,-m_Cylinder.getHalfLength());
        Vector3<T> dir = Vector3<T>(m_vPoint.x,m_vPoint.y,0);
        dir.Normalize();
        Vector3<T> pointOnRing = diskCenter + m_Cylinder.getRadius() * dir;
        Vector3<T> diff = m_vPoint - pointOnRing;
        T distance = diff.mag();
        return distance;        
      }
    }
    //point is in the mid section
    else
    {
      T distance = m_vPoint.mag() - m_Cylinder.getRadius();
      return distance;
    }
  }
  else
  {
    T distance = m_Cylinder.getRadius() - m_vPoint.mag();
    if(m_vPoint.z > 0.0)
    {
      T distTop = m_Cylinder.getRadius() - m_vPoint.z;
      if(distTop < distance) 
        return distTop;
      else
        return distance;
    }
    else
    {
      T distBottom = m_Cylinder.getRadius() - fabs(m_vPoint.z);      
      if(distBottom < distance)
        return distBottom;
      else
        return distance;
    }
  }
  
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistancePointCylinder<float>;
template class CDistancePointCylinder<double>;

}