/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "intersectormpr.h"
#include <mymath.h>
#include <vtkwriter.h>
#include <vector>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace i3d {

template<class T>
CIntersectorMPR<T>::CIntersectorMPR()
{
  intersection = false;
}//end constructor

template<class T>
CIntersectorMPR<T>::CIntersectorMPR(const CConvexShape<T> &shape0, const CConvexShape<T> &shape1)
{
  m_pShape0 = &shape0;
  m_pShape1 = &shape1;
  intersection = false;
}

template<class T>
bool CIntersectorMPR<T>::Intersection()
{

  //Phase 1: find an initial portal
  FindInitialPortal();

  CheckPortalRay();

  RefinePortal();

  return intersection;

}//end Intersection

template<class T>
void CIntersectorMPR<T>::FindInitialPortal()
{

  //get the origin of the minkowski difference shape0.center - shape1.center
  v = m_pShape0->GetCenter() - m_pShape1->GetCenter();

  //get the support point in the direction of -v
  a = m_pShape0->GetSupport(-v) - m_pShape1->GetSupport(v);

  //check for collinearity

  CVector3<T> va = CVector3<T>::Cross(v,a);

  b = m_pShape0->GetSupport(va) - m_pShape1->GetSupport(-va);

  CVector3<T> avbv = CVector3<T>::Cross(a-v,b-v);

  c = m_pShape0->GetSupport(avbv) - m_pShape1->GetSupport(-avbv);
  
  std::vector< CVector3<T> > verts;
  verts.push_back(v);
  verts.push_back(a);
  verts.push_back(b);
  verts.push_back(c);  
  verts.push_back(CVector3<T>(0,0,0));  
  
  std::string sFileName("output/MPR_initial.vtk");
  std::ostringstream sName;
  CVtkWriter writer;
  writer.WriteMPR(verts, 0, sFileName.c_str());    
  
}//end FindInitialPortal

template<class T>
void CIntersectorMPR<T>::CheckPortalRay()
{
  //the origin ray
  CVector3<T> r = -v;
  int iter=0;
  bool stop;
  do {

    stop = true;
    //compute the triangle normals fo the three other triangles
    CVector3<T> nvab = CVector3<T>::Cross((a-v),(b-v));
    CVector3<T> nvbc = CVector3<T>::Cross((b-v),(c-v));
    CVector3<T> nvca = CVector3<T>::Cross((c-v),(a-v));

    //check the direction of the normals
    if(r*nvab > 0)
    {
      //swap a and b to invert the normal
      CVector3<T> temp = a;
      a = b;
      b = temp;

      //we can replace the third point by one that is closer to the origin
      c = m_pShape0->GetSupport(nvab) - m_pShape1->GetSupport(-nvab);
      stop = false;
    }
    else if(r*nvbc > 0)
    {
      //swap b and c to invert the normal
      CVector3<T> temp = b;
      b = c;
      c = temp;

      //we can replace the third point by one that is closer to the origin
      a = m_pShape0->GetSupport(nvbc) - m_pShape1->GetSupport(-nvbc);
      stop = false;
    }
    else if(r*nvca > 0)
    {
      //swap b and c to invert the normal
      CVector3<T> temp = c;
      c = a;
      a = temp;

      //we can replace the third point by one that is closer to the origin
      b = m_pShape0->GetSupport(nvca) - m_pShape1->GetSupport(-nvca);
      stop = false;
    }

    std::vector< CVector3<T> > verts;
    verts.push_back(v);
    verts.push_back(a);
    verts.push_back(b);
    verts.push_back(c);  
    verts.push_back(CVector3<T>(0,0,0));  
    
    std::string sFileName("output/MPR_check");
    std::ostringstream sName;
    sName<<"."<<std::setfill('0')<<std::setw(3)<<iter<<".vtk";
    sFileName.append(sName.str());
    
    CVtkWriter writer;
    writer.WriteMPR(verts, 0, sFileName.c_str());
    iter++;
  }while(!stop);

}//end CheckPortalRay

template<class T>
void CIntersectorMPR<T>::RefinePortal()
{
  int iter=0;
  CVector3<T> n_old(0,0,0);
  while(true) {
    CVector3<T> n = CVector3<T>::Cross((c-a),(b-a));
    n.Normalize();
    std::cout<<"n * n_old = "<<n*n_old<<" iter="<<iter<<std::endl;
    //check convergence criterion
    if(n*n_old > 1 - CMath<T>::EPSILON5)
    {
      return;
    }

    //if (a-0)*n > 0 then the origin is inside the tetrahedron
    if(a*n > 0.0)
    {
      //intersection
      intersection = true;
      std::cout<<"Intersection=true"<<std::endl;
      //return;
    }

    std::vector< CVector3<T> > verts;
    verts.push_back(v);
    verts.push_back(a);
    verts.push_back(b);
    verts.push_back(c);  
    verts.push_back(CVector3<T>(0,0,0));  
    
    CVector3<T> mid=(1.0/3.0)*(a+b+c);
    verts.push_back(mid);  

    CVector3<T> nnorm=n;
    nnorm/=nnorm.mag();
    verts.push_back(mid+v.mag()*nnorm);

    std::string sFileName("output/MPR_refine");
    std::ostringstream sName;
    sName<<"."<<std::setfill('0')<<std::setw(3)<<iter<<".vtk";
    sFileName.append(sName.str());
    
    CVtkWriter writer;
    writer.WriteMPR(verts, 0, sFileName.c_str());

    //
    if((a*n < CMath<T>::EPSILON3) && (a*n > -CMath<T>::EPSILON3))
    {
      intersection = true;
      std::cout<<"Origin in Portal plane"<<std::endl;
      //return;
    }

    //get the support point in the direction of n
    CVector3<T> p = m_pShape0->GetSupport(n) - m_pShape1->GetSupport(-n);
  
    //origin is outside
    if(p * n < -CMath<T>::EPSILON4)
    {
      intersection = false;
      std::cout<<"Origin outside... p*n: "<<p*n<<" iter: "<<iter<<std::endl;
      return;
    }

    //if the origin is in the positive half-space of pva
    if(v * CVector3<T>::Cross(p,a) > 0.0)
    {
      //if the origin is in the positive half-space of pvb
      if(v * CVector3<T>::Cross(p,b) < 0.0)
        c = p;
      else
        a = p;
    }
    //if the origin is in the negative half-space of pva
    else
    {
      if(v * CVector3<T>::Cross(p,c) > 0.0)
        b = p; //keep a, replace b
      else
        a = p; //keep b, replace a
    }
    n_old=n;
    iter++;
    if(iter==20)break;
  }//end while

}//end RefinePortal

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CIntersectorMPR<Real>;

//----------------------------------------------------------------------------

}



