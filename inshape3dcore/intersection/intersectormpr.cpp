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

  GenerateContactPoints();

  return intersection;

}//end Intersection

template<class T>
void CIntersectorMPR<T>::FindInitialPortal()
{

  //get the origin of the minkowski difference shape0.center - shape1.center
  m_Portal.m_Points0[0] = m_pShape0->GetCenter();
  m_Portal.m_Points1[0] = m_pShape1->GetCenter();
  m_Portal.v() = m_Portal.m_Points0[0] - m_Portal.m_Points1[0];

  //get the support point in the direction of -v
  m_Portal.m_Points0[1] = m_pShape0->GetSupport(-m_Portal.v());
  m_Portal.m_Points1[1] = m_pShape1->GetSupport(m_Portal.v());
  std::cout<<m_Portal.m_Points0[1] - m_Portal.m_Points1[1];
  m_Portal.a() = m_Portal.m_Points0[1] - m_Portal.m_Points1[1];

  //check for collinearity

  CVector3<T> va = CVector3<T>::Cross(m_Portal.v(),m_Portal.a());
  m_Portal.m_Points0[2] = m_pShape0->GetSupport(va);
  m_Portal.m_Points1[2] = m_pShape1->GetSupport(-va);
  m_Portal.b() = m_Portal.m_Points0[2] - m_Portal.m_Points1[2];

  CVector3<T> avbv = CVector3<T>::Cross(m_Portal.a()-m_Portal.v(),m_Portal.b()-m_Portal.v());
  m_Portal.m_Points0[3] = m_pShape0->GetSupport(avbv);
  m_Portal.m_Points1[3] = m_pShape1->GetSupport(-avbv);
  m_Portal.c() = m_Portal.m_Points0[3] - m_Portal.m_Points1[3];

  std::vector< CVector3<T> > verts;
  verts.push_back(m_Portal.v());
  verts.push_back(m_Portal.a());
  verts.push_back(m_Portal.b());
  verts.push_back(m_Portal.c());  
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
  CVector3<T> r = -m_Portal.v();
  int iter=0;
  bool stop;
  do {

    stop = true;
    //compute the triangle normals fo the three other triangles
    CVector3<T> nvab = CVector3<T>::Cross((m_Portal.a()-m_Portal.v()),(m_Portal.b()-m_Portal.v()));
    CVector3<T> nvbc = CVector3<T>::Cross((m_Portal.b()-m_Portal.v()),(m_Portal.c()-m_Portal.v()));
    CVector3<T> nvca = CVector3<T>::Cross((m_Portal.c()-m_Portal.v()),(m_Portal.a()-m_Portal.v()));

    //check the direction of the normals
    if(r*nvab > 0)
    {
      //swap a and b to invert the normal
      m_Portal.Swap(1,2);

      //we can replace the third point by one that is closer to the origin
      m_Portal.GenerateNewSupport(m_pShape0->GetSupport(nvab), m_pShape1->GetSupport(-nvab));
      m_Portal.ReplaceCByNewSupport();
      stop = false;
    }
    else if(r*nvbc > 0)
    {
      //swap b and c to invert the normal
      m_Portal.Swap(2,3);

      //we can replace the third point by one that is closer to the origin
      m_Portal.GenerateNewSupport(m_pShape0->GetSupport(nvbc), m_pShape1->GetSupport(-nvbc));
      m_Portal.ReplaceAByNewSupport();
      stop = false;
    }
    else if(r*nvca > 0)
    {
      //swap a and c to invert the normal
      m_Portal.Swap(1,3);

      //we can replace the third point by one that is closer to the origin
      m_Portal.GenerateNewSupport(m_pShape0->GetSupport(nvca), m_pShape1->GetSupport(-nvca));
      m_Portal.ReplaceBByNewSupport();
      stop = false;
    }

    std::vector< CVector3<T> > verts;
    verts.push_back(m_Portal.v());
    verts.push_back(m_Portal.a());
    verts.push_back(m_Portal.b());
    verts.push_back(m_Portal.c());  
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
  while(true) {

    //compute current normal
    m_Portal.n = CVector3<T>::Cross((m_Portal.c()-m_Portal.a()),(m_Portal.b()-m_Portal.a()));
    m_Portal.n.Normalize();

    std::cout<<"n * n_old = "<<m_Portal.n*m_Portal.n_old<<" iter="<<iter<<std::endl;
    //check convergence criterion
    if(m_Portal.n*m_Portal.n_old > 1 - CMath<T>::EPSILON5)
    {
      return;
    }

    std::vector< CVector3<T> > verts;
    verts.push_back(m_Portal.v());
    verts.push_back(m_Portal.a());
    verts.push_back(m_Portal.b());
    verts.push_back(m_Portal.c());  
    verts.push_back(CVector3<T>(0,0,0));  
    CVector3<T> mid=(1.0/3.0)*(m_Portal.a()+m_Portal.b()+m_Portal.c());
    verts.push_back(mid);  

    CVector3<T> nnorm=m_Portal.n;
    nnorm/=nnorm.mag();
    verts.push_back(mid+m_Portal.v().mag()*nnorm);  

    std::string sFileName("output/MPR_refine");
    std::ostringstream sName;
    sName<<"."<<std::setfill('0')<<std::setw(3)<<iter<<".vtk";
    sFileName.append(sName.str());
    
    CVtkWriter writer;
    writer.WriteMPR(verts, 0, sFileName.c_str());

    //if (a-0)*n > 0 then the origin is inside the tetrahedron
    if(m_Portal.a()*m_Portal.n > 0.0)
    {
      //intersection
      intersection = true;
      std::cout<<"Intersection=true"<<std::endl;
      //return;
    }

    //
    if((m_Portal.a()*m_Portal.n < CMath<T>::EPSILON3) && (m_Portal.a()*m_Portal.n > -CMath<T>::EPSILON3))
    {
      intersection = true;
      std::cout<<"Origin in Portal plane"<<std::endl;
      //return;
    }

    //get the support point in the direction of n
    m_Portal.GenerateNewSupport(m_pShape0->GetSupport(m_Portal.n), m_pShape1->GetSupport(-m_Portal.n));
  
    //origin is outside
    if(m_Portal.p() * m_Portal.n < -CMath<T>::EPSILON4)
    {
      intersection = false;
      std::cout<<"Origin outside... p*n: "<<m_Portal.p()*m_Portal.n<<std::endl;
      return;
    }

    //if the origin is in the positive half-space of pva
    if(m_Portal.v() * CVector3<T>::Cross(m_Portal.p(),m_Portal.a()) > 0.0)
    {
      //if the origin is in the negative half-space of pvb
      if(m_Portal.v() * CVector3<T>::Cross(m_Portal.p(),m_Portal.b()) < 0.0)
        m_Portal.ReplaceCByNewSupport();//keep a, replace c
      else
        m_Portal.ReplaceAByNewSupport();//keep c, replace a
    }
    //if the origin is in the negative half-space of pva
    else
    {
      if(m_Portal.v() * CVector3<T>::Cross(m_Portal.p(),m_Portal.c()) > 0.0)
        m_Portal.ReplaceBByNewSupport();//keep a, replace b
      else
        m_Portal.ReplaceAByNewSupport();//keep b, replace a
    }

    m_Portal.n_old=m_Portal.n;
    iter++;
    if(iter==50)break;
  }//end while

}//end RefinePortal

  template <class T>
void CIntersectorMPR<T>::GenerateContactPoints()
{

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CIntersectorMPR<Real>;

//----------------------------------------------------------------------------

}



