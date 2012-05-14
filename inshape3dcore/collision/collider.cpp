#include "collider.h"
#include <iostream>

namespace i3d {

CCollider::CCollider(void)
{
  m_pGenerator = NULL;
}

CCollider::~CCollider(void)
{
  if(m_pGenerator!=NULL)
  {
    delete m_pGenerator;
    m_pGenerator=NULL;
  }
}

void CCollider::Collide(std::vector<CContact> &vContacts)
{

}

}