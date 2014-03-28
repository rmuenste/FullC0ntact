#include "collider.h"
#include <iostream>

namespace i3d {

Collider::Collider(void)
{
  generator_ = NULL;
}

Collider::~Collider(void)
{
  if(generator_!=NULL)
  {
    delete generator_;
    generator_=NULL;
  }
}

void Collider::collide(std::vector<Contact> &vContacts)
{

}

}