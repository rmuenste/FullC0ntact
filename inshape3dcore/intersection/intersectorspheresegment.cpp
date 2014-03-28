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

#include "intersectorspheresegment.h"

namespace i3d {
  
template <class T>
bool IntersectorSphereSegment<T>::intersection()
{
  CVector3<T> diff = segment_->center_ - sphere_->center_;

  if((diff * diff < CMath<T>::TOLERANCEZERO) && (fabs(segment_->ext_ - sphere_->radius_) < CMath<T>::TOLERANCEZERO))
  {
    points_[0] = segment_->center_ + segment_->ext_ * segment_->dir_;
    points_[1] = segment_->center_ - segment_->ext_ * segment_->dir_;
    numIntersections_ = 2;
    intersectionType_ = SEGMENT;
    return true;
  }

  T a0 = diff * diff - sphere_->radius_ * sphere_->radius_;
  T a1 = segment_->dir_ * diff;
  T discr = a1*a1 - a0;
  if (discr < (T)0)
  {
      numIntersections_ = 0;
      return false;
  }

  T tmp0 = segment_->ext_*segment_->ext_ + a0;
  T tmp1 = ((T)2)*a1*segment_->ext_;
  T qm = tmp0 - tmp1;
  T qp = tmp0 + tmp1;
  T root;
  if (qm*qp <= (T)0)
  {
      root = sqrt(discr);
      segmentParameters_[0] = (qm > (T)0 ? -a1 - root : -a1 + root);
      points_[0] = segment_->center_ + segmentParameters_[0] * segment_->dir_;
      numIntersections_ = 1;
      intersectionType_ = POINT;
      return true;
  }

  if (qm > (T)0 && fabs(a1) < segment_->ext_)
  {
    if (discr >= CMath<T>::TOLERANCEZERO)
      {
          root = sqrt(discr);
          segmentParameters_[0] = -a1 - root;
          segmentParameters_[1] = -a1 + root;
          points_[0] = segment_->center_ + segmentParameters_[0] *
              segment_->dir_;
          points_[1] = segment_->center_ + segmentParameters_[1] *
              segment_->dir_;
          numIntersections_ = 2;
          intersectionType_ = SEGMENT;
      }
      else
      {
          segmentParameters_[0] = -a1;
          points_[0] = segment_->center_ + segmentParameters_[0] *
              segment_->dir_;
          numIntersections_ = 1;
          intersectionType_ = POINT;
      }
  }
  else
  {
      numIntersections_ = 0;
      intersectionType_ = EMPTY;
  }

  return numIntersections_ > 0;
}
  
//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class IntersectorSphereSegment<Real>;

//----------------------------------------------------------------------------
}
