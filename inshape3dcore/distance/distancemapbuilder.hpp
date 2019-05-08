
#ifndef DISTANCEMAPBUILDER_HPP
#define DISTANCEMAPBUILDER_HPP

//===================================================
//                     INCLUDES
//===================================================
#include <list>
#include <aabb3.h>
#include <meshobject.h>
#include <rigidbody.h>

namespace i3d {

template<typename T>
class DistanceMapBuilder
{
public:

    RigidBody *body_;

	DistanceMapBuilder(void) {

    }

	DistanceMapBuilder(RigidBody *body) : body_(body) {

    }

    virtual void buildDistanceMap();

    virtual ~DistanceMapBuilder(void) {

    }

private:

    void modifyBoundingBoxUniform(int axis, T amount) {

        if (axis == 0)
          _x += amount;
        else if(axis == 1)
          _y += amount;
        else if(axis == 2)
          _z += amount;
        else { }

    }

    void modifyBoundingBoxNonUniform(int axis, T amount) {

        if (amount <= 0) {
            // -x
            boxCenter.m_dCoords[axis] += amount; 
        } else {
            boxCenter.m_dCoords[axis] -= amount; 
        }

        if (axis == 0)
          _x += std::abs(amount);
        else if(axis == 1)
          _y += std::abs(amount);
        else if(axis == 2)
          _z += std::abs(amount);
        else { }

    }

    Vector3<Real> boxCenter;
    Real _x; 
    Real _y; 
    Real _z; 

};

}
#endif
