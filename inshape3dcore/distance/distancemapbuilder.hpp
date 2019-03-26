
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

};

}
#endif
