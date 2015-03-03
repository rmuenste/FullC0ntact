#ifndef COLLIDER2COMPOUND_H
#define COLLIDER2COMPOUND_H



//===================================================
//                     INCLUDES
//===================================================

#include "collider.h"

namespace i3d {

	class Collider2Compound : public Collider {

	public:

	  Collider2Compound();

		~Collider2Compound();


	void collide(std::vector<Contact> &vContacts);
	};



}
#endif
