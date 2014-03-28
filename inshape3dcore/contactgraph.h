/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/ 
#ifndef CONTACTGRAPH_H
#define CONTACTGRAPH_H



//===================================================
//                     INCLUDES
//===================================================
#include <collisionhash.h>
#include <contactgroup.h>

namespace i3d {

/**
 * @brief A graph, whose vertices are the rigid bodies in the simulation and the edges are the CCollisionInfos 
 *  
 */
  
class ContactGraph {

public: 

  ContactGraph(); 

  ~ContactGraph(); 

  bool find();

  void insertEdge(CollisionInfo &info)
  {
    edges_->Insert(info);
  };
  
  void removeEdge(CollisionInfo &info)
  {
    edges_->Remove(info);
  };

  void setEdgeProperty();

  void clear()
  {
    edges_->Clear();
  };

  void update();

  void contactGroups(std::vector<ContactGroup> &groups);
  
  void computeStackLayers(ContactGroup &group);



  CollisionHash *edges_;

  //Hashtable<Nodes> m_htNodes;
  //Hashtable<Edges> m_htEdges;
private:

  void traverseGroup(int iGroupId, CollisionInfo &info, ContactGroup &group);

};

}
#endif
