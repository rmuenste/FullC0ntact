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


//===================================================
//                     INCLUDES
//===================================================


#include "contactgraph.h"
#include <queue>
#include <stdio.h>

namespace i3d {

ContactGraph::ContactGraph() 
{

}

ContactGraph::~ContactGraph() 
{
  if(edges_ != NULL)
  {
    delete edges_;
    edges_ = NULL;
  }
}

void ContactGraph::update()
{

  CollisionHash::iterator hiter = edges_->begin();
  std::list<CollisionInfo*> obsoletes;
  std::list<CollisionInfo*>::iterator iter;
  for(;hiter!=edges_->end();hiter++)
  {
    CollisionInfo &collinfo = *hiter;
    if(collinfo.m_iState == CollisionInfo::OBSOLETE)
    {
      obsoletes.push_back(&collinfo);
    }
  }

  for(iter=obsoletes.begin();iter!=obsoletes.end();iter++)
  {
    CollisionInfo &collinfo = *(*iter);
    removeEdge(collinfo);
  }
  
  edges_->update();
}

void ContactGraph::contactGroups(std::vector< i3d::ContactGroup >& groups)
{
  int igroup=0;
  CollisionHash::iterator hiter = edges_->begin();
  for(;hiter!=edges_->end();hiter++)
  {
    CollisionInfo &collinfo = *hiter;
    collinfo.m_iGroup = 0;
  }

  for(hiter=edges_->begin();hiter!=edges_->end();hiter++)
  {
    CollisionInfo &collinfo = *hiter;
    
    if(collinfo.m_iState != CollisionInfo::TOUCHING && collinfo.m_iState != CollisionInfo::PERSISTENT_TOUCHING)
      continue;
    
    if(collinfo.m_iGroup == 0)
    {
      igroup++;
      groups.push_back(ContactGroup());
      ContactGroup &group = groups.back();
      group.m_iGroupId = igroup;
      traverseGroup(igroup, collinfo, group);
    }
  }
  
  std::vector<ContactGroup>::iterator i = groups.begin();
  for(;i!=groups.end();i++)
  {
    ContactGroup &group = *i;
    std::list<CollisionInfo*>::iterator j=group.m_pEdges.begin();
    for(;j!=group.m_pEdges.end();j++)
    {
      CollisionInfo &edge = *(*j);
      if(edge.m_pBody0->group_ != group.m_iGroupId)
      {
        edge.m_pBody0->group_=group.m_iGroupId;
        group.m_pBodies.push_back(edge.m_pBody0);
      }
      if(edge.m_pBody1->group_ != group.m_iGroupId)
      {
        edge.m_pBody1->group_=group.m_iGroupId;
        group.m_pBodies.push_back(edge.m_pBody1);
      }      
    }
  }  
}

void ContactGraph::traverseGroup(int groupId, i3d::CollisionInfo& info, i3d::ContactGroup& group)
{
  std::list<CollisionInfo *>::iterator i;
  RigidBody *body;
  RigidBody *pBody[2]={info.m_pBody0,info.m_pBody1};
  info.m_iGroup  = -1;

  for(int j=0;j<2;j++)
  {
    body=pBody[j];
    body->group_ = -1;
    if(!body->affectedByGravity_)
      continue;

    for(i=body->getEdges().begin();i!=body->getEdges().end();i++)
    {
      CollisionInfo &edge = *(*i);
      
      if(edge.m_iState != CollisionInfo::TOUCHING && edge.m_iState != CollisionInfo::PERSISTENT_TOUCHING)
        continue;
      
      if(edge.m_iGroup == 0)
        traverseGroup(groupId,edge,group);
    }

  }
  info.m_iGroup  = groupId;
  group.m_pEdges.push_back(&info);
  //printf("Adding edge (%i,%i), state: %i ...\n",info.iID1,info.iID2,info.m_iState);
}

void ContactGraph::computeStackLayers(i3d::ContactGroup& group)
{

  std::list<RigidBody*>::iterator i=group.m_pBodies.begin();
  
  std::queue<RigidBody*> q;
  
  //initialize the algorithm
  for(;i!=group.m_pBodies.end();i++)
  {
    RigidBody *body = *i;
    if(!body->isAffectedByGravity() && !body->visited_)
    {
      body->height_=0;
      body->visited_ = true;
      q.push(body);
    }
    else
    {
      body->height_ = RigidBody::MAX_HEIGHT;
      body->visited_ = false;
    }
  }//end for
  
  group.m_iMaxHeight = 0;
  while(!q.empty())
  {
    RigidBody *body0 = q.front();
    q.pop();
    
    std::list<CollisionInfo*>::iterator j = body0->getEdges().begin();
    for(;j!=body0->getEdges().end();j++)
    {
      
      CollisionInfo &info = *(*j);
      
      if(info.m_iState != CollisionInfo::TOUCHING && info.m_iState != CollisionInfo::PERSISTENT_TOUCHING)
        continue;
      
      RigidBody *body1 = info.GetOther(body0);
      if(!body1->visited_)
      {
        q.push(body1);
        body1->visited_=true;
      }
      
      //assign the height of the body
      body1->height_ = std::min<int>(body1->height_,body0->height_+1);
      
      //assign the layer of the edge
      if((body1->height_ == body0->height_) && (body1->height_ != 0))
      {
        info.m_iLayer = body1->height_-1;
        //printf("edge (%i,%i), height: %i ...\n",info.iID1,info.iID2,info.m_iLayer);
      }
      else
      {
        info.m_iLayer = std::min<int>(body0->height_,body1->height_);        
        //printf("edge (%i,%i), height: %i ...\n",info.iID1,info.iID2,info.m_iLayer);
      }
      group.m_iMaxHeight = std::max<int>(info.m_iLayer,group.m_iMaxHeight);
    }
    
  }//end while

  //build the stack layers by proccessing edges and bodies examining their stack height and layer indices
  group.m_iLayers =  group.m_iMaxHeight + 1;
  group.m_pLayers = new StackLayer[group.m_iLayers];

  std::list<CollisionInfo *>::iterator j = group.m_pEdges.begin();
  for(;j!=group.m_pEdges.end();j++)
  {
    CollisionInfo &info = *(*j);
    group.m_pLayers[(*j)->m_iLayer].AddEdge((*j));
  }

  i=group.m_pBodies.begin();
  //initialize the algorithm
  for(;i!=group.m_pBodies.end();i++)
  {
    RigidBody *body = *i;
    if(!body->isAffectedByGravity())
    {
      group.m_pLayers[0].AddBody(body);
      continue;
    }

    bool upper=false;
    bool lower=false;
    std::list<CollisionInfo *>::iterator k = body->getEdges().begin();
    for(;k!=body->getEdges().end();k++)
    {

      CollisionInfo &edge = *(*k);
      if(edge.m_iState != CollisionInfo::TOUCHING && edge.m_iState != CollisionInfo::PERSISTENT_TOUCHING)
        continue;

      RigidBody *body2 = (*k)->GetOther(body);
      
      if(body2->height_ > body->height_)
      {
        upper = true;
      }
      
      if(body2->height_ < body->height_)
      {
        lower = false;
      }

      if(lower && upper)
      {
        break;
      }
    }//end for

    if(upper) group.m_pLayers[body->height_].AddBody(body);

    if(lower) group.m_pLayers[body->height_-1].AddBody(body);
  }

}


}
