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

CContactGraph::CContactGraph() 
{

}

CContactGraph::~CContactGraph() 
{
  if(m_pEdges != NULL)
  {
    delete m_pEdges;
    m_pEdges = NULL;
  }
}

void CContactGraph::Update()
{

  CCollisionHash::iterator hiter = m_pEdges->begin();
  std::list<CCollisionInfo*> obsoletes;
  std::list<CCollisionInfo*>::iterator iter;
  for(;hiter!=m_pEdges->end();hiter++)
  {
    CCollisionInfo &collinfo = *hiter;
    if(collinfo.m_iState == CCollisionInfo::OBSOLETE)
    {
      obsoletes.push_back(&collinfo);
    }
  }

  for(iter=obsoletes.begin();iter!=obsoletes.end();iter++)
  {
    CCollisionInfo &collinfo = *(*iter);
    Remove(collinfo);
  }
  
  m_pEdges->Update();
}

void CContactGraph::ContactGroups(std::vector< i3d::CContactGroup >& groups)
{
  int igroup=0;
  CCollisionHash::iterator hiter = m_pEdges->begin();
  for(;hiter!=m_pEdges->end();hiter++)
  {
    CCollisionInfo &collinfo = *hiter;
    collinfo.m_iGroup = 0;
  }

  for(hiter=m_pEdges->begin();hiter!=m_pEdges->end();hiter++)
  {
    CCollisionInfo &collinfo = *hiter;
    
    if(collinfo.m_iState != CCollisionInfo::TOUCHING && collinfo.m_iState != CCollisionInfo::PERSISTENT_TOUCHING)
      continue;
    
    if(collinfo.m_iGroup == 0)
    {
      igroup++;
      groups.push_back(CContactGroup());
      CContactGroup &group = groups.back();
      group.m_iGroupId = igroup;
      TraverseGroup(igroup, collinfo, group);
    }
  }
  
  std::vector<CContactGroup>::iterator i = groups.begin();
  for(;i!=groups.end();i++)
  {
    CContactGroup &group = *i;
    std::list<CCollisionInfo*>::iterator j=group.m_pEdges.begin();
    for(;j!=group.m_pEdges.end();j++)
    {
      CCollisionInfo &edge = *(*j);
      if(edge.m_pBody1->m_iGroup != group.m_iGroupId)
      {
        edge.m_pBody1->m_iGroup=group.m_iGroupId;
        group.m_pBodies.push_back(edge.m_pBody1);
      }
      if(edge.m_pBody2->m_iGroup != group.m_iGroupId)
      {
        edge.m_pBody2->m_iGroup=group.m_iGroupId;
        group.m_pBodies.push_back(edge.m_pBody2);
      }      
    }
  }  
}

void CContactGraph::TraverseGroup(int iGroupId, i3d::CCollisionInfo& info, i3d::CContactGroup& group)
{
  std::list<CCollisionInfo *>::iterator i;
  CRigidBody *body;
  CRigidBody *pBody[2]={info.m_pBody1,info.m_pBody2};
  info.m_iGroup  = -1;

  for(int j=0;j<2;j++)
  {
    body=pBody[j];
    body->m_iGroup = -1;
    if(!body->m_bAffectedByGravity)
      continue;

    for(i=body->m_pEdges.begin();i!=body->m_pEdges.end();i++)
    {
      CCollisionInfo &edge = *(*i);
      
      if(edge.m_iState != CCollisionInfo::TOUCHING && edge.m_iState != CCollisionInfo::PERSISTENT_TOUCHING)
        continue;
      
      if(edge.m_iGroup == 0)
        TraverseGroup(iGroupId,edge,group);
    }

  }
  info.m_iGroup  = iGroupId;
  group.m_pEdges.push_back(&info);
  //printf("Adding edge (%i,%i), state: %i ...\n",info.iID1,info.iID2,info.m_iState);
}

void CContactGraph::ComputeStackLayers(i3d::CContactGroup& group)
{

  std::list<CRigidBody*>::iterator i=group.m_pBodies.begin();
  
  std::queue<CRigidBody*> q;
  
  //initialize the algorithm
  for(;i!=group.m_pBodies.end();i++)
  {
    CRigidBody *body = *i;
    if(!body->IsAffectedByGravity() && !body->m_bVisited)
    {
      body->m_iHeight=0;
      body->m_bVisited = true;
      q.push(body);
    }
    else
    {
      body->m_iHeight = CRigidBody::MAX_HEIGHT;
      body->m_bVisited = false;
    }
  }//end for
  
  group.m_iMaxHeight = 0;
  while(!q.empty())
  {
    CRigidBody *body0 = q.front();
    q.pop();
    
    std::list<CCollisionInfo*>::iterator j = body0->m_pEdges.begin();
    for(;j!=body0->m_pEdges.end();j++)
    {
      
      CCollisionInfo &info = *(*j);
      
      if(info.m_iState != CCollisionInfo::TOUCHING && info.m_iState != CCollisionInfo::PERSISTENT_TOUCHING)
        continue;
      
      CRigidBody *body1 = info.GetOther(body0);
      if(!body1->m_bVisited)
      {
        q.push(body1);
        body1->m_bVisited=true;
      }
      
      //assign the height of the body
      body1->m_iHeight = std::min<int>(body1->m_iHeight,body0->m_iHeight+1);
      
      //assign the layer of the edge
      if((body1->m_iHeight == body0->m_iHeight) && (body1->m_iHeight != 0))
      {
        info.m_iLayer = body1->m_iHeight-1;
        //printf("edge (%i,%i), height: %i ...\n",info.iID1,info.iID2,info.m_iLayer);
      }
      else
      {
        info.m_iLayer = std::min<int>(body0->m_iHeight,body1->m_iHeight);        
        //printf("edge (%i,%i), height: %i ...\n",info.iID1,info.iID2,info.m_iLayer);
      }
      group.m_iMaxHeight = std::max<int>(info.m_iLayer,group.m_iMaxHeight);
    }
    
  }//end while

  //build the stack layers by proccessing edges and bodies examining their stack height and layer indices
  group.m_iLayers =  group.m_iMaxHeight + 1;
  group.m_pLayers = new CStackLayer[group.m_iLayers];

  std::list<CCollisionInfo *>::iterator j = group.m_pEdges.begin();
  for(;j!=group.m_pEdges.end();j++)
  {
    CCollisionInfo &info = *(*j);
    group.m_pLayers[(*j)->m_iLayer].AddEdge((*j));
  }

  i=group.m_pBodies.begin();
  //initialize the algorithm
  for(;i!=group.m_pBodies.end();i++)
  {
    CRigidBody *body = *i;
    if(!body->IsAffectedByGravity())
    {
      group.m_pLayers[0].AddBody(body);
      continue;
    }

    bool upper=false;
    bool lower=false;
    std::list<CCollisionInfo *>::iterator k = body->m_pEdges.begin();
    for(;k!=body->m_pEdges.end();k++)
    {

      CCollisionInfo &edge = *(*k);
      if(edge.m_iState != CCollisionInfo::TOUCHING && edge.m_iState != CCollisionInfo::PERSISTENT_TOUCHING)
        continue;

      CRigidBody *body2 = (*k)->GetOther(body);
      
      if(body2->m_iHeight > body->m_iHeight)
      {
        upper = true;
      }
      
      if(body2->m_iHeight < body->m_iHeight)
      {
        lower = false;
      }

      if(lower && upper)
      {
        break;
      }
    }//end for

    if(upper) group.m_pLayers[body->m_iHeight].AddBody(body);

    if(lower) group.m_pLayers[body->m_iHeight-1].AddBody(body);
  }

}


}
