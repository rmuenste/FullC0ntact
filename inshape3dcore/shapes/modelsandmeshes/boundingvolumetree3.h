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

#ifndef BOUNDINGVOLUMETREE3_H
#define BOUNDINGVOLUMETREE3_H

///==============================================
///                  INCLUDES
///==============================================

#include <vector3.h>
#include <vector>
#include <queue>
#include <iostream>
#include <traits.h>

namespace i3d {

///==============================================
///	          Forward Declarations
///==============================================

template<class BV, class T, class Traits, class SD>
class CBoundingVolumeTree3;

///==============================================
///	      CLASS CBoundingVolumeNode3
///==============================================

/**
* @brief A node in a bounding volume hierarchy
*
*
*/

template<class BV, class T, class Traits>
class CBoundingVolumeNode3
{
public:

	//standard constructor
	CBoundingVolumeNode3();

	//copy constructor
	CBoundingVolumeNode3(const BV &rBV);

	//deconstructor
	~CBoundingVolumeNode3();

	//member functions

	//Is this node a leaf?
	inline bool IsLeaf() const {return m_Children[0] == NULL;};

	//returns the center of the node

	//returns the lower bound for the branch and bound calculation
	inline T GetLowerBound(const CVector3<T> &vQuery) {return m_BV.minDistance(vQuery);};

	//returns the upper bound for the branch and bound calculation
	inline T GetUpperBound(const CVector3<T> &vQuery) {return m_BV.maxDistance(vQuery);};

	//returns the squared lower bound for the branch and bound calculation
	inline T GetLowerBoundSqr(const CVector3<T> &vQuery) {return m_BV.minDistanceSqr(vQuery);};

	//returns the squared upper bound for the branch and bound calculation
	inline T GetUpperBoundSqr(const CVector3<T> &vQuery) {return m_BV.maxDistanceSqr(vQuery);};

	//translate the node by the vector vTranslation
	inline void Translate(const CVector3<T> &vTranslation) {m_BV.Translate(vTranslation);};

	//rotate the node by the angle nAngle
	inline void Rotate(T nAngle) {m_BV.Rotate(nAngle);};

	//returns the center of the bounding volume
	inline CVector3<T> GetCenter() {return m_BV.getCenter();};

	//returns the depth of the GetSubTreeDepth with root pNode
	static int GetSubTreeDepth(CBoundingVolumeNode3<BV,T,Traits> *pNode, int iDepth)
	{
		int dR = 0;
		int dL = 0;
		int iReturn = 0;

		if(!pNode->IsLeaf())
		{
			dL = GetSubTreeDepth(pNode->m_Children[0],iDepth+1);
			dR = GetSubTreeDepth(pNode->m_Children[1],iDepth+1);
			iReturn = ( dL >= dR ) ? dL : dR;
			return iReturn;
		}//end if
		else
		{
			iReturn = iDepth;
			return iReturn;
		}
		return iReturn;
	}

	//public member variables

	//pointer to the children of this node
	CBoundingVolumeNode3<BV,T,Traits> *m_Children[2];

	//friends
	template <class,class,class,class> friend class CBoundingVolumeTree3;

	//the node's bounding volume
	BV m_BV;

	//Traits of the node
	Traits m_Traits;

private:

};


///==============================================
///	      CLASS CBoundingVolumeTree3
///==============================================

/**
* @brief A bounding volume hierarchy
*
* A bounding volume hierarchy that can use various shapes
* as bounding volumes
*
*/
template<class BV, class T, class Traits, class SD>
class CBoundingVolumeTree3
{
public:

	//constructor
	CBoundingVolumeTree3();
	CBoundingVolumeTree3(SD *pSD) : m_iNumChildren(0)
	{
		m_pSD=pSD;
	};
	~CBoundingVolumeTree3();

	//member functions
	void BreadthFirstSearch();
	
	//initializes the tree
	void InitTree();
  void InitBoxTree(SD *pSD);
	void InitTree(SD *pSD);

	//deallocates all nodes etc...
	void DestroyTree();

	//updates the nodes after a translation
	void UpdateTree();
	
	//return a vector containing the nodes at the specified level
	std::vector<CBoundingVolumeNode3<BV,T,Traits> *> GetNodesLevel(int iLevel);

	//returns the root
	inline CBoundingVolumeNode3<BV,T,Traits>* GetChild(int i){return m_Children[i];};
	inline const CBoundingVolumeNode3<BV,T,Traits>* GetChild(int i) const {return m_Children[i];};

	void GenTreeStatistics();
	
	void DestroyAndRebuilt(SD *pSD);

	//calculates the Depth of the tree
	int GetDepth();
	
	//returns the number of children
	inline int GetNumChildren()
	{
		return m_iNumChildren;
	}//end GetNumChildren

	//Set the number of children
	inline void SetNumChildren(int i)
	{
		m_iNumChildren = i;
	}//end GetNumChildren

	SD *m_pSD;

private:

	//private member functions

	//calculates the depth of Subtree with node pNode
	int CalcSubTreeDepth(CBoundingVolumeNode3<BV,T,Traits> *pNode, int iDepth);

	//deletes the subtree with root pNode
	void DeleteSubTree(CBoundingVolumeNode3<BV,T,Traits> *pNode);

	//copies the subtree with node pNode
	void CopySubTree(CBoundingVolumeNode3<BV,T,Traits> *pNode);

	//private member variables
	CBoundingVolumeNode3<BV,T,Traits> **m_Children;

	//number of child nodes
	int m_iNumChildren;

};

#include "boundingvolumetree3.cpp"

}

#endif
