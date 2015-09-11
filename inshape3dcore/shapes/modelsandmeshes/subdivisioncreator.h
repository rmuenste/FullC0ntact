#ifndef SUBDIVISIONCREATOR_H
#define SUBDIVISIONCREATOR_H

#include <vector>
#include <aabb3.h>
#include <triangle3.h>

namespace i3d {

//Forward declaration
template<class BV, class T, class Traits> class CBoundingVolumeNode3;
class CTraits;

template <typename BV, typename T, typename Traits, typename SD>
class CBoundingVolumeTree3;


/**
 * @brief The class contains raw triangle data and user-defined parameters for BVH-creation
 * 
 * The class CSubDivRessources contains raw triangle mesh data and user-defined
 * parameters to set up a bounding volume hierarchy for the mesh data.
 */
class CSubDivRessources
{
public:
	CSubDivRessources(int iBranch0, int iDepth, int iID, AABB3r box, std::vector<Triangle3r> *pTriangles)
	{

		m_iBranchLevel0 = iBranch0;

		m_iID = iID;

		m_pTriangles = pTriangles;

		m_iDepth = iDepth;

	};
	~CSubDivRessources(){};

	int m_iBranchLevel0;

	int m_iID;

	int m_iDepth;

	AABB3r box;

	std::vector<Triangle3r> *m_pTriangles;

};


/**
 * @brief Creates a bounding volume hierarchy from a triangle mesh
 */
class CSubdivisionCreator
{
public:
	CSubdivisionCreator();
	CSubdivisionCreator(CSubDivRessources *Ressources) {m_pRessources=Ressources;};
	CSubdivisionCreator(const CSubdivisionCreator &copy);
	~CSubdivisionCreator();
	void Subdivide(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes);
  void Subdivide(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes, CBoundingVolumeTree3<AABB3r, Real, CTraits, CSubdivisionCreator> &tree);
	void SubdivideBox(CBoundingVolumeNode3<AABB3r,Real,CTraits> **&pNodes);
	void SubdivideNode(CBoundingVolumeNode3<AABB3r,Real,CTraits> *&pNode);
	bool EvaluateTerminationCrit(CBoundingVolumeNode3<AABB3r,Real,CTraits> *pNode, int iDepth);
	CSubDivRessources *m_pRessources;

private: 
  void ApproxUpperBound(CBoundingVolumeNode3<AABB3r,Real,CTraits> *&pNode);

  int inode_;

};

}

#endif
