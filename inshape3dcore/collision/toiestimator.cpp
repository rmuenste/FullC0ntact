#include "toiestimator.h"
#include <world.h>
#include <limits>
#include <iostream>
#include <mymath.h>

CTOIEstimator::CTOIEstimator(void)
{
}

CTOIEstimator::~CTOIEstimator(void)
{
}

CTOIEstimator::CTOIEstimator(CWorld * pDomain, std::priority_queue<CCollisionInfo,std::vector<CCollisionInfo> , CmpInfos> *pCollisionHeap)
{
	m_pWorld=pDomain;
	m_pCollisionHeap = pCollisionHeap;
}

Real CTOIEstimator::CalcLowerBound(const VECTOR3 &p0, const VECTOR3 &p1, const VECTOR3 &v0, const VECTOR3 &v1, Real rad0, Real rad1,Real &dist)
{

	//we get the task (i,j)
	//calculate the distance
	dist = (p1 - p0).mag();
	dist = dist - rad0 - rad1;
	//calculate t_l = distance/(|v(t_0)|)
	Real normVel0 = (v1 - v0).mag();

	if(normVel0 > CMath<Real>::EPSILON5)
	{
		return (dist/normVel0);
	}
	else
	{
		return std::numeric_limits<Real>::max();
	}

}

Real CTOIEstimator::CalcRootDist(const VECTOR3 &p0, const VECTOR3 &p1, const VECTOR3 &v0, const VECTOR3 &v1, Real rad0, Real rad1)
{

	VECTOR3 p01 = p0 - p1;
	VECTOR3 v01 = v0 - v1;

	Real t0 = std::numeric_limits<Real>::max();
	Real t1 = std::numeric_limits<Real>::max();

	Real r2 = rad0 + rad1;
	r2 *= r2;

	Real a = v01 * v01;

	Real b = 2.0 * (p01 * v01);

	Real c = (p01 * p01) - r2;

	Real disc = (b*b) - 4.0 * a * c;

	//if disc > 0 a collision will occur that has
	//an entry time and a exit
	if(disc > 0)
	{
		t0 = (-b + sqrt(disc))/(2.0*a);
		t1 = (-b - sqrt(disc))/(2.0*a);
		//the t can be < 0, then collision has already occured
		if((t0 >= 0) && (t1 >= 0))
		{
			return std::min(t0,t1);
		}
		else if(t0 >= 0)
		{
			return t0;
		}
		else if(t1 >= 0)
		{
			return t1;
		}
		else
		{
			return std::numeric_limits<Real>::max();
		}
	}
	//if disc == 0 then the spheres will touch in one point
	else if((disc < CMath<Real>::EPSILON7) && (disc > 0))
	{
		t0=t1=-b/(2.0*a);
		return t0;
	}
	//else the spheres will not touch at all
	else
	{
		return std::numeric_limits<Real>::max();
	}

	return 0;
}

// the function initializes the collision heap and computes the TOIs for all pairs
void CTOIEstimator::BuildHeap(void)
{
	//for all pairs in the domain
	std::vector< C3DModel >::iterator first;
	std::vector< C3DModel >::iterator second;
	int i,j;
	i=j=0;
	//Check every pair
	for(first=m_pWorld->m_vParticles.begin();first!=m_pWorld->m_vParticles.end();first++)
	{
		C3DModel &p1 = *first;
		j=i+1;
		for(second=first+1;second!=m_pWorld->m_vParticles.end();second++)
		{

			C3DModel &p2 = *second;

			//the distance
			Real dist=0;

			//the lower bound on collision time
			//Real t_l = CalcLowerBound(p1.m_vMeshes[0].m_vOrigin,
			//	p2.m_vMeshes[0].m_vOrigin,
			//	m_pWorld->m_pParams->m_vVelocities[i],
			//	m_pWorld->m_pParams->m_vVelocities[j],p1.m_dRadius,p2.m_dRadius,dist);

			//calculate the roots of the distance function
			Real t_l = CalcRootDist(p1.m_vMeshes[0].m_vOrigin,
				p2.m_vMeshes[0].m_vOrigin,
				m_pWorld->m_pParams->m_vVelocities[i],
				m_pWorld->m_pParams->m_vVelocities[j],p1.m_dRadius,p2.m_dRadius);


			//now build a CollisionInfo
			CCollisionInfo Info(&p1,&p2,dist,i,j);

			//before we push it onto the heap add set some additional properties
			Info.m_TOI = m_pWorld->m_pTimeControl->GetTime() + t_l;
			
			//save what kind of collision we have
			Info.m_iCollisionID = CCollisionInfo::SPHERE_SPHERE;

			//distance between the collision objects
			Info.m_dDistance = dist;
			
			//push onto the heap
			m_pCollisionHeap->push(Info);

			//advance the particle id counter
			j++;
		}//end for j
		
		//now check for all walls
		for(int k=0;k<6;k++)
		{
			//calculate the distance
			int indexOrigin = k/2;
			Real position = p1.m_vMeshes[0].m_vOrigin.m_dCoords[indexOrigin];
			Real dist = fabs(m_pWorld->m_pBoundary->m_Values[k] - position)-p1.m_dRadius;

			Real relVel = -(m_pWorld->m_pParams->m_vVelocities[i] * m_pWorld->m_pBoundary->m_vNormals[k]);
			Real relG = -(m_pWorld->m_pParams->m_vGrav * m_pWorld->m_pBoundary->m_vNormals[k]);

			Real t_l;
			if(relG == 0.0)
				//solve linear
				t_l = CalcConstVelocity(relVel,dist);
			else
			{
				//now we can have either :
				if((relG > 0.0) && (relVel < 0.0))
					t_l = CalcRiseFall(relG,relVel,dist);
				else if((relG < 0.0) && (relVel > 0.0))
					t_l = CalcRise(relG,relVel,dist);
				else
					t_l = CalcFreeFall(relG,relVel,dist);
			}

			//now build a CollisionInfo
			CCollisionInfo Info(&p1,NULL,dist,i,k);

			//before we push it onto the heap add set some additional properties
			Info.m_TOI = m_pWorld->m_pTimeControl->GetTime() + t_l;
			
			//save what kind of collision we have
			Info.m_iCollisionID = CCollisionInfo::SPHERE_WALL;

			//distance between the collision objects
			Info.m_dDistance = dist;

			//push onto the heap
			m_pCollisionHeap->push(Info);

		}//end for all walls

		i++;
	}//end for i
}

Real CTOIEstimator::CalcFreeFall(Real dAcceleration, Real dVelocity, Real dDistance)
{
	Real x1,x2,t;
	Real dist2 = 2.0 * dDistance;
	Real v0 = dVelocity;
	Real p = v0/dAcceleration;
	Real q = -dist2/dAcceleration;

	Real p2 = p * p;
	if((p2 - q) >= 0)
	{
		x1 = -p + sqrt(p2 - q);
		x2 = -p - sqrt(p2 - q);
		//both greater 0
		if(x1 > 0.0 && x2 > 0.0)
		{
			if(x1 < x2)
			{
				return x1;
			}
			else
			{
				return x2;
			}
		}
		if(x1 > 0)
			return x1;
		else if(x2 > 0)
			return x2;
		else
			return std::numeric_limits<Real>::max();
	}
	//cannot compute the square root return MAXREAL
	else
	{
		return std::numeric_limits<Real>::max();
	}
}

Real CTOIEstimator::CalcConstVelocity(Real dVelocity, Real dDistance)
{
	Real t;

	if(dVelocity > CMath<Real>::EPSILON5)
		t = dDistance/dVelocity;
	else
		t = std::numeric_limits<Real>::max();
	
	return t;
}

Real CTOIEstimator::CalcRiseFall(Real dAcceleration, Real dVelocity, Real dDistance)
{
	Real riseHeight;
	Real t_rise;
	Real t;
	t_rise = -dVelocity/dAcceleration;

	riseHeight = -dVelocity * t_rise - dAcceleration/2.0  * t_rise * t_rise;

	//and now for a fall of rise_height + distance
	t = CalcFreeFall(dAcceleration, 0.0, riseHeight+dDistance);

	t += t_rise;

	return t;
}

Real CTOIEstimator::CalcRise(Real dAcceleration, Real dVelocity, Real dDistance)
{
	Real t;
	t = std::numeric_limits<Real>::max();
	return t;
}

// the function updates the collision heap after a collision changed the velocities of the particles
void CTOIEstimator::UpdateHeap(void)
{
	while(!m_pCollisionHeap->empty())
	{
		m_pCollisionHeap->pop();
	}
	BuildHeap();
}
