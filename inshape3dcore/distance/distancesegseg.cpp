#include "distancesegseg.h"

namespace i3d {

template <typename T>
CDistanceSegSeg<T>::CDistanceSegSeg(void)
{

}

template <typename T>
CDistanceSegSeg<T>::~CDistanceSegSeg(void)
{

}

template <typename T>
CDistanceSegSeg<T>::CDistanceSegSeg(const Segment3<T> &Seg0, const Segment3<T> &Seg1) : m_Seg0(Seg0), m_Seg1(Seg1)
{

}

template <typename T>
T CDistanceSegSeg<T>::ComputeDistance()
{
	return sqrt(ComputeDistanceSqr());
}

template <typename T>
T CDistanceSegSeg<T>::ComputeDistanceSqr()
{

	CVector3<T> diff = m_Seg0.center_ - m_Seg1.center_;
	T a01 = -m_Seg0.dir_*m_Seg1.dir_;
	T b0 = diff*m_Seg0.dir_;
	T b1 = -diff * m_Seg1.dir_;
	T c = diff * diff;
	T det = fabs((T)1 - a01*a01);
	T s0, s1, sqrDist, extDet0, extDet1, tmpS0, tmpS1;

	if (det >= E5)
	{
		// Segments are not parallel.
		s0 = a01*b1 - b0;
		s1 = a01*b0 - b1;
		extDet0 = m_Seg0.ext_*det;
		extDet1 = m_Seg1.ext_*det;

		if (s0 >= -extDet0)
		{
			if (s0 <= extDet0)
			{
					if (s1 >= -extDet1)
					{
							if (s1 <= extDet1)  // region 0 (interior)
							{
									// Minimum at interior points of segments.
									T invDet = ((T)1)/det;
									s0 *= invDet;
									s1 *= invDet;
									sqrDist = s0*(s0 + a01*s1 + ((T)2)*b0) +
											s1*(a01*s0 + s1 + ((T)2)*b1) + c;
							}
							else  // region 3 (side)
							{
									s1 = m_Seg1.ext_;
									tmpS0 = -(a01*s1 + b0);
									if (tmpS0 < -m_Seg0.ext_)
									{
											s0 = -m_Seg0.ext_;
											sqrDist = s0*(s0 - ((T)2)*tmpS0) +
													s1*(s1 + ((T)2)*b1) + c;
									}
									else if (tmpS0 <= m_Seg0.ext_)
									{
											s0 = tmpS0;
											sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
									}
									else
									{
											s0 = m_Seg0.ext_;
											sqrDist = s0*(s0 - ((T)2)*tmpS0) +
													s1*(s1 + ((T)2)*b1) + c;
									}
							}
					}
					else  // region 7 (side)
					{
							s1 = -m_Seg1.ext_;
							tmpS0 = -(a01*s1 + b0);
							if (tmpS0 < -m_Seg0.ext_)
							{
									s0 = -m_Seg0.ext_;
									sqrDist = s0*(s0 - ((T)2)*tmpS0) +
											s1*(s1 + ((T)2)*b1) + c;
							}
							else if (tmpS0 <= m_Seg0.ext_)
							{
									s0 = tmpS0;
									sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
							}
							else
							{
									s0 = m_Seg0.ext_;
									sqrDist = s0*(s0 - ((T)2)*tmpS0) +
											s1*(s1 + ((T)2)*b1) + c;
							}
					}
			}
			else
			{
					if (s1 >= -extDet1)
					{
							if (s1 <= extDet1)  // region 1 (side)
							{
									s0 = m_Seg0.ext_;
									tmpS1 = -(a01*s0 + b1);
									if (tmpS1 < -m_Seg1.ext_)
									{
											s1 = -m_Seg1.ext_;
											sqrDist = s1*(s1 - ((T)2)*tmpS1) +
													s0*(s0 + ((T)2)*b0) + c;
									}
									else if (tmpS1 <= m_Seg1.ext_)
									{
											s1 = tmpS1;
											sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
									}
									else
									{
											s1 = m_Seg1.ext_;
											sqrDist = s1*(s1 - ((T)2)*tmpS1) +
													s0*(s0 + ((T)2)*b0) + c;
									}
							}
							else  // region 2 (corner)
							{
									s1 = m_Seg1.ext_;
									tmpS0 = -(a01*s1 + b0);
									if (tmpS0 < -m_Seg0.ext_)
									{
											s0 = -m_Seg0.ext_;
											sqrDist = s0*(s0 - ((T)2)*tmpS0) +
													s1*(s1 + ((T)2)*b1) + c;
									}
									else if (tmpS0 <= m_Seg0.ext_)
									{
											s0 = tmpS0;
											sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
									}
									else
									{
											s0 = m_Seg0.ext_;
											tmpS1 = -(a01*s0 + b1);
											if (tmpS1 < -m_Seg1.ext_)
											{
													s1 = -m_Seg1.ext_;
													sqrDist = s1*(s1 - ((T)2)*tmpS1) +
															s0*(s0 + ((T)2)*b0) + c;
											}
											else if (tmpS1 <= m_Seg1.ext_)
											{
													s1 = tmpS1;
													sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
											}
											else
											{
													s1 = m_Seg1.ext_;
													sqrDist = s1*(s1 - ((T)2)*tmpS1) +
															s0*(s0 + ((T)2)*b0) + c;
											}
									}
							}
					}
					else  // region 8 (corner)
					{
							s1 = -m_Seg1.ext_;
							tmpS0 = -(a01*s1 + b0);
							if (tmpS0 < -m_Seg0.ext_)
							{
									s0 = -m_Seg0.ext_;
									sqrDist = s0*(s0 - ((T)2)*tmpS0) +
											s1*(s1 + ((T)2)*b1) + c;
							}
							else if (tmpS0 <= m_Seg0.ext_)
							{
									s0 = tmpS0;
									sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
							}
							else
							{
									s0 = m_Seg0.ext_;
									tmpS1 = -(a01*s0 + b1);
									if (tmpS1 > m_Seg1.ext_)
									{
											s1 = m_Seg1.ext_;
											sqrDist = s1*(s1 - ((T)2)*tmpS1) +
													s0*(s0 + ((T)2)*b0) + c;
									}
									else if (tmpS1 >= -m_Seg1.ext_)
									{
											s1 = tmpS1;
											sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
									}
									else
									{
											s1 = -m_Seg1.ext_;
											sqrDist = s1*(s1 - ((T)2)*tmpS1) +
													s0*(s0 + ((T)2)*b0) + c;
									}
							}
					}
			}
		}
		else
		{
				if (s1 >= -extDet1)
				{
						if (s1 <= extDet1)  // region 5 (side)
						{
								s0 = -m_Seg0.ext_;
								tmpS1 = -(a01*s0 + b1);
								if (tmpS1 < -m_Seg1.ext_)
								{
										s1 = -m_Seg1.ext_;
										sqrDist = s1*(s1 - ((T)2)*tmpS1) +
												s0*(s0 + ((T)2)*b0) + c;
								}
								else if (tmpS1 <= m_Seg1.ext_)
								{
										s1 = tmpS1;
										sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
								}
								else
								{
										s1 = m_Seg1.ext_;
										sqrDist = s1*(s1 - ((T)2)*tmpS1) +
												s0*(s0 + ((T)2)*b0) + c;
								}
						}
						else  // region 4 (corner)
						{
								s1 = m_Seg1.ext_;
								tmpS0 = -(a01*s1 + b0);
								if (tmpS0 > m_Seg0.ext_)
								{
										s0 = m_Seg0.ext_;
										sqrDist = s0*(s0 - ((T)2)*tmpS0) +
												s1*(s1 + ((T)2)*b1) + c;
								}
								else if (tmpS0 >= -m_Seg0.ext_)
								{
										s0 = tmpS0;
										sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
								}
								else
								{
										s0 = -m_Seg0.ext_;
										tmpS1 = -(a01*s0 + b1);
										if (tmpS1 < -m_Seg1.ext_)
										{
												s1 = -m_Seg1.ext_;
												sqrDist = s1*(s1 - ((T)2)*tmpS1) +
														s0*(s0 + ((T)2)*b0) + c;
										}
										else if (tmpS1 <= m_Seg1.ext_)
										{
												s1 = tmpS1;
												sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
										}
										else
										{
												s1 = m_Seg1.ext_;
												sqrDist = s1*(s1 - ((T)2)*tmpS1) +
														s0*(s0 + ((T)2)*b0) + c;
										}
								}
						}
				}
				else   // region 6 (corner)
				{
						s1 = -m_Seg1.ext_;
						tmpS0 = -(a01*s1 + b0);
						if (tmpS0 > m_Seg0.ext_)
						{
								s0 = m_Seg0.ext_;
								sqrDist = s0*(s0 - ((T)2)*tmpS0) +
										s1*(s1 + ((T)2)*b1) + c;
						}
						else if (tmpS0 >= -m_Seg0.ext_)
						{
								s0 = tmpS0;
								sqrDist = -s0*s0 + s1*(s1 + ((T)2)*b1) + c;
						}
						else
						{
								s0 = -m_Seg0.ext_;
								tmpS1 = -(a01*s0 + b1);
								if (tmpS1 < -m_Seg1.ext_)
								{
										s1 = -m_Seg1.ext_;
										sqrDist = s1*(s1 - ((T)2)*tmpS1) +
												s0*(s0 + ((T)2)*b0) + c;
								}
								else if (tmpS1 <= m_Seg1.ext_)
								{
										s1 = tmpS1;
										sqrDist = -s1*s1 + s0*(s0 + ((T)2)*b0) + c;
								}
								else
								{
										s1 = m_Seg1.ext_;
										sqrDist = s1*(s1 - ((T)2)*tmpS1) +
												s0*(s0 + ((T)2)*b0) + c;
								}
						}
				}
		}
	}
	else
	{
			// The segments are parallel.  The average b0 term is designed to
			// ensure symmetry of the function.  That is, dist(seg0,seg1) and
			// dist(seg1,seg0) should produce the same number.
			T e0pe1 = m_Seg0.ext_ + m_Seg1.ext_;
			T sign = (a01 > (T)0 ? (T)-1 : (T)1);
			T b0Avr = ((T)0.5)*(b0 - sign*b1);
			T lambda = -b0Avr;
			if (lambda < -e0pe1)
			{
					lambda = -e0pe1;
			}
			else if (lambda > e0pe1)
			{
					lambda = e0pe1;
			}

			s1 = -sign*lambda*m_Seg1.ext_/e0pe1;
			s0 = lambda + sign*s1;
			sqrDist = lambda*(lambda + ((T)2)*b0Avr) + c;
	}

	m_vClosestPoint0 = m_Seg0.center_ + s0*m_Seg0.dir_;
	m_vClosestPoint1 = m_Seg1.center_ + s1*m_Seg1.dir_;
	m_Seg0Param = s0;
	m_Seg1Param = s1;

	// Account for numerical round-off errors.
	if (sqrDist < (T)0)
	{
			sqrDist = (T)0;
	}
	return sqrDist;

}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template class CDistanceSegSeg<float>;
template class CDistanceSegSeg<double>;

}