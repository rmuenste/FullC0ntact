#include "triface.h"

namespace i3d {

CTriFace::CTriFace(void)
{

}

CTriFace::~CTriFace(void)
{

}

CTriFace::CTriFace(int vertexIndex[3])
{

	for(int i = 0; i < 3; i++)
	{
		m_VertIndices[i] = vertexIndex[i];
	}//end for

}

CTriFace::CTriFace(const CTriFace& f)
{
	
	for(int i=0;i<3;i++)
	{
		m_VertIndices[i]=f[i];
	}//end for

}//end  

void CTriFace::InitFace(const int vertexIndex[3])
{

	for(int i = 0; i < 3; i++)
	{
		m_VertIndices[i] = vertexIndex[i];
	}//end for

}//end initFace

void CTriFace::InitFace(int vertexIndex[3])
{

	for(int i = 0; i < 3; i++)
	{
		m_VertIndices[i] = vertexIndex[i];
	}//end for

}//end initFace

}

