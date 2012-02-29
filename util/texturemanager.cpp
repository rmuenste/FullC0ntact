#include "TextureManager.h"
#include "Bitmap24.h"
#include "../Geometry/3DModel.h"
#include <GL/gl.h>
#include <GL/glext.h>
#include <iostream>
#include <string>

CTextureManager::CTextureManager(void)
{
	m_iCount=0;
	for(int i=0;i<MAX_TEX;i++)
		this->m_iTextures[i]=0;
}

CTextureManager::~CTextureManager(void)
{
}

int CTextureManager::BuildModelTex(C3DModel &pModel)
{
	using namespace std;
	vector<tMaterialInfo>::iterator mIter;

	for(mIter=pModel.m_pMaterials.begin();mIter!=pModel.m_pMaterials.end();mIter++)
	{
		tMaterialInfo& t_matInfo=*mIter;

		char *strFileName = t_matInfo.strFile;

		string sFile(strFileName);

		size_t pos = sFile.find(".");

		string sType = sFile.substr(pos);

		if(sType == ".jpg")
		{
			
		}//end if
		else if(sType == ".bmp" || sType ==".BMP")
		{
			CreateTexture(strFileName);
		}//end if
		else if(sType == ".dds" || sType ==".DDS")
		{
			sFile.replace(pos,sFile.npos,".bmp");
			CreateTexture(sFile.c_str());
		}//end else if
		else
		{
			return 0;
		}//end else

		//set the material ID
		t_matInfo.texureId=m_iCount;
		//increase the amount of managed textures
		m_iCount++;
	}//end for

	return 1;
}

int CTextureManager::GetTexture(int iTexID)
{
	return m_iTextures[iTexID];
}

void CTextureManager::CreateTexture(const char* strFileName)
{

	//create a bitmap
	CBitmap24 tex;
	tex.LoadFromFile(strFileName);

	//generate a texture name and store it int the parameter texture
	glGenTextures(1,&m_iTextures[m_iCount]); 
	glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
	//bind and create the current texture
	glBindTexture(GL_TEXTURE_2D, m_iTextures[m_iCount]);

	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,tex.m_lWidth,tex.m_lHeight,0,GL_BGR, GL_UNSIGNED_BYTE,tex.m_PixelData);
	

}//end LoadTexture

void CTextureManager::CreateTextureSimple(const char* strFileName)
{

	//create a bitmap
	CBitmap24 tex;
	tex.LoadFromFile(strFileName);

	//generate a texture name and store it int the parameter texture
	glGenTextures(1,&m_iTextures[m_iCount]); 
	glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
	//bind and create the current texture
	glBindTexture(GL_TEXTURE_2D, m_iTextures[m_iCount]);

	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,tex.m_lWidth,tex.m_lHeight,0,GL_BGR, GL_UNSIGNED_BYTE,tex.m_PixelData);
	
	m_iCount++;

}//end LoadTexture