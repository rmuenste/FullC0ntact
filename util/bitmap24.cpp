#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "Bitmap24.h"

using namespace std;

/**
*/
CBitmap24::CBitmap24(void)
{
	m_PixelData=NULL;
}// CBitmap24

/**
*/
CBitmap24::CBitmap24(const char* strFileName)
{

}// CBitmap24

/**
*/
CBitmap24::~CBitmap24(void)
{
	if(this->m_PixelData)
	{
		delete[] m_PixelData;
		m_PixelData=NULL;
	}//end if

}// ~CBitmap24

/** 
	Loads a Bitmap image from the specified file
*/ 
int CBitmap24::LoadFromFile(const char* strFileName)
{
	//offset pointer in the data file
	unsigned int iOffset = 0;
	//get the size of a DWORD on the machine
	int sizelong =sizeof(DWORD);
	//the boolean value is true is the image is padded
	bool bIsPadded = false;

	//open the file in read binary mode
	FILE* pFile = fopen(strFileName,"rb");

	if(!pFile) 
	{
		// Display an error message and don't load anything if no file was found
		cerr<<"File: "<<strFileName<<" was not found..."<<endl;
		return 0;
	}//end if
	
	//read in the bitmap header
	fread(&bmfh,sizeof(BITMAPFILEHEADER),1,pFile);
	//advance the offset accordingly
	iOffset += sizeof(BITMAPFILEHEADER);

	//check if the file type is right
	if(bmfh.bfType != 19778)
	{
		cout<<"Wrong file type"<<endl;
		return WRONG_FILE_SIZE;
	}//end if

	//next we read the bitmapinfoheader from the file
	fread(&bmih,sizeof(BITMAPINFOHEADER),1,pFile);
	//advance the offset accordingly
	iOffset += sizeof(BITMAPINFOHEADER);

	//if this is not 24bit Bitmap, we exit
	if(bmih.biBitCount != 24)
	{
		cout<<"This is not a 24bit bitmap file, these file types are not supported"<<endl;
	}//end if

	//since this a 24bit Bitmap the raw image data should start now
	//check if the width of the image ends on a DWORD border
	if(bmih.biWidth % sizelong != 0)
		bIsPadded=true;

	//we calculate the width of the image in bytes
	LONG byteWidth=(LONG)((float)bmih.biWidth*(float)bmih.biBitCount/8.0);
	//
	LONG widthPadd=byteWidth;
	//his block of bytes describes the image, pixel by pixel.
	//Pixels are stored "upside-down" with respect to normal image raster scan order,
	//starting in the lower left corner, going from left to right,
	//and then row by row from the bottom to the top of the image.
	//Uncompressed Windows bitmaps can also be stored from the top row to the bottom,
	//if the image height value is negative.In this case the image height is a
	//negative value.

	//So we will check the image height value and
	//if it is negative we will start reading from the
	//end of the file.
	//Some programs will pad not only the data,
	//but also the file size to a DWORD border.So we have to
	//take into account that there maybe padding at the
	//end of the file.
	
	//The size without the end padding...
	int size=byteWidth*bmih.biHeight+bmfh.bfOffBits;
	BYTE* pImageData = new BYTE[size];
	//read in the data all at once
	iOffset += fread(pImageData,1,size,pFile);
	
	widthPadd=widthPadd%4;
	if(widthPadd==0)
	{
		this->m_PixelData= new BYTE[size];
		memcpy(m_PixelData,pImageData,size);

	}//end if
	else
	{
		cerr<<"Error: The bitmap size is not divisible by 4."<<endl;
		return WRONG_IMAGE_SIZE;
	}//end else

	//assign the parameters of the bitmap
	m_lWidth  = bmih.biWidth;
	m_lHeight = bmih.biHeight;

	delete[] pImageData;
	return SUCCESS;

}// LoadFromFile
