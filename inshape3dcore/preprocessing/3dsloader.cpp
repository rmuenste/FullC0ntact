
#include "3dsloader.h"
#include "3dmodel.h"
#include <string.h>

namespace i3d {

C3DSLoader::C3DSLoader(void)
{
}

C3DSLoader::~C3DSLoader(void)
{
}

/* reads the .obj file specified in strFileName */
void C3DSLoader::ReadModelFromFile(C3DModel *pModel,const char *strFileName)
{

	unsigned short id;
	unsigned int   length;
	unsigned int   bytesRead = 0;

	//set the global model pointer
	this->m_pModel = pModel;

	// The file pointer
	m_FilePointer = fopen(strFileName, "rb");

	if(!m_FilePointer)
	{
		cout<<"Could not open file: "<<strFileName<<endl;
		return;
	}
	
	//We read in the first chunk, the
	//id should be MAIN3DS 
	bytesRead+=fread(&id,1,2,m_FilePointer);
	bytesRead+=fread(&length,1,4,m_FilePointer);
	
	//If the Id is not main3ds the must be an error and we quit
	if(id != MAIN3DS)
	{
		cerr<<"Model is invalid..."<<endl;		
		return;
	}
	//read the main chunk
	ReadMainChunk(length,bytesRead);


	//close the filePointer
	fclose(m_FilePointer);

}//end ReadModelFromFile

// Reads the main chunk of a 3ds file
void C3DSLoader::ReadMainChunk(unsigned int iLength, unsigned int iPosition)
{
	//get a fresh working chunk	
	tChunk Chunk = {0};

	//Read until we find the end
	//of the file
	while(iPosition < iLength)
	{
		//start by reading the next chunk
		ReadChunk(Chunk);

		switch(Chunk.ID)
		{
		case VERSION:
			//cout<<"found Version"<<endl;
			Chunk.bytesRead+=fread(m_iBuffer,1,Chunk.length-Chunk.bytesRead,m_FilePointer);
			//cout<<"Version: "<<m_iBuffer[0]<<endl;
			break;
		case EDIT3DS:
			//cout<<"found EditorChunk"<<endl;
			ReadEditChunk(Chunk);
			break;
		default:
			//We dont know the chunk and/or dont want it so
			//we simply read past it and forget about the information
			//inside the chunk.
			Chunk.bytesRead+=fread(m_iBuffer,1,Chunk.length-Chunk.bytesRead,m_FilePointer);
			break;

		}//end switch

		//add the bytes read to the main chunk offset
		iPosition+=Chunk.bytesRead;

	}//end while

}//end ReadMainChunk

// Read a simple chunk
void C3DSLoader::ReadChunk(tChunk& pChunk)
{
	//the first 2 bytes of the chunk are its ID
	pChunk.bytesRead = fread(&pChunk.ID,1,2,m_FilePointer);

	//read in the length of the chunk, it is stored
	//in the next 4 bytes of the chunk
	pChunk.bytesRead += fread(&pChunk.length,1,4,m_FilePointer);

}//end C3DSLoader

int C3DSLoader::ReadEditChunk(tChunk& pEditChunk)
{
	//get a fresh working chunk
	tChunk currentChunk={0};

	//read the sub chunks until we find the end of the parent chunk
	while(pEditChunk.bytesRead < pEditChunk.length)
	{
		//Read the next chunk
		ReadChunk(currentChunk);

		switch(currentChunk.ID)
		{
		case EDIT_MATERIAL:
			//cout<<"found Material Chunk"<<endl;
			ReadMaterialChunk(currentChunk);
			break;
		case EDIT_OBJECT:
			//cout<<"found Object Chunk"<<endl;
			//read the name of the object
			char strName[255];
			currentChunk.bytesRead+=GetString(strName);
			{
			C3DMesh mesh(strName);
			//We have read in the name of the object
			//now we are going to read its data
			ReadObjectChunk(currentChunk,mesh);
			m_pModel->m_vMeshes.push_back(mesh);
			}
			break;
		default:
			currentChunk.bytesRead+=fread(m_iBuffer,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			break;
		}//end switch
 
		//Just read and discard unknown or unwanted chunks
		pEditChunk.bytesRead+=currentChunk.bytesRead;
	}//end while
	return 0;
}//end ReadEditChunk

void C3DSLoader::ReadMaterialChunk(tChunk& pMaterialChunk)
{
	//get a fresh working chunk
	tChunk currentChunk={0};

	//here we want to get a MaterialInfo structure
	tMaterialInfo t_matInf = {{0}};

	//read the sub chunks until we find the end of the parent chunk
	while(pMaterialChunk.bytesRead < pMaterialChunk.length)
	{
		//Read the next chunk
		ReadChunk(currentChunk);

		switch(currentChunk.ID)
		{
		case MATNAME:
			cout<<"found MATNAME"<<endl;
			//read the material name to our structure
			currentChunk.bytesRead+=fread(&t_matInf.strName,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			cout<<t_matInf.strName<<endl;
			break;
		case MATDIFFUSE:
			cout<<"found MATDIFFUSE"<<endl;
			currentChunk.bytesRead+=fread(m_iBuffer,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			break;
		case MATMAP:
			cout<<"found MATMAP"<<endl;
			break;
		case MATMAPFILE:
			//because we at the moment do not exately know if the matmap is always
			//followed by exately one matmapfile, the current programming may cause on error
			cout<<"found MATMAPFILE"<<endl;
			currentChunk.bytesRead+=fread(&t_matInf.strFile,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			cout<<t_matInf.strFile<<endl;
			break;
		default:
			//Just read and discard unknown or unwanted chunks
			currentChunk.bytesRead+=fread(m_iBuffer,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			break;
		}//end switch
 
		//add the bytes read to the parentChunk
		pMaterialChunk.bytesRead+=currentChunk.bytesRead;
	}//end while
	
	//finally add the new material to the models list of materials
	m_pModel->AddMaterial(t_matInf);

}//end ReadMaterialChunk

void C3DSLoader::ReadObjectChunk(tChunk& pObjectChunk,C3DMesh& pMesh)
{
	//get a fresh working chunk
	tChunk currentChunk={0};

	//read the sub chunks until we find the end of the parent chunk
	while(pObjectChunk.bytesRead < pObjectChunk.length)
	{
		//Read the next chunk
		ReadChunk(currentChunk);

		switch(currentChunk.ID)
		{
		case OBJECT_MESH:					
		
			// We found a new object, so let's read in it's info using recursion
			ReadTrimeshChunk(currentChunk,pMesh);
			break;
		default:
			currentChunk.bytesRead+=fread(m_iBuffer,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			break;
		}//end switch
 
		//Just read and discard unknown or unwanted chunks
		pObjectChunk.bytesRead+=currentChunk.bytesRead;
	}//end while

}//end ReadObjectChunk

void C3DSLoader::ReadTrimeshChunk(tChunk& pTriMeshChunk,C3DMesh& pMesh)
{
	//get a fresh working chunk
	tChunk currentChunk={0};

	//read the sub chunks until we find the end of the parent chunk
	while(pTriMeshChunk.bytesRead < pTriMeshChunk.length)
	{
		//Read the next chunk
		ReadChunk(currentChunk);

		switch(currentChunk.ID)
		{

		case OBJECT_VERTICES:				// This is the objects vertices
			ReadVertexChunk(currentChunk,pMesh);
			break;

		case OBJECT_FACES:					// This is the objects face information
			ReadVertexIndexChunk(currentChunk,pMesh);
			break;

		case OBJECT_MATERIAL:				// This holds the material name that the object has
			ReadObjectMaterialChunk(currentChunk,pMesh);			
			break;

		case OBJECT_UV:						// This holds the UV texture coordinates for the object
			// This chunk holds all of the UV coordinates for our object.  Let's read them in.
			ReadTexCoordChunk(currentChunk,pMesh);
			break;
		default:
			//Just read and discard unknown or unwanted chunks
			currentChunk.bytesRead+=fread(m_iBuffer,1,currentChunk.length-currentChunk.bytesRead,m_FilePointer);
			break;
		}//end switch
 
		//advance the parent pointer
		pTriMeshChunk.bytesRead+=currentChunk.bytesRead;
	}//end while

}//End ReadTrimeshChunk

void C3DSLoader::ReadVertexChunk(tChunk& pVertexChunk,C3DMesh& pMesh)
{

	short int sVerts=0;
	//read the number of vertices
	pVertexChunk.bytesRead+=fread(&sVerts,1,2,m_FilePointer);
	//set the number of vertices in the object
	pMesh.SetNumVertices((int)sVerts);
	Vertex3Array& pVerts = pMesh.GetVertices();
	pVerts.Resize((int)sVerts);
	//read in all the vertices at once
	pVertexChunk.bytesRead+=fread(&pVerts[0],1,pVertexChunk.length-pVertexChunk.bytesRead,m_FilePointer);
	
	//In the coordinate system of 3d max the z axis is pointing up
	//so to get the standard OpenGl like orientation we need to
	//swap the z and y coordindates and negate the z 
	//correct the orientation
	for(int i=0;i<pMesh.NumVertices();i++)
	{
		float y=pMesh.GetVertices()[i].y;
		pMesh.GetVertices()[i].y = pMesh.GetVertices()[i].z;
		pMesh.GetVertices()[i].z = -y;
	}//end for

}//end ReadVertexChunk

void C3DSLoader::ReadObjectMaterialChunk(tChunk& pObjectMaterialChunk,C3DMesh& pMesh)
{
	//A 3ds material can be a color, a texture map or it can hold information
	//about the shine, the glossiness of the material
	
	char strMaterial[255] = {0};

	//we first read the material string     
	pObjectMaterialChunk.bytesRead+=GetString(strMaterial);

	//loop through all texture
	for(unsigned int i=0;i<m_pModel->m_pMaterials.size();i++)
	{
		
		if(strcmp(strMaterial,m_pModel->m_pMaterials[i].strName)==0)
		{
			//the material was found check if it is a texture map
			if(strlen(strMaterial)>0)
			{
				pMesh.SetTextured(true);
				pMesh.SetMaterialID(i);
			}//end if
		//leave the for loop
		break;
		}//end if
		else
		{
			if(!pMesh.IsTextured())
			{
				pMesh.SetMaterialID(-1);
			}//end if
		}//end else
	}//end for
	//read the rest of this chunk and discard this information
	pObjectMaterialChunk.bytesRead+=fread(m_iBuffer,1,pObjectMaterialChunk.length-pObjectMaterialChunk.bytesRead,m_FilePointer);
}//end ReadObjectMaterialChunk

void C3DSLoader::ReadVertexIndexChunk(tChunk& pVertexIndexChunk,C3DMesh& pMesh)
{

	unsigned short idx=0;
	short int iFaces=0;
	//read the number of faces
	pVertexIndexChunk.bytesRead+=fread(&iFaces,1,2,m_FilePointer);
	//set the number of vertices in the object
	pMesh.SetNumFaces((int)iFaces);
	TriFaceArray& pFaces=pMesh.GetFaces();
	//adjust the size of the faces
	pFaces.Resize((int)iFaces);

	//loop over all faces of the object
	for(int i=0;i<pMesh.NumFaces();i++)
	{
		//loop over all vertices of the face
		//the 4th vertex carries visibility info
		for(int j=0;j<3;j++)
		{
			//read the vertices
			pVertexIndexChunk.bytesRead+=fread(&idx,1,2,m_FilePointer);
			pFaces[i].SetIndex(j,(int)idx);
		}//end for j
		pVertexIndexChunk.bytesRead+=fread(&idx,1,2,m_FilePointer);
	}//end for i


}//end ReadVertexIndexChunk

void C3DSLoader::ReadTexCoordChunk(tChunk& pTCoordChunk,C3DMesh& pMesh)
{

	short int sTVerts=0;
	//read the number of vertices
	pTCoordChunk.bytesRead+=fread(&sTVerts,1,2,m_FilePointer);
	//set the number of vertices in the object
	pMesh.SetNumTCoords((int)sTVerts);
	TCoordArray& pTVerts = pMesh.GetTCoords();
	pTVerts.Resize(pMesh.NumTCoords());
	//read in all the vertices at once
	pTCoordChunk.bytesRead+=fread(&pTVerts[0],1,pTCoordChunk.length-pTCoordChunk.bytesRead,m_FilePointer);

}//end ReadTexCoordChunk

int C3DSLoader::GetString(char *pBuffer)
{
	int idx = 0;
	char a;
	//fread(pBuffer,1,1,m_FilePointer);
	a=pBuffer[idx];
	do
	{
		fread(pBuffer+idx,1,1,m_FilePointer);
		a=pBuffer[idx];
	}while(*(pBuffer + idx++) !=0);

	return (int)strlen(pBuffer)+1;

}//end GetString

}