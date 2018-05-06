#include "PointCloud.h"


PointCloud::PointCloud()
{
}


PointCloud::~PointCloud()
{
	vertexBuffer->Release();
	vertices.clear(); 

	delete lod; 
}

PointCloud::PointCloud(Vertex* pArrVertex, int vertCount, LOD* lod): lod(lod)
{
	vertices.clear();
	vertices.reserve(vertCount); 
	for (int i = 0; i < vertCount; ++i)
	{
		vertices.push_back(*(pArrVertex + i));
	}

}

