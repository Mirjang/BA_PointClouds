#include "PointCloud.h"


PointCloud::PointCloud()
{
}


PointCloud::~PointCloud()
{
	vertexBuffer->Release();
	vertices.clear(); 
}

PointCloud::PointCloud(Vertex* pArrVertex, int vertCount) 
{
	vertices.clear();
	vertices.reserve(vertCount); 
	for (int i = 0; i < vertCount; ++i)
	{
		vertices.push_back(*(pArrVertex + i));
	}

}

