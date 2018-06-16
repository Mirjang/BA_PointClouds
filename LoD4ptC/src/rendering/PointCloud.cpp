#include "PointCloud.h"


PointCloud::PointCloud()
{
}


PointCloud::~PointCloud()
{
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


void PointCloud::createLod(ID3D11Device* const device, LODMode mode)
{

	switch (g_lodSettings.mode)
	{
	case LODMode::NESTED_OCTREE_NAIVE: 
	{
		lod = new Nested_Octree_Naive_Avg(); 
		break; 
	}
	case LODMode::NESTED_OCTREE_POSSIONDISK:
	{
		lod = new Nested_Octree_PossionDisk();
		break;
	}
	case LODMode::KMEANS_SPHERE:
	{
		lod = new Kmeans_Spheres();
		break;
	}
	case LODMode::KMEANS_ELLIPSE: 
	{
		lod = new Kmeans_Ellipses(); 
		break; 
	}	
	case LODMode::REGIONS_SPHERE:
	{
		lod = new Regions_Spheres();
		break;
	}
	default:
		lod = new No_LOD();
		break;
	}

	lod->create(device, vertices); 

}
