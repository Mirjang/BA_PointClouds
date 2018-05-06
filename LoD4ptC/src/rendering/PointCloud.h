#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include <windows.h>
#include <vector>
#include "Vertex.h"

#include "../lod/LOD.h"

//Basic class representing a 3D PointCloud
class PointCloud
{
public:
	PointCloud();
	PointCloud(Vertex* pArrVertex, int vertCount, LOD* lod = nullptr);
	virtual ~PointCloud();


	size_t getVertexCount() { return vertices.size(); }

	ID3D11Buffer* vertexBuffer; 
	UINT strides; 
	std::vector<Vertex> vertices;	
	int refCount = 0; 
	LOD* lod; 
};

