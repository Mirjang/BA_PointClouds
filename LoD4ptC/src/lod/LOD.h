#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "../global/utils.h"
#include "../global/Octree.h"
#include "../rendering/PointCloud.h"
#include "../rendering/Vertex.h"

using std::vector; 
using std::unordered_map; 



namespace LOD_Helpers
{
	struct NodeInfo
	{
		int numChildren; 

	};

	template<class VertType>
	void sortIntoOctree(const vector<Vertex>& vertices, Octree<VertType>& outTree); 

}

class LOD
{
public:
	virtual void create(const ID3D11Device* device, vector<Vertex>& vertices) = 0; 
	virtual void draw() = 0; 
};


