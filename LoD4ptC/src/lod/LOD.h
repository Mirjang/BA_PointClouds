#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include "../global/utils.h"
#include "../global/Octree.h"
#include "../rendering/Vertex.h"
#include "../rendering/Effects.h"

#include <AntTweakBar.h>


using std::vector; 
using std::unordered_map; 

namespace LOD_Helpers
{

}

class LOD
{
public:
	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) = 0;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) = 0;
	static void setUpTweakBar() {};
	virtual void draw(ID3D11DeviceContext* const context) = 0;


};
