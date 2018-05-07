#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include "LOD.h"


class No_LOD : public LOD
{
public:
	No_LOD(); 
	virtual ~No_LOD(); 

	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static void setUpTweakBar() {};
	virtual void draw(ID3D11DeviceContext* const context) override;

private: 
	ID3D11Buffer* vertexBuffer = nullptr;
	UINT strides;
	UINT vertCount; 
};

