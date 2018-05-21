#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include <DirectXMath.h>
#include <iostream>

#include "../datastructures/NestedOctree.h"


namespace LOD_Utils
{
	struct VertexBuffer
	{
		ID3D11Buffer* buffer; 
		UINT32 size; 

		VertexBuffer() {}
		VertexBuffer(ID3D11Buffer* buffer, UINT32 size) : buffer(buffer), size(size)
		{

		}

	};

	inline DirectX::XMVECTOR signVector(UINT index)
	{
		return XMVectorSet(index & 1 ? 1 : -1, index & 2 ? 1 : -1, index & 4 ? 1 : -1, 1); 
	}

	VertexBuffer createVertexBufferFromNode(NestedOctreeNode<Vertex>* pNode, ID3D11Device* device);


}
