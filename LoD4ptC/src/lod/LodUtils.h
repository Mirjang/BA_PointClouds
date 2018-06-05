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
		char marked = 0; 

		VertexBuffer() {}
		VertexBuffer(ID3D11Buffer* buffer, UINT32 size) : buffer(buffer), size(size)
		{

		}

	};

	inline DirectX::XMVECTOR signVector(UINT index)
	{
		return XMVectorSet(index & 1 ? 1.0f : -1.0f, index & 2 ? 1.0f : -1.0f, index & 4 ? 1 : -1.0f, 1.0f); 
	}

	VertexBuffer createVertexBufferFromNode(NestedOctreeNode<Vertex>* pNode, ID3D11Device* device);

	void printTreeStructure(const std::vector<OctreeVectorNode<VertexBuffer>>& verts, UINT32 nodeIndex = 0, UINT32 maxDepth = -1, UINT32 depth = 0);

}
