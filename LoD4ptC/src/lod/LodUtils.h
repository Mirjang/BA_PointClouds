#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

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


	VertexBuffer createVertexBufferFromNode(NestedOctreeNode<Vertex>* pNode, ID3D11Device* device);


}
