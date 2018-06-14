#pragma once

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#include <DirectXMath.h>
#include <iostream>
#include <cmath>

#include "../datastructures/NestedOctree.h"


namespace LOD_Utils
{
	struct VertexBuffer
	{
		ID3D11Buffer* buffer; 
		UINT32 size; 
		bool marked = false; 

		VertexBuffer() {}
		VertexBuffer(ID3D11Buffer* buffer, UINT32 size) : buffer(buffer), size(size)
		{

		}

	};

	struct EllipticalVertexBuffer : VertexBuffer
	{
		EllipticalVertexBuffer() : VertexBuffer() {}
		EllipticalVertexBuffer(ID3D11Buffer* buffer, UINT32 size, float pixelBouzndingRadius) : VertexBuffer( buffer, size), maxPixelWorldSize(pixelBouzndingRadius * 2)
		{

		}
		float maxPixelWorldSize; 
	};


	inline DirectX::XMVECTOR signVector(UINT index)
	{
		return XMVectorSet(
			index & 1 ? 1.0f : -1.0f,
			index & 2 ? 1.0f : -1.0f,
			index & 4 ? 1.0f : -1.0f, 
			1.0f); 
	}


	inline DirectX::XMVECTOR cartToPolarNormal(const DirectX::XMVECTOR& normal)
	{
		float zenith = acosf(normal.m128_f32[1]);
		float azimuth = atan2f(normal.m128_f32[1],  normal.m128_f32[0]);
		return XMVectorSet(zenith, azimuth, 0, 0); 
	}


	inline DirectX::XMVECTOR polarToCartNormal(const DirectX::XMVECTOR& normal)
	{
		float x = sinf(normal.m128_f32[0]) * cosf(normal.m128_f32[1]); 
		float y = sinf(normal.m128_f32[0]) * sinf(normal.m128_f32[1]);
		float z = cosf(normal.m128_f32[0]); 
		return XMVectorSet(x, y, z, 0); 
	}


	VertexBuffer createVertexBufferFromNode(NestedOctreeNode<Vertex>* pNode, ID3D11Device* device);

	EllipticalVertexBuffer createEllipsisVertexBufferFromNode(NestedOctreeNode<EllipticalVertex>* pNode, ID3D11Device* device);

	EllipticalVertexBuffer createSphereVertexBufferFromNode(NestedOctreeNode<SphereVertex>* pNode, ID3D11Device* device);


	void printTreeStructure(const std::vector<OctreeVectorNode<VertexBuffer>>& verts, UINT32 nodeIndex = 0, UINT32 maxDepth = -1, UINT32 depth = 0);

}
