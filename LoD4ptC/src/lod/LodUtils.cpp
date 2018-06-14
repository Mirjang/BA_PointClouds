#include "LodUtils.h"

namespace LOD_Utils
{

	VertexBuffer createVertexBufferFromNode(NestedOctreeNode<Vertex>* pNode, ID3D11Device* device)
	{
		

		HRESULT result;
		ID3D11Buffer* vbuffer = nullptr;

		//create vertex buffer
		D3D11_BUFFER_DESC descVertexBuffer;
		ZeroMemory(&descVertexBuffer, sizeof(D3D11_BUFFER_DESC));
		descVertexBuffer.Usage = D3D11_USAGE_DEFAULT;
		descVertexBuffer.ByteWidth = pNode->data.size() * sizeof(Vertex);
		descVertexBuffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;

		D3D11_SUBRESOURCE_DATA dataVertexBuffer;
		ZeroMemory(&dataVertexBuffer, sizeof(D3D11_SUBRESOURCE_DATA));
		dataVertexBuffer.pSysMem = pNode->data.data();

		result = device->CreateBuffer(&descVertexBuffer, &dataVertexBuffer, &vbuffer);

		if (FAILED(result))
		{
			std::cout << "failed to create vertex buffer " << result << std::endl;
			std::cin.get();
		}

		return VertexBuffer(vbuffer, pNode->data.size());

	}

	EllipticalVertexBuffer createSphereVertexBufferFromNode(NestedOctreeNode<SphereVertex>* pNode, ID3D11Device* device)
	{
		float boundingSphere = 0; 
		for (auto vert : pNode->data)
		{
			boundingSphere = max(boundingSphere, vert.radius); 
		}


		HRESULT result;
		ID3D11Buffer* vbuffer = nullptr;

		//create vertex buffer
		D3D11_BUFFER_DESC descVertexBuffer;
		ZeroMemory(&descVertexBuffer, sizeof(D3D11_BUFFER_DESC));
		descVertexBuffer.Usage = D3D11_USAGE_DEFAULT;
		descVertexBuffer.ByteWidth = pNode->data.size() * sizeof(SphereVertex);
		descVertexBuffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;

		D3D11_SUBRESOURCE_DATA dataVertexBuffer;
		ZeroMemory(&dataVertexBuffer, sizeof(D3D11_SUBRESOURCE_DATA));
		dataVertexBuffer.pSysMem = pNode->data.data();

		result = device->CreateBuffer(&descVertexBuffer, &dataVertexBuffer, &vbuffer);

		if (FAILED(result))
		{
			std::cout << "failed to create vertex buffer " << result << std::endl;
			std::cin.get();
		}

		return EllipticalVertexBuffer(vbuffer, pNode->data.size(), boundingSphere);

	}

	EllipticalVertexBuffer createEllipsisVertexBufferFromNode(NestedOctreeNode<EllipticalVertex>* pNode, ID3D11Device* device)
	{
		XMVECTOR boundingSphere = XMVectorZero();
		for (auto vert : pNode->data)
		{
			boundingSphere = XMVectorMax(boundingSphere, XMLoadFloat3(&vert.major)); 
		}


		HRESULT result;
		ID3D11Buffer* vbuffer = nullptr;

		//create vertex buffer
		D3D11_BUFFER_DESC descVertexBuffer;
		ZeroMemory(&descVertexBuffer, sizeof(D3D11_BUFFER_DESC));
		descVertexBuffer.Usage = D3D11_USAGE_DEFAULT;
		descVertexBuffer.ByteWidth = pNode->data.size() * sizeof(EllipticalVertex);
		descVertexBuffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;

		D3D11_SUBRESOURCE_DATA dataVertexBuffer;
		ZeroMemory(&dataVertexBuffer, sizeof(D3D11_SUBRESOURCE_DATA));
		dataVertexBuffer.pSysMem = pNode->data.data();

		result = device->CreateBuffer(&descVertexBuffer, &dataVertexBuffer, &vbuffer);

		if (FAILED(result))
		{
			std::cout << "failed to create vertex buffer " << result << std::endl;
			std::cin.get();
		}

		return EllipticalVertexBuffer(vbuffer, pNode->data.size(), XMVector3Length(boundingSphere).m128_f32[0]);

	}

	void printTreeStructure(const std::vector<OctreeVectorNode<VertexBuffer>>& verts, UINT32 nodeIndex, UINT32 maxDepth, UINT32 depth)
	{
		for (int i = 0; i < depth; ++i)
		{
			std::cout << "--";
		}
		std::cout << nodeIndex << std::endl;


		if (depth != maxDepth)
		{
			UINT8 numchildren = 0;

			for (int i = 0; i < 8; ++i)
			{
				if (verts[nodeIndex].children & (0x01 << i))	//ist ith flag set T->the child exists
				{
					printTreeStructure(verts, verts[nodeIndex].firstChildIndex + numchildren, maxDepth, depth + 1);
					++numchildren;
				}
			}
		}

	}


}