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

}