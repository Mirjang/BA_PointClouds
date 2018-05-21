#include "No_LOD.h"

#include "../global/utils.h"


No_LOD::No_LOD()
{
}


No_LOD::~No_LOD()
{
	SafeRelease(vertexBuffer);
}


void No_LOD::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT result; 

	vertCount = vertices.size();
	strides = sizeof(Vertex);


	//create vertex buffer
	D3D11_BUFFER_DESC descVertexBuffer;
	ZeroMemory(&descVertexBuffer, sizeof(D3D11_BUFFER_DESC));
	descVertexBuffer.Usage = D3D11_USAGE_DEFAULT;
	descVertexBuffer.ByteWidth = strides * vertCount;
	descVertexBuffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;

	D3D11_SUBRESOURCE_DATA dataVertexBuffer;
	ZeroMemory(&dataVertexBuffer, sizeof(D3D11_SUBRESOURCE_DATA));
	dataVertexBuffer.pSysMem = vertices.data();

	result = device->CreateBuffer(&descVertexBuffer, &dataVertexBuffer, &vertexBuffer);

	if (FAILED(result))
	{
		std::cout << "failed to create vertex buffer " << result << std::endl;
		std::cin.get();
	}

}

void No_LOD::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	SafeRelease(vertexBuffer); 
	create(device, vertices); 
}


void No_LOD::draw(ID3D11DeviceContext* const context)
{
	g_statistics.verticesDrawn = vertCount; 
	Effects::g_pCurrentPass->apply(context); 

	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, &vertexBuffer, &strides, &offset);

	context->Draw(vertCount, 0);
}
