#include "Renderer.h"

#include <string>

#include <iostream>

#include "Vertex.h"

using namespace DirectX;

Renderer::Renderer(HWND* _phWindow, int* _pScreen_width, int* _pScreen_heigth) :phWindow(_phWindow), pScreen_width(_pScreen_width), pScreen_heigth(_pScreen_heigth)
{

}


Renderer::~Renderer()
{
	swapChain->SetFullscreenState(FALSE, NULL);


	SafeRelease(cbPerObjectBuffer);
	SafeRelease(vertexLayout);
	SafeRelease(VS);
	SafeRelease(VS_buffer);
	SafeRelease(GS);
	SafeRelease(GS_buffer);
	SafeRelease(PS);
	SafeRelease(PS_buffer);


	for (auto mesh : meshDict)
	{
		mesh.second->vertexBuffer->Release(); 
	}


	depthStencilBuffer->Release();
	depthStencilView->Release();
	renderTargetView->Release();

	stateRS_default->Release();
	stateRS_CCW->Release(); 

	swapChain->Release();
	d3dContext->Release();
	d3dDevice->Release();

#ifdef _DEBUG
	d3dDebug->ReportLiveDeviceObjects(D3D11_RLDO_DETAIL);

#endif //_DEBUG

}

/**
*	###	Init Functions	###
*/

void Renderer::initialize()
{
	initDirectX();
}

/* @TODO: Check return values!!!
*/
void Renderer::initDirectX()
{
	HRESULT result;

	//Multisample AA
	DXGI_SAMPLE_DESC descSampling;
	descSampling.Count = 8;
	descSampling.Quality = 0;

	DXGI_SWAP_CHAIN_DESC descSwapChain;
	ZeroMemory(&descSwapChain, sizeof(DXGI_SWAP_CHAIN_DESC));
	descSwapChain.BufferDesc.Height = *pScreen_heigth;
	descSwapChain.BufferDesc.Width = *pScreen_width;
	descSwapChain.BufferDesc.RefreshRate.Denominator = 1;
	descSwapChain.BufferDesc.RefreshRate.Numerator = 60;
	descSwapChain.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
	descSwapChain.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	descSwapChain.BufferDesc.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
	descSwapChain.BufferCount = 1;
	descSwapChain.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	descSwapChain.OutputWindow = *phWindow;
	descSwapChain.SampleDesc = descSampling;
	descSwapChain.Windowed = true;		//window / full screen
	descSwapChain.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

#ifdef _DEBUG
	result = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, D3D11_CREATE_DEVICE_DEBUG, NULL, NULL, D3D11_SDK_VERSION, &descSwapChain, &swapChain, &d3dDevice, NULL, &d3dContext);
	d3dDevice->QueryInterface(__uuidof(ID3D11Debug), (void**)&d3dDebug);

#else
	result = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, NULL, D3D11_SDK_VERSION, &descSwapChain, &swapChain, &d3dDevice, NULL, &d3dContext);
#endif // _DEBUG

	if (FAILED(result))
	{
		std::cout << "D3D init failed" << std::endl; 
		std::cin.get(); 
	}

	ID3D11Texture2D* backBuffer;
	swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&backBuffer));
	result = d3dDevice->CreateRenderTargetView(backBuffer, NULL, &renderTargetView);
	backBuffer->Release();

	if (FAILED(result))
	{
		std::cout << "D3D init failed" << std::endl;
		std::cin.get();
	}


	D3D11_TEXTURE2D_DESC descDepthStencil;
	ZeroMemory(&descDepthStencil, sizeof(D3D11_TEXTURE2D_DESC));
	descDepthStencil.Width = *pScreen_width;
	descDepthStencil.Height = *pScreen_heigth;
	descDepthStencil.MipLevels = 1;
	descDepthStencil.ArraySize = 1;
	descDepthStencil.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	descDepthStencil.SampleDesc = descSampling;
	descDepthStencil.Usage = D3D11_USAGE_DEFAULT;
	descDepthStencil.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	descDepthStencil.CPUAccessFlags = NULL;
	descDepthStencil.MiscFlags = NULL;



	result = d3dDevice->CreateTexture2D(&descDepthStencil, NULL, &depthStencilBuffer);
	if (FAILED(result))
	{
		std::cout << "D3D init failed" << std::endl;
		std::cin.get();
	}

	D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
	ZeroMemory(&descDSV, sizeof(D3D11_DEPTH_STENCIL_VIEW_DESC));
	descDSV.Format = DXGI_FORMAT_D32_FLOAT_S8X24_UINT;
	descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DMS;

	result = d3dDevice->CreateDepthStencilView(depthStencilBuffer, NULL, &depthStencilView);
	if (FAILED(result))
	{
		std::cout << "D3D init failed" << std::endl;
		std::cin.get();
	}

	D3D11_RASTERIZER_DESC descRasterizer;
	ZeroMemory(&descRasterizer, sizeof(D3D11_RASTERIZER_DESC));
	descRasterizer.FillMode = D3D11_FILL_SOLID;
	descRasterizer.CullMode = D3D11_CULL_NONE;
	descRasterizer.FrontCounterClockwise = true;
	descRasterizer.DepthBias = 0;
	descRasterizer.SlopeScaledDepthBias = 0.0f;
	descRasterizer.DepthBiasClamp = 0.0f;
	descRasterizer.DepthClipEnable = true;
	descRasterizer.ScissorEnable = false;
	descRasterizer.MultisampleEnable = true;
	descRasterizer.AntialiasedLineEnable = true;

	d3dDevice->CreateRasterizerState(&descRasterizer, &stateRS_default);

	descRasterizer.FrontCounterClockwise = true; 

	d3dDevice->CreateRasterizerState(&descRasterizer, &stateRS_CCW); 

	d3dContext->RSSetState(stateRS_default);

	d3dContext->OMSetRenderTargets(1, &renderTargetView, depthStencilView);

}


void Renderer::reloadShaders(RenderMode mode)
{

	HRESULT result;
	std::wstring shader = L""; 

	SafeRelease(cbPerObjectBuffer); 
	SafeRelease(vertexLayout); 



	 //Old code replaced by RenderEffect.h
	/* */
	SafeRelease(VS);
	SafeRelease(VS_buffer);
	SafeRelease(GS);
	SafeRelease(GS_buffer);
	SafeRelease(PS); 
	SafeRelease(PS_buffer); 


	switch (mode)
	{
	case QUAD_SPLAT:
		shader = L"./shaders/quadsplat.hlsl";
		break;
	case CIRCLE_SPLAT:
		shader = L"./shaders/circularsplat.hlsl";
		break;
	case ELLIPTIC_SPLAT:
		throw "Not implemented"; 
		break;
	default:
		break;
	}

	//load shaders
	ID3DBlob *errBlob;
	result = D3DCompileFromFile(shader.c_str(), NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, "VS", "vs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &VS_buffer, &errBlob);
	if (FAILED(result))
	{
		std::cout << "failed to compile shaders " << std::endl;
		std::cout << (char*)errBlob->GetBufferPointer() << std::endl; 
		std::cin.get();		
	}
	result = D3DCompileFromFile(shader.c_str(), NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, "PS", "ps_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &PS_buffer, &errBlob);
	if (FAILED(result))
	{
		std::cout << "failed to compile shaders " << std::endl;
		std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
		std::cin.get();

	}
	result = D3DCompileFromFile(shader.c_str(), NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, "GS", "gs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &GS_buffer, &errBlob);
	if (FAILED(result))
	{
		std::cout << "failed to compile shaders " << std::endl;
		std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
		std::cin.get();
	}


	result = d3dDevice->CreateVertexShader(VS_buffer->GetBufferPointer(), VS_buffer->GetBufferSize(), NULL, &VS);
	if (FAILED(result))
	{
		std::cout << "failed to load VertexShader "<< result << std::endl;
		std::cin.get();
	}

	result = d3dDevice->CreatePixelShader(PS_buffer->GetBufferPointer(), PS_buffer->GetBufferSize(), NULL, &PS);

	if (FAILED(result))
	{
		std::cout << "failed to load PixelShader " << result << std::endl;
		std::cin.get();
	}


	result = d3dDevice->CreateGeometryShader(GS_buffer->GetBufferPointer(), GS_buffer->GetBufferSize(), NULL, &GS);

	if (FAILED(result))
	{
		std::cout << "failed to load GeometryShader " << result << std::endl;
		std::cin.get();
	}

	d3dContext->VSSetShader(VS, NULL, NULL);
	d3dContext->GSSetShader(GS, NULL, NULL);
	d3dContext->PSSetShader(PS, NULL, NULL);
	/**/ //end old code

	if (g_renderEffect)	delete g_renderEffect;

	g_renderEffect = new RenderEffect(d3dDevice); 

	D3D11_INPUT_ELEMENT_DESC descVertexLayout[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 }, 

	};



	//create and set Input layout

	result = d3dDevice->CreateInputLayout(descVertexLayout, ARRAYSIZE(descVertexLayout), VS_buffer->GetBufferPointer(), VS_buffer->GetBufferSize(), &vertexLayout);
	if (FAILED(result))
	{
		std::cout << "failed to create InputLayout " << result << std::endl;
		std::cin.get();
	}
	d3dContext->IASetInputLayout(vertexLayout);

	d3dContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

	//create shader buffers
	D3D11_BUFFER_DESC desc_perObjBuffer;
	ZeroMemory(&desc_perObjBuffer, sizeof(D3D11_BUFFER_DESC));
	desc_perObjBuffer.Usage = D3D11_USAGE_DEFAULT;
	desc_perObjBuffer.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	desc_perObjBuffer.ByteWidth = sizeof(cbPerObject);

	result = d3dDevice->CreateBuffer(&desc_perObjBuffer, NULL, &cbPerObjectBuffer);
	if (FAILED(result))
	{
		if (FAILED(result))
		{
			std::cout << "failed to create constant buffer " << result << std::endl;
			std::cin.get();
		}
	}

	D3D11_VIEWPORT viewport;
	ZeroMemory(&viewport, sizeof(viewport));
	viewport.Height = static_cast<float>(*pScreen_heigth);
	viewport.Width = static_cast<float>(*pScreen_width);
	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	d3dContext->RSSetViewports(1, &viewport);
	
	d3dContext->RSSetState(stateRS_default);

	transparents.clear(); 

}

void Renderer::loadMesh( const std::string& name)
{

	//mesh has already been loaded or doesnt exist
	if (name == "") return; 

	auto res = meshDict.find(name);
	if (res != meshDict.end())
	{
		++res->second->refCount;
		return; 
	}

	HRESULT result;

	//Load mesh to RAM
	
	PointCloud* mesh = new PointCloud;
	mesh->refCount++;
	g_RessourceLoader->loadAsset(name, mesh);

	//generate LOD






	//init GPU buffers

	//create vertex buffer
	D3D11_BUFFER_DESC descVertexBuffer;
	ZeroMemory(&descVertexBuffer, sizeof(D3D11_BUFFER_DESC));
	descVertexBuffer.Usage = D3D11_USAGE_DEFAULT;
	descVertexBuffer.ByteWidth = sizeof(Vertex) * mesh->getVertexCount();
	descVertexBuffer.BindFlags = D3D11_BIND_VERTEX_BUFFER;

	D3D11_SUBRESOURCE_DATA dataVertexBuffer;
	ZeroMemory(&dataVertexBuffer, sizeof(D3D11_SUBRESOURCE_DATA));
	dataVertexBuffer.pSysMem = mesh->vertices.data();

	result = d3dDevice->CreateBuffer(&descVertexBuffer, &dataVertexBuffer, &mesh->vertexBuffer);

	if (FAILED(result))
	{
		std::cout << "failed to create vertex buffer " << result << std::endl;
		std::cin.get();
	}

	mesh->strides = sizeof(Vertex);

	meshDict.insert(std::pair<std::string, PointCloud*>(name, mesh));

}

void Renderer::deleteMesh(const std::string& name) //decrements refcounter and removes mesh if 0
{
	if (name == "") return;

	auto res = meshDict.find(name);
	if (res != meshDict.end())
	{
		--res->second->refCount;
		if (!res->second->refCount) //no longer referenced
		{
			PointCloud* mesh = res->second;
			meshDict.erase(res);
			delete mesh;

		}
	}
}



/**
*	###	Runtime Functions	###
*/

void Renderer::newFrame( XMFLOAT4X4* _m_View, XMFLOAT4X4* _m_Proj)
{

	m_View = _m_View;
	m_Proj = _m_Proj; 
	float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };

	d3dContext->ClearRenderTargetView(renderTargetView, clearColor); //Clears backbuffer ... Rendering has to be done after this point
	d3dContext->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, NULL);
}

void Renderer::render(const std::string& meshName, const DirectX::XMFLOAT4X4*  _m_World)
{
	auto result = meshDict.find(meshName);

	if (result == meshDict.end()) return; 

	UINT offset = 0; 
	d3dContext->IASetVertexBuffers(0, 1, &result->second->vertexBuffer, &result->second->strides, &offset); 

	//send WVP-Matrix to Vertex Shader
	XMStoreFloat4x4(&cbPerObj.m_wvp, XMLoadFloat4x4(_m_World) * XMLoadFloat4x4(m_View) * XMLoadFloat4x4(m_Proj));
	d3dContext->UpdateSubresource(cbPerObjectBuffer, 0, NULL, &cbPerObj, 0, 0);
	d3dContext->VSSetConstantBuffers(0, 1, &cbPerObjectBuffer);
	d3dContext->GSSetConstantBuffers(0, 1, &cbPerObjectBuffer);
	d3dContext->VSSetShader(VS, NULL, 0);
	d3dContext->GSSetShader(GS, NULL, 0);
	d3dContext->PSSetShader(PS, NULL, 0);
	d3dContext->Draw(result->second->getVertexCount(), 0);
	//d3dContext->Draw(10000, 0); // surface cant handle my shitty code


}

void Renderer::endFrame()
{

	swapChain->Present(0, 0);

	transparents.clear();
}