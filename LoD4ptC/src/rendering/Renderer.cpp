#include "Renderer.h"

#include <string>

#include <iostream>

#include "Vertex.h"

#define _DEBUG

using namespace DirectX;

Renderer::Renderer(HWND* _phWindow, int* _pScreen_width, int* _pScreen_heigth) :phWindow(_phWindow), pScreen_width(_pScreen_width), pScreen_heigth(_pScreen_heigth)
{

}


Renderer::~Renderer()
{
	swapChain->SetFullscreenState(FALSE, NULL);

	SafeRelease(Effects::cbShaderSettingsBuffer)
	SafeRelease(Effects::cbPerObjectBuffer);

	for (auto mesh : meshDict)
	{
		delete mesh.second->lod;
	}

	SafeRelease(depthStencilBuffer);
	SafeRelease(depthStencilView);
	SafeRelease(renderTargetView);

	SafeRelease(swapChain);
	SafeRelease(d3dContext);
	SafeRelease(d3dDevice);

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

	d3dContext->OMSetRenderTargets(1, &renderTargetView, depthStencilView);

}


void Renderer::reloadShaders()
{
	if (Effects::g_pCurrentPass) delete Effects::g_pCurrentPass; 

	Effects::deinit();
	Effects::init(d3dDevice);

	//create pass based on g_renderSettings

	//TODO: there will be different passes based on selected LOD strategy
	//ok this part is now officially a mess
	std::string vsname, gsname, psname;


	if (g_lodSettings.mode == LODMode::KMEANS_ELLIPSE)	//or any method that uses elliptical splats
	{
		vsname = g_renderSettings.drawLOD ? "VS_ELLIPTICAL_APPLY_DEPTHCOLOR" : "VS_ELLIPTICAL_PASSTHROUGH";

		gsname = "GS_ELLIPTICAL"; 

		switch (g_renderSettings.splatMode)
		{
		case QUAD_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_QUADELLIPSE_PHONG" : "PS_QUADELLIPSE_NOLIGHT";
			break;
		}
		case CIRCLE_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_ELLIPSE_PHONG" : "PS_ELLIPSE_NOLIGHT";
			break;
		}
		}
		Effects::g_pCurrentPass = Effects::createPass(vsname, psname, gsname, Effects::RS_STATE.CULL_NONE, Effects::LayoutEllipsis);

	}
	else if(g_lodSettings.mode == LODMode::KMEANS_SPHERE)
	{
		vsname = g_renderSettings.drawLOD ? "VS_RADIUS_APPLY_DEPTHCOLOR" : "VS_RADIUS_PASSTHROUGH";
		gsname = "GS_RADIUS";

		switch (g_renderSettings.splatMode)
		{
		case QUAD_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_QUAD_PHONG" : "PS_QUAD_NOLIGHT";
			break;
		}
		case CIRCLE_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_CIRCLE_PHONG" : "PS_CIRCLE_NOLIGHT";
			break;
		}
		}
		Effects::g_pCurrentPass = Effects::createPass(vsname, psname, gsname, Effects::RS_STATE.CULL_NONE, Effects::LayoutSpheres);
	}
	else
	{


		vsname = g_renderSettings.drawLOD ? "VS_APPLY_DEPTHCOLOR" : "VS_PASSTHROUGH";

		if (g_renderSettings.useLight)
		{
			if (g_renderSettings.determineSplatsize)
			{

				if (g_renderSettings.drawLOD)
				{
					gsname = "GS_LIT_ADAPTIVESPLATSIZE_DEPTHCOLOR";
					vsname = "VS_PASSTHROUGH";
				}
				else
				{
					gsname = "GS_LIT_ADAPTIVESPLATSIZE";
				}

			}
			else
			{
				gsname = "GS_LIT";
			}
		}
		else
		{
			if (g_renderSettings.determineSplatsize)
			{
				if (g_renderSettings.drawLOD)
				{
					gsname = "GS_UNLIT_ADAPTIVESPLATSIZE_DEPTHCOLOR";
					vsname = "VS_PASSTHROUGH";
				}
				else
				{
					gsname = "GS_UNLIT_ADAPTIVESPLATSIZE";
				}

			}
			else
			{
				gsname = "GS_UNLIT";
			}
		}


		switch (g_renderSettings.splatMode)
		{
		case QUAD_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_QUAD_PHONG" : "PS_QUAD_NOLIGHT";


			break;
		}
		case CIRCLE_SPLAT:
		{
			psname = g_renderSettings.useLight ? "PS_CIRCLE_PHONG" : "PS_CIRCLE_NOLIGHT";
			break;
		}
		}
		Effects::g_pCurrentPass = Effects::createPass(vsname, psname, gsname, Effects::RS_STATE.CULL_NONE);
	}



	HRESULT result;
	SafeRelease(Effects::cbShaderSettingsBuffer)

	SafeRelease(Effects::cbPerObjectBuffer);

	d3dContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

	//create shader buffers
	D3D11_BUFFER_DESC desc_perObjBuffer;
	ZeroMemory(&desc_perObjBuffer, sizeof(D3D11_BUFFER_DESC));
	desc_perObjBuffer.Usage = D3D11_USAGE_DEFAULT;
	desc_perObjBuffer.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	desc_perObjBuffer.ByteWidth = sizeof(Effects::PerObject);

	result = d3dDevice->CreateBuffer(&desc_perObjBuffer, NULL, &Effects::cbPerObjectBuffer);
	if (FAILED(result))
	{
		if (FAILED(result))
		{
			std::cout << "failed to create constant buffer " << result << std::endl;
			std::cin.get();
		}
	}


	desc_perObjBuffer.ByteWidth = sizeof(Effects::ShaderSettings);

	result = d3dDevice->CreateBuffer(&desc_perObjBuffer, NULL, &Effects::cbShaderSettingsBuffer);
	if (FAILED(result))
	{
		if (FAILED(result))
		{
			std::cout << "failed to create constant buffer " << result << std::endl;
			std::cin.get();
		}
	}

	desc_perObjBuffer.ByteWidth = sizeof(Effects::PerLOD);

	result = d3dDevice->CreateBuffer(&desc_perObjBuffer, NULL, &Effects::cbPerLODBuffer);
	if (FAILED(result))
	{
		if (FAILED(result))
		{
			std::cout << "failed to create constant buffer " << result << std::endl;
			std::cin.get();
		}
	}


	D3D11_VIEWPORT viewport = { 0 };
	viewport.Height = *pScreen_heigth;
	viewport.Width = *pScreen_width; 
	viewport.MinDepth = 0.0; 
	viewport.MaxDepth = 1.0;

	d3dContext->RSSetViewports(1, &viewport); 

	transparents.clear(); 

	Effects::cbShaderSettings.maxLOD = g_statistics.maxDepth; 

	updateShaderSettings(); 

}

void Renderer::updateShaderSettings()
{
	Effects::cbShaderSettings.aspectRatio = static_cast<float>(g_screenParams.width) / static_cast<float>(g_screenParams.height);
	d3dContext->UpdateSubresource(Effects::cbShaderSettingsBuffer, 0, NULL, &Effects::cbShaderSettings, 0, 0);
	d3dContext->VSSetConstantBuffers(Effects::BufferSlots::bsShaderSettings, 1, &Effects::cbShaderSettingsBuffer);
	d3dContext->GSSetConstantBuffers(Effects::BufferSlots::bsShaderSettings, 1, &Effects::cbShaderSettingsBuffer);
	d3dContext->PSSetConstantBuffers(Effects::BufferSlots::bsShaderSettings, 1, &Effects::cbShaderSettingsBuffer);
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

	mesh->createLod(d3dDevice, g_lodSettings.mode); 

	//init GPU buffers <--- moved to LOD implementatiions

	meshDict.insert(std::pair<std::string, PointCloud*>(name, mesh));

	updateShaderSettings();


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

void Renderer::newFrame( XMFLOAT4X4* _m_View, XMFLOAT4X4* _m_Proj, XMFLOAT4* camPos, XMFLOAT4* camDir)
{

	g_statistics.verticesDrawn = 0; 

	XMStoreFloat4(&Effects::cbPerObj.camPos, XMLoadFloat4(camPos));
	XMStoreFloat4(&Effects::cbPerObj.camDir, XMLoadFloat4(camDir));
	Effects::cbPerObj.viewMat = *_m_View;

	m_Proj = _m_Proj; 
	float clearColor[] = { 0.0f, 0.0f, 0.0f, 1.0f };

	d3dContext->ClearRenderTargetView(renderTargetView, clearColor); //Clears backbuffer ... Rendering has to be done after this point
	d3dContext->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, NULL);
}

void Renderer::render(const std::string& meshName, const DirectX::XMFLOAT4X4*  _m_World)
{
	auto result = meshDict.find(meshName);

	if (result == meshDict.end()) return; 

	XMMATRIX worldmat = XMLoadFloat4x4(_m_World); 


	//Set and update constant buffers
	XMStoreFloat4x4(&Effects::cbPerObj.worldMat, worldmat);
	XMStoreFloat4x4(&Effects::cbPerObj.wvpMat, worldmat * XMLoadFloat4x4(&Effects::cbPerObj.viewMat) * XMLoadFloat4x4(m_Proj));



	d3dContext->UpdateSubresource(Effects::cbPerObjectBuffer, 0, NULL, &Effects::cbPerObj, 0, 0);
	d3dContext->VSSetConstantBuffers(Effects::BufferSlots::bsPerObject, 1, &Effects::cbPerObjectBuffer);
	d3dContext->GSSetConstantBuffers(Effects::BufferSlots::bsPerObject, 1, &Effects::cbPerObjectBuffer);
	d3dContext->PSSetConstantBuffers(Effects::BufferSlots::bsPerObject, 1, &Effects::cbPerObjectBuffer);



	//use lod specific draw function (performs i.e traversal aswell as the final draw)
	result->second->lod->draw(d3dContext);

}

void Renderer::endFrame()
{

	swapChain->Present(0, 0);

	transparents.clear();
}