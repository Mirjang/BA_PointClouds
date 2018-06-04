#pragma once

#include <windows.h>
#include <windowsx.h>


// DirectX includes

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>

#include <DirectXMath.h>

#include <list>
#include <map>

#include "Effects.h"

#include "../ressources/RessourceLoader.h"
#include "./PointCloud.h"
#include "../global/utils.h"

#include "../lod/LodImplementations.h"

using namespace DirectX; 

class Renderer
{
public:
	Renderer(HWND* _phWindow, int* _pScreen_width, int* _pScreen_heigth);
	~Renderer();

	void initialize();	//Inits DirectX
	void loadMesh(const std::string& name);	//creates per mesh vertex buffer
	void deleteMesh(const std::string& name); //decrements refcounter and removes mesh if 0


	/**
	*	###	Init Function Prototypes	###
	*/

	inline void initDirectX(); 
	void reloadShaders(); 

	void updateShaderSettings(); 


	/**
	*	###	Runtime Function Prototypes	###
	*/
	void newFrame(XMFLOAT4X4* _m_View, XMFLOAT4X4* _m_Proj, XMFLOAT4* camPos, XMFLOAT4* camDir);		//Signals a new Frame, clears screen
	void render(const std::string& meshName, const XMFLOAT4X4*  m_World);		//Renders 1 GO to screen
	void endFrame();

	void setAndUploadLODSplatSettings(const float& size)
	{
		if (Effects::cbPerLOD.splatRadius.x == size) return; 

		Effects::SetSplatSize(size); 
		Effects::UpdatePerLODBuffer(d3dContext); 
	}

	void setUserInputSplatSize(const float& size)
	{
		if (Effects::cbShaderSettings.splatSize.x == size) return; 
		Effects::cbShaderSettings.splatSize = XMFLOAT2(size, size * Effects::cbShaderSettings.aspectRatio);

		updateShaderSettings();
	}

	void setLight(const XMVECTOR& direction, const XMVECTOR& color)
	{
		XMStoreFloat4(&Effects::cbPerObj.lightDir,direction);
		XMStoreFloat4(&Effects::cbPerObj.lightColor, color);
	}

	/**
	*	### public Variables	###
	*/
	HWND* phWindow;
	int* pScreen_width;
	int* pScreen_heigth;

	ID3D11Device* d3dDevice = NULL;
	ID3D11DeviceContext* d3dContext = NULL;
	IDXGISwapChain* swapChain = NULL;
#ifdef _DEBUG
	ID3D11Debug* d3dDebug = NULL;
#endif // _DEBUG

	std::map<std::string, PointCloud*> meshDict; 


private: 


	ID3D11RenderTargetView* renderTargetView = NULL;
	ID3D11DepthStencilView* depthStencilView = NULL;
	ID3D11Texture2D* depthStencilBuffer = NULL;

	XMFLOAT4X4 *m_Proj;

	std::list<PointCloud*> transparents;
};

