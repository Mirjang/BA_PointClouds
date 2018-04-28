#pragma once

#include <windows.h>
#include <windowsx.h>


// DirectX includes

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>

#include <DirectXMath.h>

#include "RenderEffect.h"

#include <list>
#include <map>

#include "../ressources/RessourceLoader.h"
#include "./PointCloud.h"
#include "../global/utils.h"


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
	void reloadShaders(RenderMode mode); 


	/**
	*	###	Runtime Function Prototypes	###
	*/
	void newFrame(XMFLOAT4X4* _m_View, XMFLOAT4X4*  _m_Proj);							//Signals a new Frame, clears screen
	void render(const std::string& meshName, const XMFLOAT4X4*  m_World);		//Renders 1 GO to screen
	void endFrame();

	void setSplatSize(float size)
	{
		cbPerObj.splatRadius = size;
		cbPerObj.splatDiameter = size + size;
	}

	void setLight(const XMVECTOR& pos, const XMVECTOR& color)
	{
		XMStoreFloat3(&cbPerObj.lightpos,pos); 
		XMStoreFloat3(&cbPerObj.lightcolor, color); 
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

	ID3D11RasterizerState* stateRS_default = NULL;
	ID3D11RasterizerState* stateRS_CCW = NULL;		//backface rendering


	ID3D11RenderTargetView* renderTargetView = NULL;
	ID3D11DepthStencilView* depthStencilView = NULL;
	ID3D11Texture2D* depthStencilBuffer = NULL;

	ID3D11InputLayout* vertexLayout = NULL;

	ID3D11VertexShader* VS = NULL;
	ID3DBlob* VS_buffer = NULL;
	ID3D11PixelShader* PS = NULL;
	ID3DBlob* PS_buffer = NULL;
	
	ID3D11GeometryShader* GS = NULL;
	ID3DBlob* GS_buffer = NULL;

	/**
	*	###	Runtime Variables	###
	*/
	__declspec(align(16))
	struct cbPerObject
	{
		XMFLOAT4X4 m_wvp;
		XMFLOAT3 lightpos; 
		XMFLOAT3 lightcolor; 
		float splatRadius; 
		float splatDiameter; 
	};

	cbPerObject cbPerObj;

	ID3D11Buffer* cbPerObjectBuffer = NULL;

	XMFLOAT4X4 *m_View, *m_Proj;
	std::list<PointCloud*> transparents;

	RenderEffect* g_renderEffect = nullptr; 
};

