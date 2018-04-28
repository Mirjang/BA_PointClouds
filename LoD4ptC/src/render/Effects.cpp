#include "Effects.h"

#include <sstream>

#define RS_FAILED(hr) 	if (FAILED(hr))\
	{\
	std::cout << "failed to create RS state " << std::endl;\
	std::cout << (char*)errBlob->GetBufferPointer() << std::endl;\
	std::cin.get();\
	}
	

using namespace Effects; 

Pass* Pass::create(std::string VSname, std::string PSname, std::string GSname, ID3D11RasterizerState* RSstate)
{

	auto vs = vertexShaders.find(VSname); 
	if (vs == vertexShaders.end())
	{
		std::cerr << "Could not find VS: " << VSname << std::endl; 
		return nullptr; 
	}
	VS = vs->second; 

	auto ps = pixelShaders.find(PSname);
	if (ps == pixelShaders.end())
	{
		std::cerr << "Could not find PS: " << PSname << std::endl;
		return nullptr;
	}
	PS = ps->second;

	auto gs = geometryShaders.find(GSname);
	if (ps != pixelShaders.end())
	{
		GS = gs->second; 
	}
	else
	{	//every splat should prob. use a GS but reusability and stuff
		std::cout << "Created pass without GS: " << GSname << std::endl; 
	}

	RS_STATE = RSstate; 




}


void Effects::init(ID3D11Device* device)
{
	HRESULT hr;
	ID3DBlob *errBlob;

	D3D11_RASTERIZER_DESC descRasterizer;
	ZeroMemory(&descRasterizer, sizeof(D3D11_RASTERIZER_DESC));
	descRasterizer.FillMode = D3D11_FILL_SOLID;
	descRasterizer.CullMode = D3D11_CULL_BACK;
	descRasterizer.FrontCounterClockwise = false;
	descRasterizer.DepthBias = 0;
	descRasterizer.SlopeScaledDepthBias = 0.0f;
	descRasterizer.DepthBiasClamp = 0.0f;
	descRasterizer.DepthClipEnable = true;
	descRasterizer.ScissorEnable = false;
	descRasterizer.MultisampleEnable = true;
	descRasterizer.AntialiasedLineEnable = true;

	hr = device->CreateRasterizerState(&descRasterizer, &RS_STATE.CULL_BACK_CCW);
	RS_FAILED(hr);

	descRasterizer.CullMode = D3D11_CULL_FRONT;
	hr = device->CreateRasterizerState(&descRasterizer, &RS_STATE.CULL_FRONT_CCW);
	RS_FAILED(hr);

	descRasterizer.FrontCounterClockwise = true;
	hr = device->CreateRasterizerState(&descRasterizer, &RS_STATE.CULL_FRONT_CW);
	RS_FAILED(hr);

	descRasterizer.CullMode = D3D11_CULL_BACK;
	hr = device->CreateRasterizerState(&descRasterizer, &RS_STATE.CULL_BACK_CW);
	RS_FAILED(hr);

	descRasterizer.CullMode = D3D11_CULL_NONE;
	hr = device->CreateRasterizerState(&descRasterizer, &RS_STATE.CULL_NONE);
	RS_FAILED(hr);

	int bufferctr = 0; 
	intern::shaderblobs.resize(intern::GS_NAMES.size() + intern::GS_NAMES.size() + intern::PS_NAMES.size()); //alloc space for all shader blobs

	//load vertex shaders
	for each (const std::string& name in intern::VS_NAMES)
	{
		hr = D3DCompileFromFile(intern::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "vs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &intern::shaderblobs[bufferctr], &errBlob);
		if (FAILED(hr))
		{
			std::cout << "failed to compile vertex shader " << std::endl;
			std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
			std::cin.get();
		}

		ID3D11VertexShader* shader;
		hr = device->CreateVertexShader(intern::shaderblobs[bufferctr]->GetBufferPointer(), intern::shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
		if (FAILED(hr))
		{
			std::cout << "failed to load VertexShader " << hr << std::endl;
			std::cin.get();
		}

		vertexShaders.insert(std::pair<std::string, ID3D11VertexShader*>(name, shader));

		//create input layout based on 1st vertex shader... every shader gets same input... kinda dumb but CreateInputLayout requires a shader blob
		if (!bufferctr)
		{
			hr = device->CreateInputLayout(intern::LAYOUT_POS3_NOR3_COL4, ARRAYSIZE(intern::LAYOUT_POS3_NOR3_COL4), intern::shaderblobs[bufferctr]->GetBufferPointer(), intern::shaderblobs[bufferctr]->GetBufferSize(), &intern::layout);
			if (FAILED(hr))
			{
				std::cout << "failed to create InputLayout " << hr << std::endl;
				std::cin.get();
			}
		}



		bufferctr++; 
	}

	//load geometry shaders
	for each (const std::string& name in intern::GS_NAMES)
	{
		hr = D3DCompileFromFile(intern::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "gs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &intern::shaderblobs[bufferctr], &errBlob);
		if (FAILED(hr))
		{
			std::cout << "failed to compile geometry shader " << std::endl;
			std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
			std::cin.get();
		}

		ID3D11GeometryShader* shader;
		hr = device->CreateGeometryShader(intern::shaderblobs[bufferctr]->GetBufferPointer(), intern::shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
		if (FAILED(hr))
		{
			std::cout << "failed to load VertexShader " << hr << std::endl;
			std::cin.get();
		}

		geometryShaders.insert(std::pair<std::string, ID3D11GeometryShader*>(name, shader));
		bufferctr++;
	}

	//load pixel shaders
	for each (const std::string& name in intern::PS_NAMES)
	{
		hr = D3DCompileFromFile(intern::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "ps_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &intern::shaderblobs[bufferctr], &errBlob);
		if (FAILED(hr))
		{
			std::cout << "failed to compile pixel shader " << std::endl;
			std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
			std::cin.get();
		}

		ID3D11PixelShader* shader;
		hr = device->CreatePixelShader(intern::shaderblobs[bufferctr]->GetBufferPointer(), intern::shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
		if (FAILED(hr))
		{
			std::cout << "failed to load VertexShader " << hr << std::endl;
			std::cin.get();
		}

		pixelShaders.insert(std::pair<std::string, ID3D11PixelShader*>(name, shader));
		bufferctr++;
	}
	
}


void Effects::deinit()
{
	SafeRelease(intern::layout); 

	SafeRelease(RS_STATE.CULL_FRONT_CW);
	SafeRelease(RS_STATE.CULL_BACK_CW);
	SafeRelease(RS_STATE.CULL_FRONT_CCW);
	SafeRelease(RS_STATE.CULL_BACK_CCW);
	SafeRelease(RS_STATE.CULL_NONE);

	for (auto o : vertexShaders)
	{
		SafeRelease(o.second);
	}
	for (auto o : geometryShaders)
	{
		SafeRelease(o.second);
	}
	for (auto o : pixelShaders)
	{
		SafeRelease(o.second);
	}

	for (auto o : intern::shaderblobs)
	{
		SafeRelease(o);
	}

}