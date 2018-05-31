#include "Effects.h"

#include "EffectConfig.h"	//this file should really only be used here for reasons of bad coding


#define RS_FAILED(hr) 	if (FAILED(hr))\
	{\
	std::cout << "failed to create RS state " << std::endl;\
	std::cin.get();\
	}
	
namespace Effects
{
	namespace config {
		extern std::vector<ID3DBlob*> shaderblobs;

		extern const wchar_t SHADER_FILE[];

		extern const std::vector<std::string> VS_NAMES;
		extern const std::vector<std::string> GS_NAMES;
		extern const std::vector<std::string> PS_NAMES;
		extern const D3D11_INPUT_ELEMENT_DESC LAYOUT_POS3_NOR3_COL4[];
		extern ID3D11InputLayout* layout;
	}

	st_RS_STATES RS_STATE;
	PerObject cbPerObj;
	ShaderSettings cbShaderSettings;
	PerLOD cbPerLOD; 


	ID3D11Buffer* cbPerObjectBuffer = NULL;
	ID3D11Buffer* cbPerLODBuffer = NULL;
	ID3D11Buffer* cbShaderSettingsBuffer = NULL;

	std::unordered_map<std::string, ID3D11VertexShader*> vertexShaders;
	std::unordered_map<std::string, ID3D11GeometryShader*> geometryShaders;
	std::unordered_map<std::string, ID3D11PixelShader*> pixelShaders;

	ID3D11InputLayout* layout;
	std::vector<ID3DBlob*> shaderblobs;

	Pass* createPass(std::string VSname, std::string PSname, std::string GSname, ID3D11RasterizerState* RSstate)
	{
		Pass* pass = new Pass();


		auto vs = vertexShaders.find(VSname);
		if (vs == vertexShaders.end())
		{
			std::cerr << "Could not find VS: " << VSname << std::endl;
			return nullptr;
		}
		pass->VS = vs->second;

		auto ps = pixelShaders.find(PSname);
		if (ps == pixelShaders.end())
		{
			std::cerr << "Could not find PS: " << PSname << std::endl;
			return nullptr;
		}
		pass->PS = ps->second;

		auto gs = geometryShaders.find(GSname);
		if (ps != pixelShaders.end())
		{
			pass->GS = gs->second;
		}
		else
		{	//every splat should prob. use a GS but reusability and stuff
			std::cout << "No GS set - continuing " << GSname << std::endl;
		}

		pass->RS_STATE = RSstate;

		std::cout << "Created pass:" <<VSname << " " << GSname << " " << PSname << std::endl;


		return pass;
	}

	void init(ID3D11Device* device)
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
		shaderblobs.resize(config::GS_NAMES.size() + config::GS_NAMES.size() + config::PS_NAMES.size()); //alloc space for all shader blobs

		//load vertex shaders
		for each (const std::string& name in config::VS_NAMES)
		{
			hr = D3DCompileFromFile(config::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "vs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &shaderblobs[bufferctr], &errBlob);
			if (FAILED(hr))
			{
				std::cout << "failed to compile vertex shader " << std::endl;
				std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
				std::cin.get();
			}

			ID3D11VertexShader* shader;
			hr = device->CreateVertexShader(shaderblobs[bufferctr]->GetBufferPointer(), shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
			if (FAILED(hr))
			{
				std::cout << "failed to load VertexShader " << hr << std::endl;
				std::cin.get();
			}

			vertexShaders.insert(std::pair<std::string, ID3D11VertexShader*>(name, shader));

			//create input layout based on 1st vertex shader... every shader gets same input... kinda dumb but CreateInputLayout requires a shader blob
			if (!bufferctr)
			{
				hr = device->CreateInputLayout(config::LAYOUT_POS3_NOR3_COL4, config::layoutSize, shaderblobs[bufferctr]->GetBufferPointer(), shaderblobs[bufferctr]->GetBufferSize(), &layout);
				if (FAILED(hr))
				{
					std::cout << "failed to create InputLayout " << hr << std::endl;
					std::cin.get();
				}
			}

			bufferctr++;
		}

		//load geometry shaders
		for each (const std::string& name in config::GS_NAMES)
		{
			hr = D3DCompileFromFile(config::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "gs_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &shaderblobs[bufferctr], &errBlob);
			if (FAILED(hr))
			{
				std::cout << "failed to compile geometry shader " << std::endl;
				std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
				std::cin.get();
			}

			ID3D11GeometryShader* shader;
			hr = device->CreateGeometryShader(shaderblobs[bufferctr]->GetBufferPointer(), shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
			if (FAILED(hr))
			{
				std::cout << "failed to load VertexShader " << hr << std::endl;
				std::cin.get();
			}

			geometryShaders.insert(std::pair<std::string, ID3D11GeometryShader*>(name, shader));
			bufferctr++;
		}

		//load pixel shaders
		for each (const std::string& name in config::PS_NAMES)
		{
			hr = D3DCompileFromFile(config::SHADER_FILE, NULL, D3D_COMPILE_STANDARD_FILE_INCLUDE, name.c_str(), "ps_5_0", D3DCOMPILE_DEBUG | D3DCOMPILE_ENABLE_STRICTNESS, NULL, &shaderblobs[bufferctr], &errBlob);
			if (FAILED(hr))
			{
				std::cout << "failed to compile pixel shader " << std::endl;
				std::cout << (char*)errBlob->GetBufferPointer() << std::endl;
				std::cin.get();
			}

			ID3D11PixelShader* shader;
			hr = device->CreatePixelShader(shaderblobs[bufferctr]->GetBufferPointer(), shaderblobs[bufferctr]->GetBufferSize(), NULL, &shader);
			if (FAILED(hr))
			{
				std::cout << "failed to load VertexShader " << hr << std::endl;
				std::cin.get();
			}

			pixelShaders.insert(std::pair<std::string, ID3D11PixelShader*>(name, shader));
			bufferctr++;
		}

	}


	void deinit()
	{
		SafeRelease(layout);

		SafeRelease(RS_STATE.CULL_FRONT_CW);
		SafeRelease(RS_STATE.CULL_BACK_CW);
		SafeRelease(RS_STATE.CULL_FRONT_CCW);
		SafeRelease(RS_STATE.CULL_BACK_CCW);
		SafeRelease(RS_STATE.CULL_NONE);

		for (auto o : vertexShaders)
		{
			SafeRelease(o.second);
		}
		vertexShaders.clear(); 

		for (auto o : geometryShaders)
		{
			SafeRelease(o.second);
		}
		geometryShaders.clear(); 

		for (auto o : pixelShaders)
		{
			SafeRelease(o.second);
		}
		pixelShaders.clear(); 

		for (auto o : shaderblobs)
		{
			SafeRelease(o);
		}
		shaderblobs.clear(); 

	}


	void SetSplatSize(const float& size)
	{
		
		cbPerLOD.splatRadius.x = size; 
		cbPerLOD.splatDiameter.x = size + size; 

		cbPerLOD.splatRadius.y = size*cbShaderSettings.aspectRatio;
		cbPerLOD.splatDiameter.y = cbPerLOD.splatRadius.y+ cbPerLOD.splatRadius.y;

	}

	void UpdatePerLODBuffer(ID3D11DeviceContext* const context)
	{
		context->UpdateSubresource(Effects::cbPerLODBuffer, 0, NULL, &Effects::cbPerLOD, 0, 0);

		context->VSSetConstantBuffers(Effects::BufferSlots::bsPerLOD, 1, &Effects::cbPerLODBuffer);
		context->GSSetConstantBuffers(Effects::BufferSlots::bsPerLOD, 1, &Effects::cbPerLODBuffer);
		context->PSSetConstantBuffers(Effects::BufferSlots::bsPerLOD, 1, &Effects::cbPerLODBuffer);
	}

	/*
	********* Pass Impl
	*/

	void Pass::apply(ID3D11DeviceContext* const context)
	{
		context->IASetInputLayout(layout);
		context->RSSetState(RS_STATE);

		context->VSSetShader(VS, nullptr, 0);
		context->PSSetShader(PS, nullptr, 0);
		if (GS)
			context->GSSetShader(GS, nullptr, 0);

	}

}