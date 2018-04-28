#pragma once
#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>

#include <iostream>
#include <vector>
#include <unordered_map>

#include "../global/utils.h"


/*
* So apperently the whole d3dx11 effects stuff is deprecated and Effects11 is only
* supposed to work as a backwards compability lib. So I guess I´ll have to improvise my own version?
*/


namespace Effects
{

	namespace intern {
		std::vector<ID3DBlob*> shaderblobs;

		const wchar_t SHADER_FILE[] = L"./shaders/effects.fx";

		const std::vector<std::string> VS_NAMES = 
		{

		};
		const std::vector<std::string> GS_NAMES =
		{

		};
		const std::vector<std::string> PS_NAMES =
		{

		};

		const D3D11_INPUT_ELEMENT_DESC LAYOUT_POS3_NOR3_COL4 [] =
		{
			{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
			{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		};

		ID3D11InputLayout* layout; 
	}

	/*pretty sure I´ll only use those*/

	std::unordered_map<std::string, ID3D11VertexShader*> vertexShaders; 
	std::unordered_map<std::string, ID3D11GeometryShader*> geometryShaders;
	std::unordered_map<std::string, ID3D11PixelShader*> pixelShaders;

	struct st_RS_STATES 
	{
		ID3D11RasterizerState* CULL_FRONT_CW;
		ID3D11RasterizerState* CULL_BACK_CW;
		ID3D11RasterizerState* CULL_FRONT_CCW; 
		ID3D11RasterizerState* CULL_BACK_CCW;
		ID3D11RasterizerState* CULL_NONE;
	}RS_STATE;


	//Inits pipline states and loads shaders defined in intern::XX_NAMES from intern::SHADER_LOCATION
	void init(ID3D11Device* device);

	//releases all com objects
	void deinit();

	struct Pass
	{
		ID3D11VertexShader* VS = nullptr; 
		ID3D11GeometryShader* GS = nullptr; 
		ID3D11PixelShader* PS = nullptr; 

		ID3D11RasterizerState* RS_STATE = nullptr; 

		void apply(ID3D11DeviceContext* const context)
		{
			context->IASetInputLayout(intern::layout); 
			context->RSSetState(RS_STATE);

			context->VSSetShader(VS, nullptr, 0); 
			context->PSSetShader(PS, nullptr, 0);
			if(GS)
				context->GSSetShader(GS, nullptr, 0);

		}

		Pass* create(std::string VSname, std::string PSname, std::string GSname, ID3D11RasterizerState* RSstate); 
		~Pass() {}; 

	private:
		Pass(); 

	};


	//should i ever need to do multi-pass rendering
	struct Technique
	{
		std::vector<Pass> passes; 
	};

}