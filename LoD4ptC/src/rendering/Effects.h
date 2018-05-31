#pragma once

#ifndef EFFECTS_H
#define EFFECTS_H

#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>

#include <DirectXMath.h>

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>

#include "../global/utils.h"


/*
* So apperently the whole d3dx11 effects stuff is deprecated and Effects11 is only
* supposed to work as a backwards compability lib. So I guess I´ll have to improvise my own version?
*/


using namespace DirectX; 

namespace Effects
{
	//strct prototypes
	struct Pass;
	struct Technique;


	__declspec(align(16)) struct PerObject
	{
		XMFLOAT4X4 wvpMat;
		XMFLOAT4X4 worldMat;
		XMFLOAT4 lightDir;
		XMFLOAT4 lightColor;
		XMFLOAT4 camPos;
		XMFLOAT4 camDir;
	};


	__declspec(align(16)) struct PerLOD
	{
		XMFLOAT2 splatRadius;
		XMFLOAT2 splatDiameter;
		UINT32 currentLOD;
	};


	__declspec(align(16)) struct ShaderSettings
	{
		XMFLOAT2 splatSize;
		float pixelThreshhold; 
		float screenheightDiv2; 
		float aspectRatio; 
		UINT32 maxLOD; 
	};

	extern PerLOD cbPerLOD; 
	extern PerObject cbPerObj;
	extern ShaderSettings cbShaderSettings;


	enum BufferSlots
	{
		bsPerLOD = 0x00,
		bsPerObject = 0x01, 
		bsShaderSettings = 0x02
	};

	namespace config {
		extern const wchar_t SHADER_FILE[];
		extern const std::vector<std::string> VS_NAMES;
		extern const std::vector<std::string> GS_NAMES;
		extern const std::vector<std::string> PS_NAMES;

		extern const D3D11_INPUT_ELEMENT_DESC LAYOUT_POS3_NOR3_COL4[];
		extern const int layoutSize;
	}

	/*pretty sure I´ll only use those*/

	extern std::unordered_map<std::string, ID3D11VertexShader*> vertexShaders; 
	extern std::unordered_map<std::string, ID3D11GeometryShader*> geometryShaders;
	extern std::unordered_map<std::string, ID3D11PixelShader*> pixelShaders;

	extern ID3D11Buffer* cbPerObjectBuffer;
	extern ID3D11Buffer* cbPerLODBuffer;
	extern ID3D11Buffer* cbShaderSettingsBuffer;

	extern ID3D11InputLayout* layout;
	extern std::vector<ID3DBlob*> shaderblobs;



	struct st_RS_STATES 
	{
		ID3D11RasterizerState* CULL_FRONT_CW;
		ID3D11RasterizerState* CULL_BACK_CW;
		ID3D11RasterizerState* CULL_FRONT_CCW; 
		ID3D11RasterizerState* CULL_BACK_CCW;
		ID3D11RasterizerState* CULL_NONE;
	};
	
	extern st_RS_STATES RS_STATE;


	//Inits pipline states and loads shaders defined in intern::XX_NAMES from intern::SHADER_LOCATION
	extern void init(ID3D11Device* device);

	//releases all com objects
	extern void deinit();


	extern void SetSplatSize(const float& size);
	extern void UpdatePerLODBuffer(ID3D11DeviceContext* const context);


	/*
	*Returns Pass based on given config.. not sure if i should compile shaders here, 
	*for testing tho shaders are compiled once Effects::init() is called which gives immidiate feedback if a shader is bad
	*/
	extern Pass* createPass(std::string VSname, std::string PSname, std::string GSname, ID3D11RasterizerState* RSstate);


	struct Pass
	{
		ID3D11VertexShader* VS = nullptr; 
		ID3D11GeometryShader* GS = nullptr; 
		ID3D11PixelShader* PS = nullptr; 

		ID3D11RasterizerState* RS_STATE = nullptr; 

		void apply(ID3D11DeviceContext* const context);

		~Pass() {}; 
		Pass() {};
//		friend Pass* createPass(std::string VSname, std::string PSname, std::string GSname, ID3D11RasterizerState* RSstate); //doesnt work -.-

	};

	extern Pass* g_pCurrentPass; 


	//should i ever need to do multi-pass rendering
	struct Technique
	{
		std::vector<Pass> passes; 
	};

}

#endif // !EFFECTS_H
