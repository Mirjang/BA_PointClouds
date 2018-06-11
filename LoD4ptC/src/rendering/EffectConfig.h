#pragma once



#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>

#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>
#include <vector>
#include <unordered_map>

/*
* Ok, so namespaces suck... lots of linking errors if you wanna define everything in one file and all the workarounds are ugly
* at least this way i have all of my hardcoded shader names in a seperate file.
*
* !!! This file should only be included in Effects.cpp
*/


namespace Effects
{
	namespace config {
		const wchar_t SHADER_FILE[] = L"./shaders/effects.hlsl";

		const std::vector<std::string> VS_NAMES =
		{
			"VS_PASSTHROUGH", 
			"VS_APPLY_DEPTHCOLOR", 
			"VS_ELLIPTICAL_PASSTHROUGH", 
			"VS_ELLIPTICAL_APPLY_DEPTHCOLOR", 
		};
		const std::vector<std::string> GS_NAMES =
		{
			"GS_UNLIT",
			"GS_LIT",
			"GS_UNLIT_ADAPTIVESPLATSIZE",
			"GS_LIT_ADAPTIVESPLATSIZE",
			"GS_UNLIT_ADAPTIVESPLATSIZE_DEPTHCOLOR",
			"GS_LIT_ADAPTIVESPLATSIZE_DEPTHCOLOR",
			"GS_ELLIPTICAL",
		};
		const std::vector<std::string> PS_NAMES =
		{
			"PS_QUAD_NOLIGHT",
			"PS_CIRCLE_NOLIGHT",
			"PS_QUAD_PHONG",
			"PS_CIRCLE_PHONG",
			"PS_QUADELLIPSE_NOLIGHT",
			"PS_QUADELLIPSE_PHONG", 
			"PS_ELLIPSE_NOLIGHT", 
			"PS_ELLIPSE_PHONG", 
		};

		const D3D11_INPUT_ELEMENT_DESC LAYOUT_POS3_NOR3_COL4[] =
		{
		{ "SV_POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		};

		const D3D11_INPUT_ELEMENT_DESC LAYOUT_POS3_NOR3_COL4_AXIS3_AXIS3[] =
		{
		{ "SV_POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "TANGENT", 1, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },

		};

		const int layoutSize = ARRAYSIZE(LAYOUT_POS3_NOR3_COL4); 
		const int layoutSizeElliptical = ARRAYSIZE(LAYOUT_POS3_NOR3_COL4_AXIS3_AXIS3);
	}

}
