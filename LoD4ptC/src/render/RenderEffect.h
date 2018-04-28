#pragma once
#pragma comment (lib, "d3d11.lib")
#include <d3d11.h>
#pragma comment (lib, "D3DCompiler.lib")
#include <D3DCompiler.h>

#include <cassert>

#include <d3dx11effect.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "../global/utils.h"


// Convenience macros for safe effect variable retrieval
// Copied from GED project
#define SAFE_GET_PASS(Technique, name, var)   {assert(Technique!=NULL); var = Technique->GetPassByName( name );						assert(var->IsValid());}
#define SAFE_GET_TECHNIQUE(effect, name, var) {assert(effect!=NULL); var = effect->GetTechniqueByName( name );						assert(var->IsValid());}
#define SAFE_GET_SCALAR(effect, name, var)    {assert(effect!=NULL); var = effect->GetVariableByName( name )->AsScalar();			assert(var->IsValid());}
#define SAFE_GET_VECTOR(effect, name, var)    {assert(effect!=NULL); var = effect->GetVariableByName( name )->AsVector();			assert(var->IsValid());}
#define SAFE_GET_MATRIX(effect, name, var)    {assert(effect!=NULL); var = effect->GetVariableByName( name )->AsMatrix();			assert(var->IsValid());}
#define SAFE_GET_SAMPLER(effect, name, var)   {assert(effect!=NULL); var = effect->GetVariableByName( name )->AsSampler();			assert(var->IsValid());}
#define SAFE_GET_RESOURCE(effect, name, var)  {assert(effect!=NULL); var = effect->GetVariableByName( name )->AsShaderResource();	assert(var->IsValid());}


struct RenderEffect
{
	ID3DX11Effect*                          effect = nullptr; // The whole rendering effect
	std::unordered_map<std::string, ID3DX11EffectTechnique*>	techniques; 



	
	//loads .fx file and creates shaders
	RenderEffect(ID3D11Device* device)
	{
		HRESULT hr; 
		
		D3DX11CreateEffectFromFile(L"./shaders/effects.fx", 0, device, &effect);

		assert(effect->IsValid());


	}

	~RenderEffect() { if (effect) effect->Release(); }
};
