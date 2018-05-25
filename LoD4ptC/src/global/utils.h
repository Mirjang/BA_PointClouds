#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <DirectXMath.h>
#include <AntTweakBar.h>

#define SafeRelease(ptr)     if (ptr){(ptr)->Release(); ptr = NULL;}

//Render modes
enum SplatType
{
	QUAD_SPLAT, CIRCLE_SPLAT, ELLIPTIC_SPLAT
};

enum LODMode
{
	NONE,
	OCTREE_NAIVE,
	NESTED_OCTREE_NAIVE,
	NESTED_OCTREE_POSSIONDISK,
	K_MEANS, 
	EGGS
};

struct Statistics
{
	float framesPerSec = 24;	//hehe 24 -- pointless value
	int verticesDrawn = 0; 
};

extern Statistics g_statistics; 

//set by AntTweakBar
struct UserInput
{
	bool showSceneMenu = false;
	bool showRenderMenu = true;
	bool showLODMenu = false;


	float cameraSpeed = 800.0f;
	float camRotateSpeed = 1.025f;

	DirectX::XMFLOAT4 objectRotation;
	DirectX::XMFLOAT3 lightDirection;
	DirectX::XMFLOAT3 lightColor = { 1,1,1 };

	bool button_lmb = false;
	bool button_rmb = false;

	bool reloadShaders = false;
	bool orbitCam = false;
	bool resetCamera = false;

	//Housekeeper
	float lastMouseX = 0;
	float lastMouseY = 0;
};

extern UserInput g_userInput;


//set by AntTweakBar
struct RenderSettings 
{
	int lod = 0;	//LOD to be rendered
	SplatType renderMode = SplatType::QUAD_SPLAT;
	float splatSize = 0.0100f;
	bool useLight = false; 
};
extern RenderSettings g_renderSettings;

//set by AntTweakBar
struct LODSettings
{
	LODMode mode = LODMode::NESTED_OCTREE_POSSIONDISK;
	int pixelThreshhold = 1; 


	bool recreate = false; 

	//housekeeper
	LODMode lastMode = LODMode::NONE;
	TwBar* twImplSettingsBar = nullptr; 
};

extern LODSettings g_lodSettings; 

struct ScreenParams 
{
	int width;
	int height;
	float nearPlane;
	float farPlane;
	float fov; 
};

extern ScreenParams g_screenParams; 

#endif // !UTIL_H
