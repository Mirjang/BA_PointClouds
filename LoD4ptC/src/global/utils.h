#pragma once
#ifndef UTIL_H
#define UTIL_H

#define SafeRelease(ptr)     if (ptr){(ptr)->Release(); ptr = NULL;}




//Render modes
enum SplatType {
	QUAD_SPLAT, CIRCLE_SPLAT, ELLIPTIC_SPLAT
};



//Settings to be input via AntTweakBar
struct RenderSettings {
	int lod = 0;	//LOD to be rendered
	SplatType renderMode = SplatType::QUAD_SPLAT;
	float splatSize = 0.0001f;
	bool useLight = true; 
};
extern RenderSettings g_renderSettings;


struct screenParams {
	int width;
	int height;
	float nearPlane;
	float farPlane;
};

#endif // !UTIL_H
