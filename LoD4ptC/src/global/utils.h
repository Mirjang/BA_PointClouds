#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <cassert>

#define SafeRelease(ptr)     if (ptr){(ptr)->Release(); ptr = NULL;}




//Render modes
enum RenderMode {
	QUAD_SPLAT, CIRCLE_SPLAT, ELLIPTIC_SPLAT
};



//Settings to be input via AntTweakBar
struct RenderSettings {
	int lod = 0;	//LOD to be rendered
	RenderMode renderMode = RenderMode::QUAD_SPLAT;
	float splatSize = 0.0001f;

};



struct screenParams {
	int width;
	int height;
	float nearPlane;
	float farPlane;
};

#endif // !UTIL_H
