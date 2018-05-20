#pragma once 
#include <Windows.h>
#include <windowsx.h>

#include <iostream>
#include <sstream>
#include <wchar.h>
#include <string>
#include <chrono>

#include <AntTweakBar.h>

#include "global\utils.h"
#include "rendering\Renderer.h"
#include "rendering\PointCloud.h"
#include "model\Camera\Camera.h"
#include "ressources\RessourceLoader.h"

//different LOD implementations
#include "lod\LodImplementations.h"

/**
*	###	Init Function Prototypes	###
*/

/*	---Window Initialisation---	*/
inline void initWindow();	//initializes the WINAPI Window used to display our game
LRESULT CALLBACK windowProc(HWND hWindow, UINT message, WPARAM wp, LPARAM lp);	//CALLBACK Function for Keyboard and Mouse events


/**
*	###	Runtime Function Prototypes	###
*/

inline void input(); //reads system messages and calls below functions to evaluate input
void evaluateKeyboardInput(WPARAM key, bool down);


inline void update(); //updates objects per frame


/**
*	###  Init Variables	###
*/
HINSTANCE g_hInstance; //Handle for this process... required for creating a window
HWND g_hWindow;	//Handle for the game window (filled below)

// ALL THE GLOBALS
ScreenParams g_screenParams; 
RenderSettings g_renderSettings; 
LODSettings g_lodSettings;
Statistics g_statistics;
UserInput g_userInput;
namespace Effects 
{
	Pass* g_pCurrentPass;
}

/**
*	###	Runtime Variables	###
*/
bool run = true;	//indicates when to stop the game loop
double lastFrameTime = 0;	//used to calculate deltaTimeMillis

GameObject g_sceneRoot; 
GameObject* g_activeObject = nullptr; 
RessourceLoader* g_RessourceLoader;

Renderer* g_renderer; 
Camera* camera; 

double g_deltaTime = 0.0f; 







