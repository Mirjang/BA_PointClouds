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
#include "render\Renderer.h"
#include "model\Camera\Camera.h"
#include "ressources\RessourceLoader.h"

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




screenParams g_ScreenParams; 


RenderSettings g_renderSettings; 

/**
*	###	Runtime Variables	###
*/
bool run = true;	//indicates when to stop the game loop
double lastFrameTime = 0;	//used to calculate deltaTimeMillis

GameObject g_sceneRoot; 
GameObject* g_activeObject = nullptr; 
RessourceLoader* g_RessourceLoader;

Renderer* g_Renderer; 
Camera* g_camera; 

double g_deltaTime = 0.0f; 



float framesPerSec = 0.0; 

struct UserInput
{

	float cameraSpeed = 250.0f;
	float camRotateSpeed = 0.025f;

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

UserInput g_userInput; 



