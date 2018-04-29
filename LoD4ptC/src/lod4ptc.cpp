#include "lod4ptc.h"

/**
*	### Entry Point	###
*/



int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{

	//debug console
	AllocConsole();
	freopen("CONIN$", "r", stdin);
	freopen("CONOUT$", "w", stdout);
	freopen("CONOUT$", "w", stderr);




	g_hInstance = hInstance;

	g_ScreenParams.width = 1680;
	g_ScreenParams.height = 1050;
	g_ScreenParams.nearPlane = 0.01f;
	g_ScreenParams.farPlane = 1000.0f;


	g_RessourceLoader = new RessourceLoader(); 

	initWindow();

	g_Renderer = new Renderer(&g_hWindow, &g_ScreenParams.width, &g_ScreenParams.height);

	g_Renderer->initialize();

	/**
	*	--- setup tweak bar -----------
	*/
	TwInit(TwGraphAPI::TW_DIRECT3D11, g_Renderer->d3dDevice);
	TwWindowSize(g_ScreenParams.width, g_ScreenParams.height);
	TwBar* menuBar = TwNewBar("Settings");
	TwBar* sceneSettings = TwNewBar("SceneSettings");
	TwBar* renderSettings = TwNewBar("RenderSettings"); 

	TwAddVarRO(renderSettings, "Level of Detail", TW_TYPE_UINT32, &g_renderSettings.lod, "readonly=false");
	TwEnumVal splatTypeEV[] = {{ SplatType::QUAD_SPLAT, "Quad-Splats"}, { SplatType::CIRCLE_SPLAT, "Circle-Splats"}, { SplatType::ELLIPTIC_SPLAT, "Ellipse-Splats"}};
	TwType twRenderMode = TwDefineEnum("Splat Type", splatTypeEV, 3);
	TwAddVarRW(renderSettings, "Render Mode", twRenderMode, &g_renderSettings.renderMode, NULL);
	TwAddVarRW(renderSettings, "Splat size", TW_TYPE_FLOAT, &g_renderSettings.splatSize, "min=0 max=5 step=0.0001");
	TwAddVarRW(renderSettings, "Use Light", TW_TYPE_BOOLCPP, &g_renderSettings.useLight, "");
	TwAddVarRW(renderSettings, "Reload Shaders", TW_TYPE_BOOLCPP, &g_userInput.reloadShaders, "");


	TwAddVarRW(sceneSettings, "OrbitCamera", TW_TYPE_BOOLCPP, &g_userInput.orbitCam, "");

	TwAddVarRW(sceneSettings, "Camera Speed", TW_TYPE_FLOAT, &g_userInput.cameraSpeed, "min=1 max=1000 step=5");
	TwAddVarRW(sceneSettings, "Camera Rotate Speed", TW_TYPE_FLOAT, &g_userInput.camRotateSpeed, "min=0.005 max=2 step=0.005");
	TwAddVarRW(sceneSettings, "Light Position", TW_TYPE_DIR3F, &light.pos, ""); 
	TwAddVarRW(sceneSettings, "Light Position", TW_TYPE_COLOR3F, &g_userInput.lightColor, "");

	TwAddVarRW(sceneSettings, "Object Rotation", TW_TYPE_QUAT4F, &g_userInput.objectRotation, "");

	TwAddVarRW(sceneSettings, "ResetCamera", TW_TYPE_BOOLCPP, &g_userInput.resetCamera, "");



	TwAddVarRO(menuBar, "Frames per Sec", TW_TYPE_FLOAT, &framesPerSec, "");



	DragAcceptFiles(g_hWindow, TRUE);

	g_Renderer->reloadShaders(); 

	/*
	* ----------- Init scene ----------
	*/
	
	g_camera = new Camera(&g_ScreenParams.width, &g_ScreenParams.height, g_ScreenParams.nearPlane, g_ScreenParams.farPlane); 
	g_camera->translateAbs(0.1f, 1.0f, -75.0f); 


	light.pos = { -100, 100, 0,0 }; 

	/*
	--------------- TEST -----------
	*/
	//GameObject* dragon = new GameObject("dragon_perlin_color.ply");

	//g_sceneRoot.addChild(dragon); 

	//-------------------


	g_sceneRoot.initialize(g_Renderer);


	auto start = std::chrono::high_resolution_clock::now();

	//render loop
	while (run)
	{
		std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
		start = std::chrono::high_resolution_clock::now();
		g_deltaTime = elapsed.count(); 

		update(); 



		g_Renderer->newFrame(g_camera->getViewMatrix(), g_camera->getProjectionMatrix());

		input();

		XMFLOAT4X4 identity;
		XMStoreFloat4x4(&identity, XMMatrixIdentity());


		g_sceneRoot._render(g_Renderer, &identity);
		TwDraw();
		g_Renderer->endFrame();

	}

	TwTerminate(); 
	delete g_Renderer; 
	DestroyWindow(g_hWindow); 
	
	std::exit(0);
}


/**
*	### lod4ptc.h impl
*/


void initWindow()
{
	//defining window class parameters
	WNDCLASSEX windowInfo;
	ZeroMemory(&windowInfo, sizeof(WNDCLASSEX));

	windowInfo.cbSize = sizeof(WNDCLASSEX);
	windowInfo.style = CS_HREDRAW | CS_VREDRAW;
	windowInfo.lpfnWndProc = &windowProc;
	windowInfo.hInstance = g_hInstance;
	windowInfo.hbrBackground = (HBRUSH)COLOR_WINDOW;		//remove for fullscreen rendering
	windowInfo.lpszClassName = L"Level of Detail for Point Clouds";
	windowInfo.hCursor = LoadCursor(NULL, IDC_ARROW);
	windowInfo.hIcon = LoadIcon(NULL, IDI_WINLOGO);
	windowInfo.hIconSm = LoadIcon(NULL, IDI_WINLOGO);

	//registering window class
	if (!RegisterClassEx(&windowInfo))
	{
		MessageBox(NULL, L"Failed to register Window class! ", L"ERROR!", MB_OK);
		std::exit(101);
	}

	//creating the window the game is going to be rendered to

	g_hWindow = CreateWindowEx(
		NULL,
		windowInfo.lpszClassName,
		L"Level of Detail for Point Clouds",
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT,
		g_ScreenParams.width, g_ScreenParams.height,
		NULL,
		NULL,
		g_hInstance,
		NULL);

	if (!g_hWindow)
	{
		MessageBox(NULL, L"Failed to create Window! ", L"ERROR!", MB_OK);
		std::exit(102);
	}


	ShowWindow(g_hWindow, true);

	UpdateWindow(g_hWindow);
}


//Callback function in case of window events
LRESULT CALLBACK windowProc(HWND hWindow, UINT message, WPARAM wParam, LPARAM lParam)
{

	if (message == WM_DESTROY)
	{
		// close the application entirely
		PostQuitMessage(0);
		return 0;
	}


	return DefWindowProc(hWindow, message, wParam, lParam);
}


void update()
{
	g_Renderer->setSplatSize(g_renderSettings.splatSize); 

	g_Renderer->setLight(XMLoadFloat4(&light.pos), XMLoadFloat3(&g_userInput.lightColor));

	if (g_activeObject)
	{
		g_activeObject->rot = g_userInput.objectRotation; 
	}

	if (g_userInput.resetCamera)
	{
		g_userInput.resetCamera = false; 
		delete g_camera; 
		g_camera = new Camera(&g_ScreenParams.width, &g_ScreenParams.height, g_ScreenParams.nearPlane, g_ScreenParams.farPlane);
		g_camera->translateAbs(0.1f, 1.0f, -75.0f);
	}

	if (g_userInput.reloadShaders)
	{
		g_userInput.reloadShaders = false; 
		g_Renderer->reloadShaders(); 
	}

	//camera mode: wasd or orbit
	g_camera->setTarget(g_userInput.orbitCam?g_activeObject:nullptr);





	framesPerSec = 1 / g_deltaTime; 


}

void input()
{
	MSG msg;

	while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
	{
		UINT message = msg.message;



		if (TwEventWin(msg.hwnd, msg.message, msg.wParam, msg.lParam))
		{
			continue; 
		}


		switch (message)
		{
		case WM_KEYDOWN:
		{
			evaluateKeyboardInput(msg.wParam, true);
			break;
		}
		case WM_KEYUP:
		{
			evaluateKeyboardInput(msg.wParam, false);
			break;
		}
		case WM_LBUTTONDOWN: 
		{
			g_userInput.button_lmb = true; 
			break; 
		}
		case WM_LBUTTONUP:
		{
			g_userInput.button_lmb = false;
			break;
		}
		case WM_MOUSEMOVE:
		{
			float mouseX = static_cast<float>(GET_X_LPARAM(msg.lParam));
			float mouseY = static_cast<float>(GET_Y_LPARAM(msg.lParam));

			if (g_userInput.button_lmb && !g_userInput.orbitCam)
			{
				g_camera->rotate(( g_userInput.lastMouseY- mouseY) * g_userInput.camRotateSpeed * g_deltaTime, (g_userInput.lastMouseX - mouseX) * g_userInput.camRotateSpeed* g_deltaTime, 0);
			}

			g_userInput.lastMouseX = mouseX;
			g_userInput.lastMouseY = mouseY;

			break;
		}
		
		case WM_DROPFILES:
		{
			wchar_t wFilename[MAX_PATH]; 
			if (DragQueryFile((HDROP)msg.wParam, 0, wFilename, MAX_PATH))
			{
				std::wstring wsFilename = std::wstring(wFilename); 
				std::string filename(wsFilename.begin(), wsFilename.end());

				std::cout << "Loading file: "<< filename << std::endl;
				
				//todo: start load thread
				if (g_activeObject)
				{
					g_sceneRoot.children.remove(g_activeObject);
					delete g_activeObject;
				}


				g_activeObject = new GameObject(filename);
				g_activeObject->initialize(g_Renderer);
				g_sceneRoot.addChild(g_activeObject);
				DragFinish((HDROP)msg.wParam); 
			}
			else 
			{
				std::cout << "Failed to open file" << std::endl;
			}

			break; 
		}
		// Programm Termination Message
		case WM_DESTROY:
		{
			PostQuitMessage(0);
			break;
		}
		case WM_QUIT:
		{
			run = false;
			break;
		}
		}

		TranslateMessage(&msg);

		DispatchMessage(&msg);
	}

//	SetCursorPos(screen_width / 2, screen_heigth / 2);
	PeekMessage(&msg, NULL, 0, 0, PM_REMOVE);
}

//Evaluate incoming keyboard and mouse events
void evaluateKeyboardInput(WPARAM key, bool down)
{
	

	switch (key)
	{

	case VK_ESCAPE:
	{
		run = false;
		break;
	}

	case 0x57: //w
	{
		g_camera->move(0,0, g_userInput.cameraSpeed * g_deltaTime); 
		break; 
	}

	case 0x53: //s
	{
		g_camera->move(0, 0, -g_userInput.cameraSpeed * g_deltaTime);
		break;
	}

	case 0x41: //a
	{
		g_camera->move(-g_userInput.cameraSpeed * g_deltaTime, 0, 0);

		break;
	}

	case 0x44: //d
	{
		g_camera->move(g_userInput.cameraSpeed * g_deltaTime, 0, 0);
		break;
	}

	}
}
