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

	g_screenParams.width = 1680;
	g_screenParams.height = 1050;
	g_screenParams.nearPlane = 0.01f;
	g_screenParams.farPlane = 1000.0f;
	g_screenParams.fov = XMConvertToRadians(45.0f);

	g_RessourceLoader = new RessourceLoader(); 

	initWindow();

	g_renderer = new Renderer(&g_hWindow, &g_screenParams.width, &g_screenParams.height);

	g_renderer->initialize();

	/**
	*	--- setup tweak bar -----------
	*/
	TwInit(TwGraphAPI::TW_DIRECT3D11, g_renderer->d3dDevice);
	TwWindowSize(g_screenParams.width, g_screenParams.height);
	TwBar* twMenuBar = TwNewBar("Menu");
	TwBar* twSceneSettings = TwNewBar("Scene");
	TwBar* twRenderSettings = TwNewBar("Render"); 
	TwBar* twLODSettings = TwNewBar("LOD");


	TwAddVarRW(twRenderSettings, "Level of Detail", TW_TYPE_UINT32, &g_renderSettings.lod, "readonly=false");
	TwEnumVal splatTypeEV[] = {{ SplatType::QUAD_SPLAT, "Quad-Splats"}, { SplatType::CIRCLE_SPLAT, "Circle-Splats"}, { SplatType::ELLIPTIC_SPLAT, "Ellipse-Splats"}};
	TwType twRenderMode = TwDefineEnum("Splat Type", splatTypeEV, ARRAYSIZE(splatTypeEV));
	TwAddVarRW(twRenderSettings, "Render Mode", twRenderMode, &g_renderSettings.renderMode, NULL);
	TwAddVarRW(twRenderSettings, "Splat size", TW_TYPE_FLOAT, &g_renderSettings.splatSize, "min=0 max=5 step=0.0001");
	TwAddVarRW(twRenderSettings, "Use Light", TW_TYPE_BOOLCPP, &g_renderSettings.useLight, "");
	TwAddSeparator(twRenderSettings, "sep", "");
	TwAddVarRW(twRenderSettings, "Apply Settings", TW_TYPE_BOOLCPP, &g_userInput.reloadShaders, "");


	TwAddVarRW(twSceneSettings, "Light Direction", TW_TYPE_DIR3F, &g_userInput.lightDirection, "");
	TwAddVarRW(twSceneSettings, "Light Color", TW_TYPE_COLOR3F, &g_userInput.lightColor, "");
	TwAddVarRW(twSceneSettings, "Object Rotation", TW_TYPE_QUAT4F, &g_userInput.objectRotation, "");
	TwAddSeparator(twSceneSettings, "sep", "");
	TwAddVarRW(twSceneSettings, "OrbitCamera", TW_TYPE_BOOLCPP, &g_userInput.orbitCam, "");
	TwAddVarRW(twSceneSettings, "Camera Speed", TW_TYPE_FLOAT, &g_userInput.cameraSpeed, "min=1 max=1000 step=5");
	TwAddVarRW(twSceneSettings, "Camera Rotate Speed", TW_TYPE_FLOAT, &g_userInput.camRotateSpeed, "min=0.005 max=2 step=0.005");
	TwAddVarRW(twSceneSettings, "Object Rotation", TW_TYPE_QUAT4F, &g_userInput.objectRotation, "");
	TwAddSeparator(twSceneSettings, "sep2", "");
	TwAddVarRW(twSceneSettings, "ResetCamera", TW_TYPE_BOOLCPP, &g_userInput.resetCamera, "");


	TwEnumVal lodTypeEV[] = { { LODMode::NONE, "None" },{ LODMode::OCTREE_NAIVE, "Octree naive" },{ LODMode::NESTED_OCTREE_NAIVE, "Nested Octree naive" },{ LODMode::K_MEANS, "k-means" },{ LODMode::EGGS, "Eggs" } };
	TwType twLODMode = TwDefineEnum("LOD Mode", lodTypeEV, ARRAYSIZE(lodTypeEV));
	TwAddVarRW(twLODSettings, "LOD Mode", twLODMode, &g_lodSettings.mode, NULL);
	TwAddVarRW(twLODSettings, "Pixel Threshold", TW_TYPE_INT32, &g_lodSettings.pixelThreshhold, "min=1 max=50 step=1");
	TwAddSeparator(twMenuBar, "sep3", "");
	TwAddVarRW(twLODSettings, "recreate LOD", TW_TYPE_BOOLCPP, &g_lodSettings.recreate, NULL);



	TwAddVarRW(twMenuBar, "Scene Menu", TW_TYPE_BOOLCPP, &g_userInput.showSceneMenu, "");
	TwAddVarRW(twMenuBar, "Render Menu", TW_TYPE_BOOLCPP, &g_userInput.showRenderMenu, "");
	TwAddVarRW(twMenuBar, "LOD Menu", TW_TYPE_BOOLCPP, &g_userInput.showLODMenu, "");
	TwAddSeparator(twMenuBar, "sep4", ""); 
	TwAddVarRO(twMenuBar, "Frames per Sec", TW_TYPE_FLOAT, &g_statistics.framesPerSec, "");
	TwAddVarRO(twMenuBar, "Vertices Drawn", TW_TYPE_INT32, &g_statistics.verticesDrawn, "");





	DragAcceptFiles(g_hWindow, TRUE);

	g_renderer->reloadShaders(); 

	/*
	* ----------- Init scene ----------
	*/
	
	camera = new Camera(&g_screenParams.width, &g_screenParams.height, g_screenParams.nearPlane, g_screenParams.farPlane); 
	camera->translateAbs(0.1f, 1.0f, -75.0f); 


	XMStoreFloat3(&g_userInput.lightDirection, XMVector3Normalize(XMVectorSet(-1, -1, 0.5, 0)));  //rnd ligth dir, somewhere from above

	/*
	--------------- TEST -----------
	*/
	//GameObject* dragon = new GameObject("dragon_perlin_color.ply");

	//g_sceneRoot.addChild(dragon); 

	//-------------------


	g_sceneRoot.initialize(g_renderer);


	auto start = std::chrono::high_resolution_clock::now();

	//render loop
	while (run)
	{
		std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
		start = std::chrono::high_resolution_clock::now();
		g_deltaTime = elapsed.count(); 

		update(); 



		g_renderer->newFrame(camera->getViewMatrix(), camera->getProjectionMatrix(), &camera->pos, &camera->forward );

		input();

		XMFLOAT4X4 identity;
		XMStoreFloat4x4(&identity, XMMatrixIdentity());


		g_sceneRoot._render(g_renderer, &identity);



		TwSetParam(twSceneSettings, NULL, "visible", TW_PARAM_CSTRING, 1, g_userInput.showSceneMenu ? "true" : "false"); 
		TwSetParam(twRenderSettings, NULL, "visible", TW_PARAM_CSTRING, 1, g_userInput.showRenderMenu ? "true" : "false");
		TwSetParam(twLODSettings, NULL, "visible", TW_PARAM_CSTRING, 1, g_userInput.showLODMenu ? "true" : "false");



		TwDraw(); 
	
		g_renderer->endFrame();

	}

	TwTerminate(); 
	delete g_renderer; 
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
		g_screenParams.width, g_screenParams.height,
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
	g_renderer->setSplatSize(g_renderSettings.splatSize); 

	g_renderer->setLight(XMLoadFloat3(&g_userInput.lightDirection), XMLoadFloat3(&g_userInput.lightColor));


	if (g_lodSettings.mode != g_lodSettings.lastMode)
	{
		if (g_lodSettings.twImplSettingsBar)
		{
			TwDeleteBar(g_lodSettings.twImplSettingsBar);
		}

		switch (g_lodSettings.mode)
		{
		case OCTREE_NAIVE:
		{
			g_lodSettings.twImplSettingsBar = Octree_Naive_Avg::setUpTweakBar();
			break;
		}
		case NESTED_OCTREE_NAIVE:
		{
			g_lodSettings.twImplSettingsBar = Nested_Octree_Naive_Avg::setUpTweakBar();
			break; 
		}
		default:
			break;
		}

		g_lodSettings.lastMode = g_lodSettings.mode;
	}

	if (g_activeObject)
	{
		g_activeObject->rot = g_userInput.objectRotation; 

		if (g_lodSettings.recreate)	//ok just assuming that the active obj has a mesh, should work for this impl might cause problems for someone else tho... 
		{
			PointCloud* mesh = g_renderer->meshDict.find(g_activeObject->getMesh())->second;
			
			if (mesh->lod)
			{
				delete mesh->lod; 
			}

			mesh->createLod(g_renderer->d3dDevice, g_lodSettings.mode); 
		}
	}
	g_lodSettings.recreate = false;	//make sure this is reset... buttons are hard 

	if (g_userInput.resetCamera)
	{
		g_userInput.resetCamera = false; 
		delete camera; 
		camera = new Camera(&g_screenParams.width, &g_screenParams.height, g_screenParams.nearPlane, g_screenParams.farPlane);
		camera->translateAbs(0.1f, 1.0f, -75.0f);
	}

	if (g_userInput.reloadShaders)
	{
		g_userInput.reloadShaders = false; 
		g_renderer->reloadShaders(); 
	}

	//camera mode: wasd or orbit
	camera->setTarget(g_userInput.orbitCam?g_activeObject:nullptr);




	g_statistics.framesPerSec = 1 / g_deltaTime; 


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
				camera->rotate(( g_userInput.lastMouseY- mouseY) * g_userInput.camRotateSpeed * g_deltaTime, (g_userInput.lastMouseX - mouseX) * g_userInput.camRotateSpeed* g_deltaTime, 0);
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
				g_activeObject->initialize(g_renderer);
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
		camera->move(0,0, g_userInput.cameraSpeed * g_deltaTime); 
		break; 
	}

	case 0x53: //s
	{
		camera->move(0, 0, -g_userInput.cameraSpeed * g_deltaTime);
		break;
	}

	case 0x41: //a
	{
		camera->move(-g_userInput.cameraSpeed * g_deltaTime, 0, 0);

		break;
	}

	case 0x44: //d
	{
		camera->move(g_userInput.cameraSpeed * g_deltaTime, 0, 0);
		break;
	}

	}
}
