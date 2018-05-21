#pragma once

#include <DirectXMath.h>

#include "../GameObject.h"


using namespace DirectX;

class Camera :
	public GameObject
{
public:

	Camera(int* _pScreen_width, int* _pScreen_heigth, float _near, float _far);
	virtual ~Camera();

	void setTarget(GameObject* _target){ target = _target; }
	XMFLOAT4X4* const getViewMatrix();
	XMFLOAT4X4* const getProjectionMatrix();

	XMFLOAT4 forward;


private:
	int* pScreen_width;
	int* pScreen_heigth;
	float nearDist;
	float farDist;

	GameObject* target = nullptr;

	XMFLOAT4 up;


	XMFLOAT4X4 m_View, m_Proj;

};

