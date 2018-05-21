#include "Camera.h"

#include "../../global/utils.h"

using namespace DirectX;

Camera::Camera(int* _pScreen_width, int* _pScreen_heigth, float _near, float _far) :
pScreen_width(_pScreen_width), pScreen_heigth(_pScreen_heigth), nearDist(_near), farDist(_far)
{
	visible = false; 
	target = new GameObject();
	XMStoreFloat4(&up, XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
	GameObject();
	XMStoreFloat4x4(&m_world, XMMatrixIdentity());

}

Camera::~Camera()
{
}


XMFLOAT4X4* const Camera::getViewMatrix()
{
	if (target)
	{
		XMStoreFloat4x4(&m_View, XMMatrixLookAtLH(XMLoadFloat4(&pos), XMLoadFloat4(&target->pos), XMVectorSet(0, 1, 0, 0)));
	}
	else
	{
		XMVECTOR quat = XMLoadFloat4(&rot);
		XMVECTOR vforward = XMVector3Rotate(XMVectorSet(0, 0, 1,0), quat);

		XMStoreFloat4(&forward, vforward); 
		XMStoreFloat4x4(&m_View, XMMatrixLookAtLH(XMLoadFloat4(&pos), XMLoadFloat4(&pos) + vforward, XMVectorSet(0,1,0,0)));
	}
	return &m_View;	
}

XMFLOAT4X4* const Camera::getProjectionMatrix()
{
	XMStoreFloat4x4(&m_Proj, XMMatrixPerspectiveFovLH(g_screenParams.fov, g_screenParams.width/g_screenParams.height, g_screenParams.nearPlane, g_screenParams.farPlane));
	return &m_Proj;
}