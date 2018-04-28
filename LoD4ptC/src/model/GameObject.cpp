#include "GameObject.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace DirectX;

GameObject::GameObject() :
sca(XMFLOAT4(1.0f, 1.0f, 1.0f, 0.0f)), rot(XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f)), pos(XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f)), transparent(false)
{
	XMStoreFloat4x4(&m_world, XMMatrixIdentity());
}

GameObject::GameObject(std::string _mesh) : meshName(_mesh),
sca(XMFLOAT4(1.0f, 1.0f, 1.0f, 0.0f)), rot(XMFLOAT4(0.0f,0.0f, 0.0f, 1.0f)), pos(XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f)), transparent(false)
{
	XMStoreFloat4x4(&m_world, XMMatrixIdentity());
}


GameObject::~GameObject()
{
}

void GameObject::_update(double dTime)
{

	this->update(dTime);

	for (GameObject* child : children){
		child->_update(dTime);
	}

}

void GameObject::initialize(Renderer* renderer)
{
	renderer->loadMesh(meshName);


	for (GameObject* child : children) {
		child->initialize(renderer);
	}


}



void GameObject::_render( Renderer* renderer, XMFLOAT4X4* const m_parrent)
{
	

	XMMATRIX t_m_world;
	if (m_parrent)
	{
		t_m_world = XMMatrixScalingFromVector(XMLoadFloat4(&sca))*XMMatrixRotationQuaternion(XMLoadFloat4(&rot))*XMMatrixTranslationFromVector(XMLoadFloat4(&pos))*XMLoadFloat4x4(m_parrent);
	}
	else
	{
		t_m_world = XMMatrixScalingFromVector(XMLoadFloat4(&sca))*XMMatrixRotationQuaternion(XMLoadFloat4(&rot))*XMMatrixTranslationFromVector(XMLoadFloat4(&pos));
	}

	XMStoreFloat4x4(&m_world, t_m_world);
	if (visible)
		renderer->render(meshName, &m_world);

	for (GameObject* child : children){
		child->_render(renderer, &m_world);
	}
}

XMFLOAT4X4* const GameObject::getWorldMatrix()
{
	return &m_world;
}

void GameObject::scale(float x, float y, float z)
{
	XMStoreFloat4(&sca, XMVectorAdd(XMLoadFloat4(&sca), XMVectorSet(x, y, z, 0.0f)));


}

void GameObject::rotate(float x, float y, float z)
{
	pitch += x; 
	if (pitch >= 2 * M_PI)
	{
		pitch -= 2 * M_PI;
	}

	yaw += y; 
	if (yaw >= 2 * M_PI)
	{
		yaw -= 2 * M_PI;
	}

//	std::cout << "Rotate:\t" << x << "  " << y << "  " << z << std::endl; 
//	std::cout << "before:\t" << rot.x << "  " << rot.y << "  " << rot.z << "  " << rot.w << std::endl; 
	XMStoreFloat4(&rot, XMQuaternionRotationRollPitchYaw(pitch, yaw, 0)); 
//	std::cout << "after:\t" << rot.x << "  " << rot.y << "  " << rot.z << "  " << rot.w << std::endl;

}

void GameObject::translate(float x, float y, float z)
{
	XMStoreFloat4(&pos, XMVectorAdd(XMLoadFloat4(&pos), XMVectorSet(x, y, z, 0.0f)));
}

void GameObject::scaleAbs(float x, float y, float z)
{
	sca = XMFLOAT4(x, y, z, 0.0f);

}

void GameObject::rotateAbs(float x, float y, float z)
{
	XMStoreFloat4(&rot, XMQuaternionRotationRollPitchYaw(x, y, z));

}

void GameObject::translateAbs(float x, float y, float z)
{
	pos = XMFLOAT4(x, y, z, 0.0f);
}


void GameObject::move(float x, float y, float z)
{
	XMStoreFloat4(&pos, XMVectorAdd(XMLoadFloat4(&pos), XMVector3Rotate(XMVectorSet(x, y, z, 0), XMLoadFloat4(&rot)))); 
}
