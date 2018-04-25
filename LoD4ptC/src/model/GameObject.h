#pragma once

#include <DirectXMath.h>
#include <list>

#include "../render/Renderer.h"

using namespace DirectX;
//Basic class for any object to be represented in the game
class GameObject
{
public:
	GameObject();
	GameObject(std::string mesh);
	virtual ~GameObject();

	virtual void scale(float x, float y, float z);
	virtual void rotate(float x, float y, float z);
	virtual void translate(float x, float y, float z);

	virtual void scaleAbs(float x, float y, float z);
	virtual void rotateAbs(float x, float y, float z);
	virtual void translateAbs(float x, float y, float z);

	virtual void move(float x, float y, float z); 


	virtual void scaleAbs(XMFLOAT4* _sca){ XMStoreFloat4(&sca, XMLoadFloat4(_sca)); }
	virtual void translateAbs(XMFLOAT4* _pos) { XMStoreFloat4(&pos, XMLoadFloat4(_pos)); }

	void initialize(Renderer* renderer); 

	const std::string& getMesh(){ return meshName; }
	void setTransparent(bool b){ transparent = b; }
	bool isTransparent(){ return transparent; }

	void addChild(GameObject* child){
		children.push_back(child);
	}



	virtual void _update(double dTime) final;

	XMFLOAT4X4* const getWorldMatrix();
	virtual void _render(Renderer* renderer, XMFLOAT4X4* const m_Parrent) final;


	XMFLOAT4 pos, sca, rot;

	float pitch = 0, yaw = 0; 
	bool visible = true;

	std::list<GameObject*> children;


protected:

	virtual void update(double dTime){};


	std::string meshName = ""; 
	bool transparent; 


	XMFLOAT4X4 m_world;

};

