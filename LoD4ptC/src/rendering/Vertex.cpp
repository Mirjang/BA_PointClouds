#include "Vertex.h"

using namespace DirectX; 


SphereVertex::SphereVertex(const Vertex& baseVert) : SphereVertex(baseVert.pos, baseVert.normal, baseVert.color)
{}

SphereVertex::SphereVertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color)
	: Vertex(pos, normal, color)
{
	XMVECTOR vmajor, vminor;
	XMVECTOR vnormal = XMLoadFloat3(&normal);
	
	radius = 0.0f; 
}

EllipticalVertex::EllipticalVertex(const Vertex& baseVert) : EllipticalVertex(baseVert.pos, baseVert.normal, baseVert.color)
{}

EllipticalVertex::EllipticalVertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color)
	: Vertex(pos, normal, color)
{
	XMVECTOR vmajor, vminor;
	XMVECTOR vnormal = XMLoadFloat3(&normal);
	
	vmajor = XMVectorScale(XMVector3Normalize(XMVector3Orthogonal(vnormal)), 0.0001f); // leaf verts should prob. be treated seperatly, but for now setting their radius to 0 moght suffice
	vminor = XMVectorScale(XMVector3Normalize(XMVector3Cross(vmajor, vnormal)), 0.0001f);
	

	/**
	vmajor = XMVector3Normalize(XMVectorSet(1,1,1,1));
	vminor = vmajor; 
	/**/

	XMStoreFloat3(&this->major, vmajor);
	XMStoreFloat3(&this->minor, vminor);
}

