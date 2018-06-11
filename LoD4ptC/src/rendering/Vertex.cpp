#include "Vertex.h"

using namespace DirectX; 

EllipticalVertex::EllipticalVertex(const Vertex& baseVert) : Vertex(baseVert)
{
	XMVECTOR vmajor, vminor;
	XMVECTOR vnormal = XMLoadFloat3(&baseVert.normal);
	vmajor = XMVector3Normalize(XMVector3Orthogonal(vnormal));
	//overnormalize just in case 
	vminor = XMVector3Normalize(XMVector3Cross(vmajor, vnormal));

	XMStoreFloat3(&this->major, vmajor);
	XMStoreFloat3(&this->minor, vminor);
}