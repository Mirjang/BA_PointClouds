#pragma once


#include <DirectXMath.h>

struct Vertex
{
public:
	Vertex() :pos( 0,0,0 ), normal(1,0,0 ), color( 0,1,0,1 )
	{
		
	}

	Vertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color)
		:pos(pos), normal(normal), color(color)
	{

	}

	Vertex(const float& x, const float&y, const float&z, const float&nx, const float&ny, const float&nz, const float&r, const float&g, const float&b, const float&a)
		:pos(x, y, z), normal(nx, ny, nz), color(r, g, b, a)
	{

	}
	
	~Vertex()
	{

	}

	DirectX::XMFLOAT3 pos;
	DirectX::XMFLOAT3 normal; 
	DirectX::XMFLOAT4 color; 
};

struct SphereVertex : Vertex
{
	float radius; 
};

struct EllipticalVertex : Vertex
{
	DirectX::XMFLOAT3 axis1, axis2; 
};

struct EllipsoidVertex : Vertex
{
	float x, y, z; 
};