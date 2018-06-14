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

struct SphereVertex : public Vertex
{
	SphereVertex() :Vertex() {};
	SphereVertex(const Vertex& baseVert);

	SphereVertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color);
	
	float radius; 
};

struct EllipticalVertex :public Vertex
{
	EllipticalVertex() :Vertex() {};
	EllipticalVertex(const Vertex& baseVert); 

	EllipticalVertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color);


	DirectX::XMFLOAT3 major; 
	DirectX::XMFLOAT3 minor; 
};

struct EllipsoidVertex : public Vertex
{
	float x, y, z; 
};