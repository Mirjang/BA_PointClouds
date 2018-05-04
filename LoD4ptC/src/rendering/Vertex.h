#pragma once


#include <DirectXMath.h>

struct Vertex
{
public:
	Vertex() :_pos( 0,0,0 ), _normal(1,0,0 ), _color( 0,1,0,1 )
	{
		
	}

	Vertex(const DirectX::XMFLOAT3& pos, const DirectX::XMFLOAT3& normal, const DirectX::XMFLOAT4& color)
		:_pos(pos), _normal(normal), _color(color)
	{

	}

	Vertex(const float& x, const float&y, const float&z, const float&nx, const float&ny, const float&nz, const float&r, const float&g, const float&b, const float&a)
		:_pos(x, y, z), _normal(nx, ny, nz), _color(r, g, b, a)
	{

	}

	~Vertex()
	{

	}

	DirectX::XMFLOAT3 _pos;
	DirectX::XMFLOAT3 _normal; 
	DirectX::XMFLOAT4 _color; 
};

