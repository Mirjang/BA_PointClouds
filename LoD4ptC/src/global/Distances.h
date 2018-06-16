#pragma once

#include <DirectXMath.h>


namespace Distances
{

	//checks wether there is a minimum distance between p1 and p2
	extern bool distanceCheckEuclidian(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2, const DirectX::XMVECTOR& minDist);

	extern bool distanceCheckManhattan(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2, const DirectX::XMVECTOR& minDist);

	extern float distanceEuclidian(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2);
	extern float distanceManhattan(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2);

	//duplicate in LodUtils.h
	inline DirectX::XMVECTOR cartToPolarNormal(const DirectX::XMVECTOR& normal)
	{
		float zenith = acosf(normal.m128_f32[1]);
		float azimuth = atan2f(normal.m128_f32[1], normal.m128_f32[0]);
		return DirectX::XMVectorSet(zenith, azimuth, 0, 0);
	}


	inline DirectX::XMVECTOR polarToCartNormal(const DirectX::XMVECTOR& normal)
	{
		float x = sinf(normal.m128_f32[0]) * cosf(normal.m128_f32[1]);
		float y = sinf(normal.m128_f32[0]) * sinf(normal.m128_f32[1]);
		float z = cosf(normal.m128_f32[0]);
		return DirectX::XMVectorSet(x, y, z, 0);
	}

}