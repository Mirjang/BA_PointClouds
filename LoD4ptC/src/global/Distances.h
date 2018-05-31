#pragma once

#include <DirectXMath.h>


namespace Distances
{

	//checks wether there is a minimum distance between p1 and p2
	extern bool distanceCheckEuclidian(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2, const DirectX::XMVECTOR& minDist);

	extern bool distanceCheckManhattan(const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2, const DirectX::XMVECTOR& minDist);


}