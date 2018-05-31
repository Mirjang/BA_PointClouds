#include "Distances.h"

using namespace DirectX; 


namespace Distances {

	//checks wether point(pos) fulfilles the minimum distance requirement (min distance = cellsize[depth]. InsertMap: all points allready inserted
	inline bool distanceCheckEuclidian(const XMVECTOR& p1, const XMVECTOR& p2, const XMVECTOR& minDist)
	{
		return XMVector3LengthSq(p1 - p2).m128_f32[0] > minDist.m128_f32[0]*minDist.m128_f32[0];
	}

	inline bool distanceCheckManhattan(const XMVECTOR& p1, const XMVECTOR& p2, const XMVECTOR& minDist)
	{
		return XMComparisonAnyFalse(XMVector3GreaterOrEqualR(XMVectorAbs(p1 - p2), minDist));
	}

}