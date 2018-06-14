#pragma once

#include <Eigen/dense>
#include <DirectXMath.h>


typedef Eigen::Matrix<float, 9, 1> Vec9f;

struct Centroid
{
	Centroid()
	{

	}

	Centroid(float x, float y, float z) : Centroid()
	{
		features << x, y, z;
	}

	Centroid(const DirectX::XMVECTOR& pos) : Centroid()
	{
		features << pos.m128_f32[0], pos.m128_f32[1], pos.m128_f32[0];
	}

	Vec9f features;
};




class Kmeans
{
public:
	Kmeans();
	~Kmeans();
};

