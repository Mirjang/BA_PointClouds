#pragma once

#include <Eigen/dense>
#include <DirectXMath.h>
#include <vector>
#include <basetsd.h>

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
	struct CreateConstants
	{
		float maxSpacialRange;
	};
	CreateConstants constants; 

	void initCentroids(const Vec9f& vmin, const Vec9f& vmax, const UINT32& numCentroids, std::vector<Centroid>& centroids);

	void updateCentroids(std::vector<Centroid>& centroids, const std::vector<Vec9f>& verts, const std::vector<UINT32>& vertCentroidTable);

	//Distance: xyz are divided by max(x,y,z) of the bounding box, normals and colors are already in a normalized space
	void updateObservations(const std::vector<Centroid>& centroids, const std::vector<Vec9f>&verts, std::vector<UINT32>& vertCentroidTable);

};
