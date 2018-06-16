#pragma once

#include <Eigen/dense>
#include <DirectXMath.h>
#include <vector>
#include <basetsd.h>
#include <iostream>

typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 9> MatX9f;


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

	Kmeans(MatX9f& data, const Vec9f& scaleParams, bool normalizeData = true)
	{
		constants.scaleParams = scaleParams;


		for (int i = 0; i < 9; ++i)
		{
			constants.lowerBound(i) = data.col(i).minCoeff();
			constants.upperBound(i) = data.col(i).maxCoeff();
		}

		constants.invNormalisationConsts = constants.upperBound - constants.lowerBound;
		constants.normalisationConsts = constants.invNormalisationConsts.cwiseInverse().cwiseProduct(scaleParams);
		constants.normalisationConsts = constants.normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
		constants.normalisationConsts = constants.normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
//	std::cout << data << std::endl; 

		if (normalizeData)
		{

			for (size_t i = 0; i < data.rows(); ++i)
			{
				data.row(i) = data.row(i).cwiseProduct(constants.normalisationConsts.transpose()).eval(); 
			}

		}
	}


	void runKMEANS(const MatX9f& data, UINT32 numCentroids, UINT32 iterations);


	struct CreateConstants
	{
		Vec9f lowerBound;
		Vec9f upperBound; 
		Vec9f normalisationConsts; 
		Vec9f invNormalisationConsts;
		Vec9f scaleParams; 
	};
	CreateConstants constants; 
	std::vector<UINT32> vertCentroidTable;
	std::vector<Centroid> centroids; 



private:

	inline void initCentroids(const UINT32& numCentroids);

	inline void updateCentroids(const MatX9f& verts);

	//Distance: xyz are divided by max(x,y,z) of the bounding box, normals and colors are already in a normalized space
	inline void updateObservations(const MatX9f& verts);

};
