#pragma once

#include <vector>
#include <DirectXMath.h> 
#include <Eigen/dense>
#include <iostream>
#include <minmax.h>
#include "../rendering/Vertex.h"
#include "../global/Distances.h"

using namespace DirectX; 
typedef Eigen::Matrix<float, 9, 1> Vec9f;
typedef Eigen::Matrix<float, Eigen::Dynamic, 9> MatX9f;

enum CenteringMode
{
	KEEP_SEED,
	AMEAN,
	SPACIAL,
	SPACIAL_POS_REST_AMEAN,
};

inline Vec9f vertex2Feature(const Vertex& vert)
{
	Vec9f fv;
	fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
		vert.color.x, vert.color.y, vert.color.z;

	return fv; 
}

inline Vertex feature2Vertex(const Vec9f& vert)
{
	Vertex newVert;
	newVert.pos.x = vert(0);
	newVert.pos.y = vert(1);
	newVert.pos.z = vert(2);
	newVert.normal.x = vert(3);
	newVert.normal.y = vert(4);
	newVert.normal.z = vert(5);
	newVert.color.x = vert(6);
	newVert.color.y = vert(7);
	newVert.color.z = vert(8);
	newVert.color.w = 1.0f; //ignoring alpha

	return newVert; 
}

inline XMVECTOR eigen2XMVector(const Eigen::Vector3f& vec)
{
	return XMVectorSet(vec.x(), vec.y(), vec.z(), 0); 
}

inline Eigen::Vector3f xmfloat2Eigen(const XMFLOAT3& vec)
{
	Eigen::Vector3f v; 
	v << vec.x, vec.y, vec.z;
	return v; 
}

template<class VertType>
struct Cluster
{
	std::vector<VertType> verts;
	Vec9f centroid;
	float maxDistSq, maxNorAngle, maxColDistSq; 
	float radius = 0.0f; 
	CenteringMode centerMode; 
	inline void initCentroid(const VertType& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode = CenteringMode::KEEP_SEED)
	{
		verts.push_back(centroid); 
		this->maxDistSq = maxDistSq; 
		this->centroid = vertex2Feature(centroid); 
		this->maxNorAngle = maxNorAngle; 
		this->maxColDistSq = maxColDistSq; 
		this->centerMode = centerMode;
	}

	bool checkAdd(const VertType& vert);

	void center()
	{
		switch (centerMode)
		{
		case AMEAN:
		{
			centroid = Vec9f::Zero();
			for (auto vert : verts)
			{
				centroid += vertex2Feature(vert);
			}

			centroid /= static_cast<float>(verts.size());
			break;
		}
		case SPACIAL:
		{
			Vec9f min = vertex2Feature(verts[0]);
			Vec9f max = min;
			for (auto vert : verts)
			{
				Vec9f current = vertex2Feature(vert);
				min = min.cwiseMin(current);
				max = max.cwiseMax(current);
			}

			centroid = (min + max) / 2.0f;
			break;
		}
		case SPACIAL_POS_REST_AMEAN:
		{
			Eigen::Vector3f min = xmfloat2Eigen(verts[0].pos);
			Eigen::Vector3f max = min;

			Eigen::Matrix<float, 6, 1> norcol = Eigen::Matrix<float, 6, 1>::Zero();

			for (auto vert : verts)
			{
				Vec9f current = vertex2Feature(vert);
				min = min.cwiseMin(current.head<3>());
				max = max.cwiseMax(current.head<3>());

				norcol += current.tail<6>();
			}
			centroid.head<3>() = (min + max) / 2;
			centroid.tail<6>() = norcol / verts.size();
			break;
		}
		default:
			break;
		}

		centroid.segment<3>(3).normalize();

	}

	VertType getSplat(); 

};


template<class VertType>
struct Cluster2 //requires verts to be ordered by dist
{

	std::vector<std::pair<float, VertType>> verts;
	Vec9f centroid;
	float maxDistSq, maxNorAngle, maxColDistSq;
	Eigen::Vector3f major, minor; 
	float majorLengthSq, minorLengthSq; 
	CenteringMode centerMode;

	inline void initCentroid(const VertType &centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode = CenteringMode::KEEP_SEED)
	{
		verts.clear();
		verts.push_back(std::pair<float, VertType>(0,centroid));
		minorLengthSq = 0.0f; 
		this->maxDistSq = maxDistSq;
		this->centroid = vertex2Feature(centroid);
		this->maxNorAngle = maxNorAngle;
		this->maxColDistSq = maxColDistSq;
		this->centerMode = centerMode; 
	}

	inline void center()
	{
		switch (centerMode)
		{
		case AMEAN:
		{
			Eigen::Matrix<float, 6, 1> norcol = Eigen::Matrix<float, 6, 1>::Zero();
			for (auto vert : verts)
			{
				Vec9f current = vertex2Feature(vert.second);
				norcol += current.tail<6>();
			}
			centroid.tail<6>() = norcol / verts.size();
			break;
		}
		case SPACIAL:
		{

			Eigen::Matrix<float, 6, 1> norcolMin = vertex2Feature(verts[0].second).tail<6>();
			Eigen::Matrix<float, 6, 1> norcolMax = norcolMin; 

			for (auto vert : verts)
			{
				Vec9f current = vertex2Feature(vert.second);

				norcolMin = current.tail<6>().cwiseMin(norcolMin);
				norcolMax = current.tail<6>().cwiseMax(norcolMax);

			}
			centroid.tail<6>() = (norcolMin + norcolMax) / 2; 
			break;
		}
		default:
			break;
		}

		centroid.segment<3>(3).normalize();
	}

	bool checkAdd(const std::pair<float, VertType>& candidate); 

	VertType getSplat(); 

};
