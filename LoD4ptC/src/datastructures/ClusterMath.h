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


template<class VertType> 
struct Cluster;

template<>
struct Cluster<SphereVertex>
{
	std::vector<SphereVertex> verts;
	Vec9f centroid;
	float maxDistSq, maxNorAngle, maxColDistSq; 

	bool debug = false; 

	float radius = 0.0f; 

	inline void initCentroid(const SphereVertex& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq)
	{
		radius = centroid.radius; 
		this->maxDistSq = maxDistSq; 
		this->centroid = vertex2Feature(centroid); 
		this->maxNorAngle = maxNorAngle; 
		this->maxColDistSq = maxColDistSq; 
	}

	bool checkAdd(const SphereVertex& vert)
	{

		Vec9f fv = vertex2Feature(vert); 

		Eigen::Vector3f pc = centroid.head<3>(); 
		Eigen::Vector3f pv = fv.head<3>();	

		float dist = (pc-pv).squaredNorm(); 
	
		if (dist > maxDistSq)
		{
			return false; 
		}
		else
		{
			Eigen::Vector3f cc = centroid.tail<3>();
			Eigen::Vector3f cv = fv.tail<3>();

			if ((cc - cv).squaredNorm() > maxColDistSq)
			{
				maxDistSq = min(maxDistSq, dist);
				return false;
			}
			else
			{
				Eigen::Vector3f nc = centroid.segment<3>(3);
				Eigen::Vector3f nv = fv.segment<3>(3);
				if (abs(acosf(nc.dot(nv))) > maxNorAngle) // normal 
				{
					//		if (debug)
					//		{
					//			std::cout << "nor fail "<<"dot: " << nc.dot(nv) <<" acos: " << acosf(nc.dot(nv)) <<"\n" << abs(acosf(nc.dot(nv))) << " > " << maxNorAngle << std::endl;
					//		}
					maxDistSq = min(maxDistSq, dist);
					return false;
				}
				else
				{
					//vert in range -> add
					verts.push_back(vert);
					return true;
				}
			}
		}
	}

	void center()
	{
		centroid = Vec9f::Zero();
		for (auto vert : verts)
		{
			centroid += vertex2Feature(vert);
		}

		centroid /= static_cast<float>(verts.size());

		centroid.segment<3>(3).normalize();

	}

	SphereVertex getSplat()
	{
		SphereVertex newVert = feature2Vertex(centroid); 
	//	newVert.radius = verts.size() == 1 ? radius : sqrtf(maxDistSq); 
		newVert.radius = radius;

		
		Eigen::Vector3f pc = centroid.head<3>();
		for (auto vert : verts)
		{
			Eigen::Vector3f pv;
			pv << vert.pos.x, vert.pos.y, vert.pos.z;
			float dist = (pv - pc).norm() + vert.radius;
			newVert.radius = max(newVert.radius, dist);
		}

		return newVert; 
	}
};

template<>
struct Cluster<EllipticalVertex>
{
	std::vector<EllipticalVertex> verts;
	Vec9f centroid;
	Eigen::Vector3f major; 
	Eigen::Vector3f minor; 
	float firstFailedDistSQ = FLT_MAX;
	float maxNorAngle, maxColDistSq;

	inline void initCentroid(const EllipticalVertex& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq)
	{
		firstFailedDistSQ = maxDistSq;
		this->centroid = vertex2Feature(centroid);
		this->maxNorAngle = maxNorAngle;
		this->maxColDistSq = maxColDistSq;
	}

	inline bool checkAdd(const EllipticalVertex& vert)
	{

		Vec9f fv = vertex2Feature(vert);

		Eigen::Vector3f pc = centroid.head<3>();
		Eigen::Vector3f pv = fv.head<3>();

		float dist = (pc - pv).squaredNorm();

		if (dist > firstFailedDistSQ) // there is already a closer vert not in range
		{
			return false;
		}
		else
		{
			Eigen::Vector3f nc = centroid.segment<3>(3);
			Eigen::Vector3f nv = fv.segment<3>(3);
			if (abs(acosf(nc.dot(nv)))>maxNorAngle) // normal 
			{
				if (dist < firstFailedDistSQ)
				{
					firstFailedDistSQ = dist;
				}
				return false;
			}
			else
			{
				Eigen::Vector3f cc = centroid.tail<3>();
				Eigen::Vector3f cv = fv.tail<3>();

				if ((cc - cv).squaredNorm() > maxColDistSq)
				{
					if (dist < firstFailedDistSQ)
					{
						firstFailedDistSQ = dist;
					}
					return false;
				}

			}
		}
		//vert in range -> add
		verts.push_back(vert);
		return true;
	}

	std::vector<EllipticalVertex> center()
	{
		/**
		std::vector<EllipticalVertex> removed;
		Vec9f newCentroid = Vec9f::Zero();
		for (auto it = verts.begin(); it != verts.end(); )
		{
			const EllipticalVertex& vert = *it;
			Vec9f fv = vertex2Feature(vert);

			Eigen::Vector3f pc = centroid.head<3>();
			Eigen::Vector3f pv = fv.head<3>();


			float dist = (pc - pv).squaredNorm();

			if (dist > firstFailedDistSQ) // there is already a closer vert not in range
			{
				it = verts.erase(it);
				removed.push_back(vert);
				continue;
			}

			if (dist > radiusSq)
			{
				radiusSq = sqrt(dist) + vert.radius;
				radiusSq *= radiusSq;
			}

			newCentroid += fv;
			++it;
		}

		newCentroid /= static_cast<float>(verts.size());
		centroid = newCentroid;
		return removed;
		/**/
	}

	EllipticalVertex getSplat()
	{
		EllipticalVertex newVert = feature2Vertex(centroid);



		if (verts.size() == 1)
		{
			newVert.color.x = 1.0f;
			newVert.color.y = 0.0f;
		}
		return newVert;
	}


};


