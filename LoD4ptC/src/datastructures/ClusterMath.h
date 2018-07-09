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
struct Cluster;

template<>
struct Cluster<SphereVertex>
{
	std::vector<SphereVertex> verts;
	Vec9f centroid;
	float maxDistSq, maxNorAngle, maxColDistSq; 
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

		float dist = (pv-pc).squaredNorm(); 
	
		if (dist > maxDistSq)
		{
			return false; 
		}
		else
		{
			Eigen::Vector3f cc = centroid.tail<3>();
			Eigen::Vector3f cv = fv.tail<3>();

			if ((cv - cc).squaredNorm() > maxColDistSq)
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
	float maxDistSq, maxNorAngle, maxColDistSq;

	Eigen::Vector3f major; 


	inline void initCentroid(const EllipticalVertex& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq)
	{ 
		this->maxDistSq = maxDistSq;
		this->centroid = vertex2Feature(centroid);
		this->maxNorAngle = maxNorAngle;
		this->maxColDistSq = maxColDistSq;
	}

	bool checkAdd(const EllipticalVertex& vert)
	{

		Vec9f fv = vertex2Feature(vert);

		Eigen::Vector3f pc = centroid.head<3>();
		Eigen::Vector3f pv = fv.head<3>();

		float dist = (pv - pc).squaredNorm();

		if (dist > maxDistSq)
		{
			return false;
		}
		else
		{
			Eigen::Vector3f cc = centroid.tail<3>();
			Eigen::Vector3f cv = fv.tail<3>();

			if ((cv - cc).squaredNorm() > maxColDistSq)
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

	EllipticalVertex getSplat()
	{

		if (verts.size() == 1)
		{
			return verts[0]; 
		}

		EllipticalVertex newVert = feature2Vertex(centroid);

		if (false && verts.size() == 2) // 2 pts wont be enough for pca
		{
			XMVECTOR vmajor, vminor;
			XMVECTOR vnormal = XMLoadFloat3(&newVert.normal);

			vmajor = XMVectorScale(XMVector3Normalize(XMVector3Orthogonal(vnormal)), maxDistSq); // leaf verts should prob. be treated seperatly, but for now setting their radius to 0 moght suffice
			vminor = XMVectorScale(XMVector3Normalize(XMVector3Cross(vmajor, vnormal)), maxDistSq);

			XMStoreFloat3(&newVert.major, vmajor);
			XMStoreFloat3(&newVert.minor, vminor);
		}
		else
		{
			Eigen::MatrixX3f clusterMat;
			clusterMat.resize(verts.size(), 3);
			for (int i = 0; i < verts.size(); ++i)
			{
				clusterMat.row(i) = xmfloat2Eigen(verts[i].pos) - centroid.head<3>();
			}

			Eigen::Matrix3f spacialCovariance = clusterMat.transpose()* clusterMat * (1/(verts.size()-1));
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca;
			pca.compute(spacialCovariance);

			Eigen::Vector3f major = pca.eigenvectors().col(2).normalized();
			Eigen::Vector3f minor = pca.eigenvectors().col(1).normalized();

			minor = major.cross(centroid.segment<3>(3)).normalized(); 

			float majorLen = clusterMat.cwiseProduct(major.transpose()).rowwise().sum().maxCoeff(); //rowwise dotProduct
			float minorLen = clusterMat.cwiseProduct(minor.transpose()).rowwise().sum().maxCoeff();

			major *= majorLen; 
			minor *= minorLen; 

			newVert.major.x = major.x();
			newVert.major.y = major.y();
			newVert.major.z = major.z();

			newVert.minor.x = minor.x();
			newVert.minor.y = minor.y();
			newVert.minor.z = minor.z();	
		}
		/**/

		return newVert;
	}


};


