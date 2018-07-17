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
struct Cluster;

template<>
struct Cluster<SphereVertex>
{
	std::vector<SphereVertex> verts;
	Vec9f centroid;
	float maxDistSq, maxNorAngle, maxColDistSq; 
	float radius = 0.0f; 
	CenteringMode centerMode; 
	inline void initCentroid(const SphereVertex& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode = CenteringMode::KEEP_SEED)
	{
		radius = centroid.radius; 
		this->maxDistSq = maxDistSq; 
		this->centroid = vertex2Feature(centroid); 
		this->maxNorAngle = maxNorAngle; 
		this->maxColDistSq = maxColDistSq; 
		this->centerMode = centerMode;
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

			centroid = (min + max) / 2;
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

	CenteringMode centerMode;


	inline void initCentroid(const EllipticalVertex& centroid, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode = CenteringMode::KEEP_SEED)
	{ 
		this->maxDistSq = maxDistSq;
		this->centroid = vertex2Feature(centroid);
		this->maxNorAngle = maxNorAngle;
		this->maxColDistSq = maxColDistSq;
		this->centerMode = centerMode;
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

			centroid = (min + max) / 2;
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

	EllipticalVertex getSplat()
	{

		if (verts.size() == 1)
		{
			return verts[0]; 
		}

		EllipticalVertex newVert = feature2Vertex(centroid);

		if (verts.size() == 2) // 2 pts wont be enough for pca
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

			
			Eigen::Matrix3f spacialCovariance = (clusterMat.transpose()* clusterMat) * (1/(verts.size()-1));
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(spacialCovariance, Eigen::ComputeEigenvectors);
			Eigen::Vector3f major = pca.eigenvectors().col(2).normalized();
			Eigen::Vector3f minor = pca.eigenvectors().col(1).normalized();

			//minor = major.cross(centroid.segment<3>(3)).normalized(); 

			//float majorLen = clusterMat.cwiseProduct(major.transpose()).rowwise().sum().maxCoeff(); //rowwise dotProduct
			//float minorLen = clusterMat.cwiseProduct(minor.transpose()).rowwise().sum().maxCoeff();

			Eigen::Vector3f nor = pca.eigenvectors().col(0).normalized();
			XMStoreFloat3(&newVert.normal, eigen2XMVector(nor));


			major *= pca.eigenvalues()(2); 
			minor *= pca.eigenvalues()(1);

			newVert.major.x = major.x();
			newVert.major.y = major.y();
			newVert.major.z = major.z();

			newVert.minor.x = minor.x();
			newVert.minor.y = minor.y();
			newVert.minor.z = minor.z();	
			/**

			size_t maxIndex; 
			clusterMat.rowwise().squaredNorm().maxCoeff(&maxIndex); 
			Eigen::Vector3f major = clusterMat.row(maxIndex);
			
			Eigen::Vector3f minorDir = major.cross(centroid.segment<3>(3)).normalized();


			float minorLen = (clusterMat * minorDir).cwiseAbs().maxCoeff();

			XMStoreFloat3(&newVert.major, eigen2XMVector(major));
			XMStoreFloat3(&newVert.minor, eigen2XMVector(minorDir*minorLen));
			/**/

//			std::cout <<"info: " << pca.info() <<  "\nmaj: " << major.transpose() << "\nmin:" << minor.transpose() << "\nEigenvalues: "
//				<< pca.eigenvalues() << "\n" << pca.eigenvectors() << "\nCluster:\n" << clusterMat <<"\nCov:\n"<< (clusterMat.transpose()* clusterMat) / static_cast<float>(verts.size() - 1)
//				<< "\nCov calc:\n"<< clusterMat.transpose()* clusterMat << std::endl;

//			std::cin.get(); 


		}

		return newVert;
	}


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
