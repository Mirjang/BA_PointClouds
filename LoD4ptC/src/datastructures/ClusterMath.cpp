#include "ClusterMath.h"
#include <minmax.h>

void Cluster<SphereVertex>::initCentroid(const SphereVertex& centroidVert, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode)
{
	verts.push_back(centroidVert);
	this->maxDistSq = maxDistSq;
	this->centroid = vertex2Feature(centroidVert);
	this->maxNorAngle = maxNorAngle;
	this->maxColDistSq = maxColDistSq;
	this->centerMode = centerMode;

	majorLen = centroidVert.radius;
	minorLen = centroidVert.radius;

	majorNor = Eigen::Vector3f::Zero(); 
	minorNor = Eigen::Vector3f::Zero();

}
void Cluster<EllipticalVertex>::initCentroid(const EllipticalVertex& centroidVert, const float& maxDistSq, const float& maxNorAngle, const float& maxColDistSq, CenteringMode centerMode)
{
	verts.push_back(centroidVert);
	this->maxDistSq = maxDistSq;
	this->centroid = vertex2Feature(centroidVert);
	this->maxNorAngle = maxNorAngle;
	this->maxColDistSq = maxColDistSq;
	this->centerMode = centerMode;


	majorNor = xmfloat2Eigen(centroidVert.major); 
	majorLen = majorNor.norm();
	majorNor /= majorLen; 

	minorNor = majorNor; 
	minorLen = xmfloat2Eigen(centroidVert.minor).norm(); 

	majorNor = Eigen::Vector3f::Zero();
	minorNor = Eigen::Vector3f::Zero();

}


bool Cluster<SphereVertex>::checkAdd(const SphereVertex& vert)
{

	Vec9f fv = vertex2Feature(vert);

	Eigen::Vector3f pc = centroid.head<3>();
	Eigen::Vector3f pv = fv.head<3>();
	Eigen::Vector3f dv = pv - pc; 

	float dist = dv.squaredNorm(); 

	float distMaj = dv.dot(majorNor); 
	float distMin = dv.dot(minorNor);

	if (dist > maxDistSq )
	{
		return false;
	}
	else
	{
		if (distMaj > majorLen || distMin > minorLen)
		{
			return false; 
		}

		distMaj += vert.radius; 
		distMin += vert.radius; 

		Eigen::Vector3f nc = centroid.segment<3>(3);
		Eigen::Vector3f nv = fv.segment<3>(3);
		Eigen::Vector3f cc = centroid.tail<3>();
		Eigen::Vector3f cv = fv.tail<3>();


		if ((cv - cc).squaredNorm() > maxColDistSq || abs(acosf(nc.dot(nv))) > maxNorAngle)
		{
			Eigen::Vector3f dvNor = dv.normalized(); 
			float angDvMaj = abs(acosf(majorNor.dot(dvNor))); 
			float angDvMin = abs(acosf(minorNor.dot(dvNor)));

			majorLen = min(majorLen, distMaj);
			minorLen = min(minorLen, distMaj);

			
			if (angDvMaj > angMajMin)
			{
				minorNor = dvNor;
				minorLen = sqrt(dist);
			}
			else if (angDvMin > angMajMin)
			{
				majorNor = dvNor;
				majorLen = sqrt(dist);
			}
			

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


bool Cluster<EllipticalVertex>::checkAdd(const EllipticalVertex& vert)
{
	Vec9f fv = vertex2Feature(vert);

	Eigen::Vector3f pc = centroid.head<3>();
	Eigen::Vector3f pv = fv.head<3>();
	Eigen::Vector3f dv = pv - pc;

	float dist = dv.squaredNorm();

	float distMaj = dv.dot(majorNor);
	float distMin = dv.dot(minorNor);

	if (dist > maxDistSq)
	{
		return false;
	}
	else
	{
		if (distMaj > majorLen || distMin > minorLen)
		{
			return false;
		}

		float vertBoundingSphere = xmfloat2Eigen(vert.major).norm();; 
		distMaj += vertBoundingSphere; 
		distMin += vertBoundingSphere; 


		Eigen::Vector3f nc = centroid.segment<3>(3);
		Eigen::Vector3f nv = fv.segment<3>(3);
		Eigen::Vector3f cc = centroid.tail<3>();
		Eigen::Vector3f cv = fv.tail<3>();


		if ((cv - cc).squaredNorm() > maxColDistSq || abs(acosf(nc.dot(nv))) > maxNorAngle)
		{
			Eigen::Vector3f dvNor = dv.normalized();
			float angDvMaj = abs(acosf(majorNor.dot(dvNor)));
			float angDvMin = abs(acosf(minorNor.dot(dvNor)));

			majorLen = min(majorLen, distMaj);
			minorLen = min(minorLen, distMaj);


			if (angDvMaj > angMajMin)
			{
				minorNor = dvNor;
				minorLen = sqrt(dist);
			}
			else if (angDvMin > angMajMin)
			{
				majorNor = dvNor;
				majorLen = sqrt(dist);
			}


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

SphereVertex Cluster<SphereVertex>::getSplat()
{
	if (verts.size() == 1)
	{
		return verts[0];
	}

	SphereVertex newVert = feature2Vertex(centroid);
	//	newVert.radius = verts.size() == 1 ? radius : sqrtf(maxDistSq); 
	newVert.radius = majorLen; 

	
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
EllipticalVertex Cluster<EllipticalVertex>::getSplat()
{

	if (verts.size() == 1)
	{
		return verts[0];
	}
	minorNor = majorNor.cross(centroid.segment<3>(3)).normalized(); 

	EllipticalVertex newVert = feature2Vertex(centroid);

	Eigen::Vector3f pc = centroid.head<3>();
	for (auto vert : verts)
	{
		Eigen::Vector3f pv;
		pv << vert.pos.x, vert.pos.y, vert.pos.z;
		float vertBoundingSphere = xmfloat2Eigen(vert.major).norm();
		float dist = (pv - pc).norm() + vertBoundingSphere;
		majorLen = max(majorLen, dist);

		minorLen = max(minorLen, abs(minorNor.dot(pc - pv)) + vertBoundingSphere); 

	}


	XMStoreFloat3(&newVert.major, eigen2XMVector(majorNor*majorLen)); 
	XMStoreFloat3(&newVert.minor, eigen2XMVector(minorNor * minorLen));

	return newVert; 
	/**
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


		Eigen::Matrix3f spacialCovariance = (clusterMat.transpose()* clusterMat) * (1 / (verts.size() - 1));
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
		/**

		//			std::cout <<"info: " << pca.info() <<  "\nmaj: " << major.transpose() << "\nmin:" << minor.transpose() << "\nEigenvalues: "
		//				<< pca.eigenvalues() << "\n" << pca.eigenvectors() << "\nCluster:\n" << clusterMat <<"\nCov:\n"<< (clusterMat.transpose()* clusterMat) / static_cast<float>(verts.size() - 1)
		//				<< "\nCov calc:\n"<< clusterMat.transpose()* clusterMat << std::endl;

		//			std::cin.get(); 


	}
	
	return newVert;
	/**/
}



bool Cluster2<SphereVertex>::checkAdd(const std::pair<float, SphereVertex>& candidate)
{
	Vec9f fv = vertex2Feature(candidate.second);
	
	Eigen::Vector3f cc = centroid.tail<3>();
	Eigen::Vector3f cv = fv.tail<3>();

	if ((cv - cc).squaredNorm() > maxColDistSq)
	{
		return false;
	}
	else
	{
		Eigen::Vector3f nc = centroid.segment<3>(3);
		Eigen::Vector3f nv = fv.segment<3>(3);
		if (abs(acosf(nc.dot(nv))) > maxNorAngle) // normal 
		{
			return false;
		}
		else
		{
			//vert in range -> add
			verts.push_back(candidate);
			return true;
		}
	}		
}


SphereVertex Cluster2<SphereVertex>::getSplat()
{
	SphereVertex newVert = feature2Vertex(centroid);
	newVert.radius = sqrt(verts.back().first) + verts.back().second.radius;
	return newVert;
}

bool Cluster2<EllipticalVertex>::checkAdd(const std::pair<float, EllipticalVertex>& candidate)
{
	Vec9f fv = vertex2Feature(candidate.second);

	Eigen::Vector3f cc = centroid.tail<3>();
	Eigen::Vector3f cv = fv.tail<3>();

	if ((cv - cc).squaredNorm() > maxColDistSq)
	{
		minorLengthSq = candidate.first;
		minor = fv.head<3>() - centroid.head<3>(); 
		return false;
	}
	else
	{
		Eigen::Vector3f nc = centroid.segment<3>(3);
		Eigen::Vector3f nv = fv.segment<3>(3);
		if (abs(acosf(nc.dot(nv))) > maxNorAngle) // normal 
		{
			minorLengthSq = candidate.first;
			return false;
		}
		else
		{
			//vert in range -> add
			verts.push_back(candidate);
			return true;
		}
	}
}


EllipticalVertex Cluster2<EllipticalVertex>::getSplat()
{
	EllipticalVertex newVert = feature2Vertex(centroid);
	XMStoreFloat3(&newVert.major, eigen2XMVector(major));
	XMStoreFloat3(&newVert.minor, eigen2XMVector(minor)); 

	return newVert;
}
