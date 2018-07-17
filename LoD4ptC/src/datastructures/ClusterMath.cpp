#include "ClusterMath.h"
#include <minmax.h>

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
