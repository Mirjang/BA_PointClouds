#include "Kmeans.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include <cfloat>


void Kmeans::initCentroids(const UINT32& numCentroids)
{

	//	float range = XMVector3Length((max - min)/numCentroids * 1.5f).m128_f32[0];	//lets have some minimum spacing between initila centroids, shall we? 

	for (UINT32 i = 0; i < numCentroids; ++i)
	{
		Centroid cent; 

		cent.features = ((Vec9f::Random() + Vec9f::Ones()) /2.0f).cwiseProduct(constants.upperBound) - constants.lowerBound;

		bool add = true;

		/** //remove duplicate centroids
		for (auto c : centroids)	//check that we dont have two centroids on the same spot
		{

			if (cent.features == c.features)
			{
				//	--i; // rnd numbers are taking too long... 
				add = false;
				break;
			}
		}
		/**/
		if (add)
			centroids.push_back(cent);

	}
}

void Kmeans::runKMEANS(const MatX9f& verts, UINT32 numCentroids, UINT32 iterations)
{
	vertCentroidTable.resize(verts.size());
	initCentroids(numCentroids);

	for (UINT32 i = 0; i < iterations; ++i)
	{
		//		auto start = std::chrono::high_resolution_clock::now();
		updateObservations(verts);
		//		std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
		//		std::cout << "Observations: " << elapsed.count() << std::endl; 
		//		start = std::chrono::high_resolution_clock::now();
		updateCentroids(verts);
		//		std::chrono::duration<double> elapsed2 = std::chrono::high_resolution_clock::now() - start;
		//		std::cout << "Observations: " << elapsed2.count() << std::endl;
	}
}

void Kmeans::updateCentroids(const MatX9f& data)
{

	std::vector<UINT32> centroidMembers;
	centroidMembers.resize(centroids.size());

	for (int i = 0; i < centroids.size(); ++i)
	{
		centroids[i].features = Vec9f::Zero();
	}


	for (int i = 0; i < data.rows(); ++i)
	{
		Centroid& cent = centroids[vertCentroidTable[i]];
		++centroidMembers[vertCentroidTable[i]];

		cent.features += data.row(i);
	}

	//divide components by #of verts in cluster
	for (int i = 0; i < centroids.size(); ++i)
	{
		if (!centroidMembers[i])//very degenerate cluster ?
		{
			continue;
		}
		centroids[i].features /= centroidMembers[i];
	}
}

//Distance: xyz are divided by max(x,y,z) of the bounding box, normals and colors are already in a normalized space
void Kmeans::updateObservations(const MatX9f& verts)
{
	UINT32 minDistIndex = -1; // this will throw an OOB exception if something goes wrong... hopefully 
	float minDist = FLT_MAX;

	for (int i = 0; i < verts.rows(); ++i)
	{
		for (int c = 0; c < centroids.size(); ++c)
		{
			//use squared dist for all components
			float distance = (verts.row(i).transpose() - centroids[c].features).squaredNorm();

			//std::cout << distance << std::endl; 

			if (distance < minDist)
			{
				minDist = distance;
				minDistIndex = c;
			}
		}
		vertCentroidTable[i] = minDistIndex;

		minDistIndex = -1;
		minDist = FLT_MAX;
	}
}
