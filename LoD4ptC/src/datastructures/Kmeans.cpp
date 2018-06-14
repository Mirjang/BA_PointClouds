#include "Kmeans.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include <random>
#include <cfloat>


void Kmeans::initCentroids(const Vec9f& vmin, const Vec9f& vmax, const UINT32& numCentroids, std::vector<Centroid>& centroids)
{

	//	float range = XMVector3Length((max - min)/numCentroids * 1.5f).m128_f32[0];	//lets have some minimum spacing between initila centroids, shall we? 
	std::default_random_engine xgen, ygen, zgen;

	std::uniform_real_distribution<float> xdist(vmin(0), vmax(0));
	std::uniform_real_distribution<float> ydist(vmin(1), vmin(1));
	std::uniform_real_distribution<float> zdist(vmin(2), vmin(2));


	for (UINT32 i = 0; i < numCentroids; ++i)
	{
		Centroid cent(xdist(xgen), ydist(ygen), zdist(zgen));

		bool add = true;

		for (auto c : centroids)	//check that we dont have two centroids on the same spot
		{

			if (cent.features == c.features)
			{
				//	--i; // rnd numbers are taking too long... 
				add = false;
				break;
			}
		}
		if (add)
			centroids.push_back(cent);

	}
}

void Kmeans::updateCentroids(std::vector<Centroid>& centroids, const std::vector<Vec9f>& verts, const std::vector<UINT32>& vertCentroidTable)
{

	std::vector<UINT32> centroidMembers;
	centroidMembers.resize(centroids.size());

	for (int i = 0; i < centroids.size(); ++i)
	{
		centroids[i].features << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	}


	for (int i = 0; i < verts.size(); ++i)
	{
		Centroid& cent = centroids[vertCentroidTable[i]];
		++centroidMembers[vertCentroidTable[i]];

		cent.features += verts[i];
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
void Kmeans::updateObservations(const std::vector<Centroid>& centroids, const std::vector<Vec9f>&verts, std::vector<UINT32>& vertCentroidTable)
{
	UINT32 minDistIndex = -1; // this will throw an OOB exception if something goes wrong... hopefully 
	float minDist = FLT_MAX;

	for (int i = 0; i < verts.size(); ++i)
	{
		Vec9f vert = verts[i];
		vert(0) /= constants.maxSpacialRange;
		vert(1) /= constants.maxSpacialRange;
		vert(2) /= constants.maxSpacialRange;


		for (int c = 0; c < centroids.size(); ++c)
		{
			//use squared dist for all components
			float distance = vert.dot(centroids[c].features);

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
