#include "NestedOctree.h"
#include <iostream>
#include <minmax.h>


/*/
//TODO: split this up into template function since only the last part actually depends on wether Sphere or Elliptical Verts are used
void NestedOctree<SphereVertex>::createRegionGrowing(NestedOctreeNode<SphereVertex>* pNode, size_t depth)
{
	if (pNode->isLeaf()) return; //keep original data untouched
	
	if (g_lodSettings.useThreads) // MT
	{
		std::vector<std::thread*> theadpool; 

		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{

					theadpool.push_back(
						new std::thread([=] {createRegionGrowing(pNode->children[i], depth + 1);})//next level  
					);

				}
			}
		}

		for (auto thread : theadpool)
		{
			thread->join(); 
			delete thread; 
		}
	}
	else
	{
		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{
					createRegionGrowing(pNode->children[i], depth + 1); //next level  
				}
			}
		}
	}
	
	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.aquire(); 
	}

	srand(RND_SEED);

	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				lowerBound = lowerBound.cwiseMin(fv).eval();
				upperBound = upperBound.cwiseMax(fv).eval();

			}
		}
	}

	lowerBound -= lowerBound.cwiseAbs() * 0.025;
	upperBound += upperBound.cwiseAbs() * 0.025;
	Vec9f nodeRange = upperBound - lowerBound;

	// diagonal / (GridRes)^3 / 2^depth === diag of one cell at LOD
	// 
	float maxDistSq = regionConstants.maxDist * (diagonal / (gridResolution*(1 << depth)));
	maxDistSq *= maxDistSq; 
	float maxAngle = regionConstants.maxNorAngle *(1 << (reachedDepth - depth-1));
	float maxColSq = regionConstants.maxColDist *(1 << (reachedDepth - depth-1));
	maxColSq *= maxColSq; 

	UINT32 searchResolution = static_cast<UINT32>(nodeRange.head<3>().maxCoeff() / (sqrt(maxDistSq))) >> 2;
	searchResolution = max(8U,searchResolution);
	//UINT32 searchResolution = gridResolution;

	//calulates clusters for current node
	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID> vertMap(searchResolution*searchResolution);

	Eigen::Vector3f evGridStart = lowerBound.head<3>(); 
	Eigen::Vector3f evCellsize = Eigen::Vector3f::Ones() * (upperBound - lowerBound).head<3>().maxCoeff() / searchResolution;
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, searchResolution, searchResolution*searchResolution;


	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Eigen::Vector3f pos;
				pos << vert.pos.x, vert.pos.y, vert.pos.z;

				if (XMVector3LengthSq(XMLoadFloat3(&vert.normal)).m128_f32[0] <0.1f && maxAngle < XM_PIDIV2) // invalid normal -> just ditch it ( otherwise dot product w/ centroid will be 0 -> angle = pi/2 > maxAngle -> cluster is not in range of its center -> bad
				{
					continue; 
				}

				//point location in the inscribed grid (searchResolution) <<-- make this float safe
				UINT32 gridIndex = (pos - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second->push_back(vert);
				}
				else
				{
					std::vector<SphereVertex>* newVec = new std::vector<SphereVertex>(); 
					newVec->push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<SphereVertex>*>(gridIndex, newVec));
				}
			}
		}
	}
	//clustering
	UINT32 itcount = 0; 
	while (!vertMap.empty())
	{
		++itcount; 
		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<SphereVertex>& centroidCell = *it->second;
		centroidCellIndex = it->first; 

		//assert(!centroidCell.empty()); 
		UINT32 centroidIndex = rand() % centroidCell.size();

		const SphereVertex& centVertex = centroidCell[centroidIndex];

		Cluster<SphereVertex> cluster; 
		cluster.initCentroid(centVertex, maxDistSq, maxAngle, maxColSq, regionConstants.centeringMode); 

		std::queue<UINT32> frontier;
		std::unordered_set<UINT32, hashID> exploredNodes;
		frontier.push(centroidCellIndex);
		exploredNodes.insert(centroidCellIndex);

		while (!frontier.empty())
		{
			UINT32 currentIDX = frontier.front();
			frontier.pop();

			bool oneInRange = findVerticesInCellAndRemove(cluster, currentIDX, vertMap);
			if (oneInRange) // found at least one vert -> explore neighbours 
			{
				auto hull = getCellNeighbours(searchResolution, currentIDX);
				for (int idx : hull)
				{
					if (idx >= 0 && idx < searchResolution*searchResolution*searchResolution && exploredNodes.find(idx) == exploredNodes.end())
					{
						exploredNodes.insert(static_cast<UINT32>(idx));
						frontier.push(static_cast<UINT32>(idx));
					}
				}
			}
		}

		cluster.center();

		SphereVertex sv = cluster.getSplat();
		pNode->data.push_back(sv);
	}//vertMap.empty()

//	std::cout << itcount << std::endl; 

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.release();
	}
}

void NestedOctree<EllipticalVertex>::createRegionGrowing(NestedOctreeNode<EllipticalVertex>* pNode, size_t depth)
{

	if (pNode->isLeaf()) return;



	if (g_lodSettings.useThreads) // MT
	{
		std::vector<std::thread*> theadpool;

		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{

					theadpool.push_back(
						new std::thread([=] {createRegionGrowing(pNode->children[i], depth + 1); })//next level  
					);

				}
			}
		}

		for (auto thread : theadpool)
		{
			thread->join();
			delete thread;
		}
	}
	else
	{
		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{
					createRegionGrowing(pNode->children[i], depth + 1); //next level  
				}
			}
		}
	}

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.aquire();
	}

	srand(RND_SEED);

	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				lowerBound = lowerBound.cwiseMin(fv).eval();
				upperBound = upperBound.cwiseMax(fv).eval();

			}
		}
	}

	lowerBound -= lowerBound.cwiseAbs() * 0.025;
	upperBound += upperBound.cwiseAbs() * 0.025;
	Vec9f nodeRange = upperBound - lowerBound;

	// diagonal / (GridRes)^3 / 2^depth === diag of one cell at LOD
	// 
	float maxDistSq = regionConstants.maxDist * (diagonal / (gridResolution*(1 << depth)));
	maxDistSq *= maxDistSq;
	float maxAngle = regionConstants.maxNorAngle *(1 << (reachedDepth - depth - 1));
	float maxColSq = regionConstants.maxColDist *(1 << (reachedDepth - depth - 1));
	maxColSq *= maxColSq;

	//unnecessary complicated ? -- works tho
	UINT32 searchResolution = static_cast<UINT32>(nodeRange.head<3>().maxCoeff() / (sqrt(maxDistSq))) >> 2;
	searchResolution = max(8U, searchResolution);
	//searchResolution = min(128U, searchResolution);


	//UINT32 searchResolution = gridResolution;

	//calulates clusters for current node
	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	std::unordered_map < UINT32, std::vector<EllipticalVertex>*, hashID> vertMap(searchResolution*searchResolution);

	Eigen::Vector3f evGridStart = lowerBound.head<3>();
	Eigen::Vector3f evCellsize = Eigen::Vector3f::Ones() * (upperBound - lowerBound).head<3>().maxCoeff() / searchResolution;
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, searchResolution, searchResolution*searchResolution;


	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Eigen::Vector3f pos;
				pos << vert.pos.x, vert.pos.y, vert.pos.z;

				if (XMVector3LengthSq(XMLoadFloat3(&vert.normal)).m128_f32[0] <0.1f && maxAngle < XM_PIDIV2) // invalid normal -> just ditch it ( otherwise dot product w/ centroid will be 0 -> angle = pi/2 > maxAngle -> cluster is not in range of its center -> bad
				{
					continue;
				}
				//point location in the inscribed grid (searchResolution) <<-- make this float safe
				UINT32 gridIndex = (pos - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second->push_back(vert);
				}
				else
				{
					std::vector<EllipticalVertex>* newVec = new std::vector<EllipticalVertex>();
					newVec->push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<EllipticalVertex>*>(gridIndex, newVec));
				}
			}
		}
	}
	//clustering
	UINT32 itcount = 0;
	while (!vertMap.empty())
	{
		++itcount;

		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<EllipticalVertex>*, hashID>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<EllipticalVertex>& centroidCell = *it->second;
		centroidCellIndex = it->first;

		//assert(!centroidCell.empty()); 
		UINT32 centroidIndex = rand() % centroidCell.size();

		const EllipticalVertex& centVertex = centroidCell[centroidIndex];

		Cluster<EllipticalVertex> cluster;
		cluster.initCentroid(centVertex, maxDistSq, maxAngle, maxColSq, regionConstants.centeringMode);

		std::queue<UINT32> frontier;
		std::unordered_set<UINT32, hashID> exploredNodes;
		frontier.push(centroidCellIndex);
		exploredNodes.insert(centroidCellIndex);

		while (!frontier.empty())
		{
			UINT32 currentIDX = frontier.front();
			frontier.pop();

			bool oneInRange = findVerticesInCellAndRemove(cluster, currentIDX, vertMap);
			if (oneInRange) // found at least one vert -> explore neighbours 
			{
				auto hull = getCellNeighbours(searchResolution, currentIDX);
				for (int idx : hull)
				{
					if (idx >= 0 && idx < searchResolution*searchResolution*searchResolution && exploredNodes.find(idx) == exploredNodes.end())
					{
						exploredNodes.insert(static_cast<UINT32>(idx));
						frontier.push(static_cast<UINT32>(idx));
					}
				}
			}
		}

		cluster.center();

		EllipticalVertex sv = cluster.getSplat();
		pNode->data.push_back(sv);
	}//vertMap.empty()

	 //	std::cout << itcount << std::endl; 

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.release();
	}
}
*/

void NestedOctree<SphereVertex>::addVertsToCluster(std::vector<std::pair<float, SphereVertex>>* candidates, Cluster2<SphereVertex>& cluster)
{
	size_t candidateIdx = 0;
	//add all candidates; 
	for (; candidateIdx != candidates->size() && cluster.checkAdd((*candidates)[candidateIdx]); ++candidateIdx);

	std::vector<std::pair<float, SphereVertex>>(candidates->begin() + candidateIdx, candidates->end()).swap(*candidates); // cuts all Verts that were added to the cluster; 
}

void NestedOctree<EllipticalVertex>::addVertsToCluster(std::vector<std::pair<float, EllipticalVertex>>* candidates, Cluster2<EllipticalVertex>& cluster)
{
	size_t candidateIdx = 0;
	//add all candidates; 
	for (; candidateIdx != candidates->size() && cluster.checkAdd((*candidates)[candidateIdx]); ++candidateIdx);

	std::vector<std::pair<float, EllipticalVertex>>(candidates->begin() + candidateIdx, candidates->end()).swap(*candidates); // cuts all Verts that were added to the cluster; 
}