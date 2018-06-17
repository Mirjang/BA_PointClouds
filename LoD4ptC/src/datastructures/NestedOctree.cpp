#include "NestedOctree.h"

#include <Eigen\dense>
#include <unordered_set>
#include "../global/Distances.h"
#include <queue>
#include <cstdlib>    
#include <ctime>      


#define RND_SEED 42

#define MIN_CENTROID_SQDIFFERENCE 0.000001f

using namespace DirectX; 

//TODO: split this up into template function since only the last part actually depends on wether Sphere or Elliptical Verts are used
void NestedOctree<SphereVertex>::createRegionGrowing(NestedOctreeNode<SphereVertex>* pNode, XMVECTOR gridStart, size_t depth)
{
	static UINT64 fuckupCTR = 0;

	gridResolution = 64; 

	if (pNode->isLeaf()) return; 

	if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
	{
		for (int i = 0; i < 8; ++i)
		{
			if (pNode->children[i])
			{
				XMVECTOR subridstart = gridStart +
					(XMLoadFloat3(&range) / (2 << depth))*XMVectorSet(i & 0x01 ? 1 : 0, i & 0x02 ? 1 : 0, i & 0x04 ? 1 : 0, 0);

				createRegionGrowing(pNode->children[i], subridstart, depth + 1); //next level  
			}				
		}
	}	
	
	srand(RND_SEED);


	//calulates clusters for current node
	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	auto hash_ID = [](UINT32 x)->size_t {return x; };
	std::unordered_map < UINT32, std::vector<SphereVertex>, decltype(hash_ID)> vertMap(gridResolution*gridResolution, hash_ID);
	XMVECTOR cellsize = XMLoadFloat3(&cellsizeForDepth[depth]);

	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				XMVECTOR relPos = XMLoadFloat3(&vert.pos) - gridStart;

				XMVECTOR cellIndex = relPos / cellsize;

				//add normals in polar coords
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
					vert.color.x, vert.color.y, vert.color.z, vert.color.w;

				lowerBound = lowerBound.cwiseMin(fv);
				upperBound = upperBound.cwiseMax(fv);

				//point location in the inscribed grid (gridresolution) <<-- make this float safe
				UINT32 gridIndex = static_cast<UINT32>(XMVectorGetX(cellIndex))
					+ gridResolution * static_cast<UINT32>(XMVectorGetY(cellIndex))
					+ gridResolution * gridResolution * static_cast<UINT32>(XMVectorGetZ(cellIndex));

				//0-8 which node this vert will go to if the tree is expanded here
				UINT32 subGridIndex = calculateSubgridIndex(relPos, depth);

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second.push_back(vert);
				}
				else
				{
					std::vector<SphereVertex> newVec;
					newVec.push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<SphereVertex>>(gridIndex, newVec));
				}
			}
		}
	}
	//clustering
	
	Vec9f invNormalisationConst = (upperBound - lowerBound);



	float sqMaxDist = regionConstants.maxDist * invNormalisationConst.squaredNorm();


	Vec9f normalisationConsts = invNormalisationConst.cwiseInverse();
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan

	Vec9f normalisationConstScaling = normalisationConsts.cwiseProduct(regionConstants.scaling); 

	Eigen::Vector3f evGridStart;
	evGridStart << gridStart.m128_f32[0], gridStart.m128_f32[1], gridStart.m128_f32[2];

	Eigen::Vector3f evCellsize;
	evCellsize << cellsizeForDepth[depth].x, cellsizeForDepth[depth].y, cellsizeForDepth[depth].z;
	Eigen::Vector3i oneGridGridSQ; 
	oneGridGridSQ << 1, gridResolution, gridResolution*gridResolution; 


	while (!vertMap.empty())
	{
		// prob.better to use existing vert as centroid
		//Vec9f centroid = ((Vec9f::Random() + Vec9f::Ones()) / 2.0f).cwiseProduct(invNormalisationConst) - lowerBound;

		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<SphereVertex>, decltype(hash_ID)>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<SphereVertex>& centroidCell = it->second;

		assert(!centroidCell.empty()); 
		UINT32 centroidIndex = rand() % centroidCell.size();

		const SphereVertex& centVertex = centroidCell[centroidIndex];

		XMVECTOR polarcoords = Distances::cartToPolarNormal(XMLoadFloat3(&centVertex.normal));

		Vec9f centroid;
		centroid << centVertex.pos.x, centVertex.pos.y, centVertex.pos.z, polarcoords.m128_f32[0], polarcoords.m128_f32[1],
			centVertex.color.x, centVertex.color.y, centVertex.color.z, centVertex.color.w;

		Vec9f lastCentroid = Vec9f::Ones() * (upperBound.squaredNorm()+24); // so 1st checl is passed ... using FLT_MAX apperently causes overflow

		MatX9f clusterVerts; 

		//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
		for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
		{
			centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
			auto result = vertMap.find(centroidCellIndex);

			if (result != vertMap.end())
			{
				std::vector<SphereVertex>& cellVector = result->second;

				for (const SphereVertex& vert : cellVector)
				{
					//add normals in polar coords	
					Vec9f fv;
					fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
						vert.color.x, vert.color.y, vert.color.z, vert.color.w;

					if ((fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() < sqMaxDist)	//vert is in range
					{
						clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

						clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
					}
				}

			}//result != vertMap.end()

			for (int i = 0; i < gridNeighboursAdj.size()&&false; ++i)
			{
				int currentIDX = centroidCellIndex + gridNeighboursAdj[i];

				if (currentIDX > 0 && currentIDX < gridResolution*gridResolution*gridResolution)	//check neighbourhood
				{
					result = vertMap.find(centroidCellIndex);

					if (result != vertMap.end())
					{
						std::vector<SphereVertex>& cellVector = result->second;

						for (const SphereVertex& vert : cellVector)
						{
							//add normals in polar coords	
							Vec9f fv;
							fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
								vert.color.x, vert.color.y, vert.color.z, vert.color.w;

							if ((fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() < sqMaxDist)	//vert is in range
							{
								clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

								clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
							}
						}

					}//result != vertMap.end()
				}
			}

			//END vertexFinding

			lastCentroid = centroid;
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows(); 
			std::cout << lastCentroid.transpose() << "\n" << centroid.transpose() << std::endl; 
			std::cout << "rows" << clusterVerts.rows() << std::endl;

			clusterVerts.resize(0, 9); 

		}//END current iteration

		//last iteration

		//calc cell the current centroid is in

		//	UINT32 fuckIdx = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ.transpose());//should work in my mind but doesnt--- FUCK 

		centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
		auto result = vertMap.find(centroidCellIndex);

		if (result != vertMap.end())
		{
			std::vector<SphereVertex>& cellVector = result->second;

			for (auto cellVecIt = cellVector.begin(); cellVecIt != cellVector.end(); )
			{
				const SphereVertex& vert = *cellVecIt;

				//add normals in polar coords	
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
					vert.color.x, vert.color.y, vert.color.z, vert.color.w;

				if ((fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() < sqMaxDist)	//vert is in range
				{
					clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

					clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
					cellVector.erase(cellVecIt);

				}
				else
				{
					++cellVecIt;
				}
			}

			if (cellVector.empty())
			{
				vertMap.erase(centroidCellIndex); 
			}


		}//result != vertMap.end()


		for (int i = 0; i < gridNeighboursAdj.size(); ++i)
		{
			int currentIDX = centroidCellIndex + gridNeighboursAdj[i];

			if (currentIDX > 0 && currentIDX < gridResolution*gridResolution*gridResolution)	//check neighbourhood
			{
				result = vertMap.find(centroidCellIndex);

				if (result != vertMap.end())
				{
					std::vector<SphereVertex>& cellVector = result->second;
					for (auto cellVecIt = cellVector.begin(); cellVecIt != cellVector.end(); )
					{
						const SphereVertex& vert = *cellVecIt;

						//add normals in polar coords	
						Vec9f fv;
						fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
							vert.color.x, vert.color.y, vert.color.z, vert.color.w;

						if ((fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() < sqMaxDist)	//vert is in range
						{
							clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

							clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
							cellVector.erase(cellVecIt);

						}
						else
						{
							++cellVecIt;
						}
					}

					if (cellVector.empty())
					{
						vertMap.erase(currentIDX);
					}

				}//result != vertMap.end()
			}
		
		}//END last iteration


		if (clusterVerts.rows() == 0)
		{
			++fuckupCTR; 
			std::cout << fuckupCTR << " FUUUUCK " << std::endl; 
			continue; 
		}

		centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
		SphereVertex newVert;
		newVert.pos.x = centroid(0);
		newVert.pos.y = centroid(1);
		newVert.pos.z = centroid(2);
		XMStoreFloat3(&newVert.normal, Distances::polarToCartNormal(XMVectorSet(centroid(3), centroid(4), 0, 0)));
		newVert.color.x = centroid(5);
		newVert.color.y = centroid(6);
		newVert.color.z = centroid(7);
		newVert.color.w = centroid(8);




		if (clusterVerts.rows() == 1)	//isolated centroid
		{
			newVert.radius = 1.0f; 
		}
		else
		{
			//calc new supervert

			
			Eigen::MatrixX3f spacialMat = clusterVerts.leftCols<3>();

			clusterVerts.resize(0, 0);

			spacialMat = spacialMat.rowwise() - centroid.head(3).transpose();


			/**  // i could also just do a search...
			Eigen::Matrix3f spacialCovariance = spacialMat.transpose()* spacialMat;
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca(spacialCovariance);


			newVert.radius = pca.eigenvalues().maxCoeff();
			/**/
			newVert.radius = sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff());

		}


		

		pNode->data.push_back(newVert); 

	}//vertMap.empty()

}

/*/
void NestedOctree<SphereVertex>::createKmeans(NestedOctreeNode<SphereVertex>* pNode, const std::vector<SphereVertex>& data, XMVECTOR gridStart, size_t depth = 0)
{

	if (depth > reachedDepth)
	{
		reachedDepth = depth;
		XMVECTOR newCellsize = XMLoadFloat3(&range) / ((1 << depth) * gridResolution);	//[was] NOT SURE ABOUT THIS [fixed it, now im a bit more sure]

		cellsizeForDepth.push_back(XMFLOAT3());
		XMStoreFloat3(&cellsizeForDepth[depth], newCellsize);

		cellMidpointForDepth.push_back(XMFLOAT3());

		XMStoreFloat3(&cellMidpointForDepth[depth], XMLoadFloat3(&range) / (2 << depth));
	}

	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	auto hash_ID = [](UINT32 x)->size_t {return x; };
	//such beautiful
	std::unordered_map < UINT32, std::vector<SphereVertex>, decltype(hash_ID)> insertMap(gridResolution*gridResolution, hash_ID); 

	UINT32 subGridOverlapps[8] = { 0 };

	std::vector<SphereVertex> subGridData[8]; //Stores verts in case the tree needs to be expanded

	XMVECTOR cellsize = XMLoadFloat3(&cellsizeForDepth[depth]);
	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	for (auto vert : data)
	{
		XMVECTOR relPos = XMLoadFloat3(&vert.pos) - gridStart;

		XMVECTOR cellIndex = relPos / cellsize;

		Vec9f fv;
		fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
			vert.color.x, vert.color.y, vert.color.z, vert.color.w;
		//point location in the inscribed grid (gridresolution) <<-- make this float safe
		UINT32 gridIndex = static_cast<UINT32>(XMVectorGetX(cellIndex))
			+ gridResolution * static_cast<UINT32>(XMVectorGetY(cellIndex))
			+ gridResolution * gridResolution * static_cast<UINT32>(XMVectorGetZ(cellIndex));

		//0-8 which node this vert will go to if the tree is expanded here
		UINT32 subGridIndex = calculateSubgridIndex(relPos, depth);

		auto result = insertMap.find(gridIndex);
		if (result != insertMap.end())
		{
			result->second.push_back(vert);
		}
		else
		{
			std::vector<SphereVertex> newVec;
			newVec.push_back(vert);
			insertMap.insert(std::pair<UINT32, std::vector<SphereVertex>>(gridIndex, newVec));
		}
	}


	Vec9f invNormalisationConst = (upperBound - lowerBound);



	float sqMaxDist = regionConstants.maxDist * invNormalisationConst.squaredNorm();


	Vec9f normalisationConsts = invNormalisationConst.cwiseInverse();
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan

	Vec9f normalisationConstScaling = normalisationConsts.cwiseProduct(regionConstants.scaling);

	Eigen::Vector3f evGridStart;
	evGridStart << gridStart.m128_f32[0], gridStart.m128_f32[1], gridStart.m128_f32[2];

	Eigen::Vector3f evInvCellsize;
	evInvCellsize << cellsizeForDepth[depth].x, cellsizeForDepth[depth].y, cellsizeForDepth[depth].z;
	evInvCellsize = evInvCellsize.cwiseInverse();
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, gridResolution, gridResolution*gridResolution;



	for (auto it : insertMap)
	{
			








		pNode->data.push_back(avgVert);

		subGridOverlapps[i] += it.second.size() - 1; //first element in grid is ok

	}
	insertMap.clear();
	


	bool expandNode = false;
	for (int i = 0; i < 8; ++i)
	{
		if (subGridOverlapps[i] > expansionThreshold)
		{
			expandNode = true;
			break;
		}
	}

	if (expandNode&&depth != maxDepth)
	{
		for (int i = 0; i < 8; ++i)// expand node(s) where to many overlapps occured
		{
			if (!subGridData[i].empty())
			{
				pNode->children[i] = new NestedOctreeNode<SphereVertex>();
				++numNodes;

				if (subGridData[i].size() > expansionThreshold)	//these nodes migth have to bee expanded even further
				{
					XMFLOAT3 gridStart3f, gridMidpoint3f;

					XMStoreFloat3(&gridMidpoint3f, gridStart + XMLoadFloat3(&range) / (2 << depth));
					XMStoreFloat3(&gridStart3f, gridStart);


					XMVECTOR subridstart = gridStart +
						(XMLoadFloat3(&range) / (2 << depth))*XMVectorSet(i & 0x01 ? 1 : 0, i & 0x02 ? 1 : 0, i & 0x04 ? 1 : 0, 0);

					createKmeans(pNode->children[i], subGridData[i], subridstart, depth + 1); //recurse one level 

				}
				else   //this will not be expanded -> leaf node -> no need for additional traversal 
				{
					//this node should be empty
					pNode->children[i]->data.insert(pNode->children[i]->data.end(), subGridData[i].begin(), subGridData[i].end());
				}
			}

			subGridData[i].clear();	//release unneeded space asap
									//	delete subGridData[i];
		}
	}
	else
	{
		pNode->data.clear();
		pNode->data = data;

		for (int i = 0; i < 8; ++i)
		{
			subGridData[i].clear();
			//delete subGridData[i];
		}

	}
}

/***/