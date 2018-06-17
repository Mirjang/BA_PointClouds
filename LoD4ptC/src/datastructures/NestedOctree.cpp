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

	Eigen::Vector3f evInvCellsize;
	evInvCellsize << cellsizeForDepth[depth].x, cellsizeForDepth[depth].y, cellsizeForDepth[depth].z;
	evInvCellsize = evInvCellsize.cwiseInverse(); 
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

		std::unordered_set<UINT32, decltype(hash_ID)> exploredNodes(1, hash_ID);
		std::queue<UINT32> frontier;

		//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
		for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
		{
			//calc cell the current centroid is in
			centroidCellIndex = (centroid.head(3) - evGridStart).cwiseProduct(evInvCellsize).cast<int>().dot(oneGridGridSQ);

			exploredNodes.clear();
			frontier = std::queue<UINT32>();

			frontier.push(centroidCellIndex);
			exploredNodes.insert(centroidCellIndex); 

			while (!frontier.empty())
			{

		//		std::cout << frontier.size()<<  "\n" << exploredNodes.size() << std::endl; 
				UINT32 currentCellIndex = frontier.front(); 
				frontier.pop(); 



				bool expandFrontier = false; 

				auto result = vertMap.find(currentCellIndex);

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
							expandFrontier = true; //if we found at least one vert that goes into the cluster -> search adj cells
						}
					}

				}//result != vertMap.end()

				if (expandFrontier || !clusterVerts.rows())	//rows == 0 -> dist too large? 
				{
					for (int i = 0; i < gridNeighboursAdj.size(); ++i)
					{
						int newIdx = currentCellIndex + gridNeighboursAdj[i];

						if (newIdx > 0 && newIdx < gridResolution*gridResolution*gridResolution && exploredNodes.find(newIdx) == exploredNodes.end())
						{
							exploredNodes.insert(newIdx);
							frontier.push(newIdx);
						}
					}
				}

			}//END vertexFinding

			lastCentroid = centroid;
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows(); 
	//		std::cout << lastCentroid.transpose() << "\n" << centroid.transpose() << std::endl; 
	//		std::cout << "rows" << clusterVerts.rows() << std::endl;

			clusterVerts.resize(0, 9); 

		}//END current iteration

		//last iteration

		centroidCellIndex = (centroid.head(3) - evGridStart).cwiseProduct(evInvCellsize).cast<int>().dot(oneGridGridSQ);

		exploredNodes.clear();
		frontier = std::queue<UINT32>();

		frontier.push(centroidCellIndex);
		exploredNodes.insert(centroidCellIndex);

		while (!frontier.empty())
		{
			UINT32 currentCellIndex = frontier.front();
			frontier.pop();
			size_t rows = clusterVerts.rows();

			auto result = vertMap.find(currentCellIndex);

			if (result != vertMap.end())
			{

				std::vector<SphereVertex>& cellVector = result->second;

				for (auto it = cellVector.begin(); it != cellVector.end();)
				{
					SphereVertex& vert = *it; 
					//add normals in polar coords
					Vec9f fv;
					fv << vert.pos.x, vert.pos.y, vert.pos.z, Distances::zenith(vert.normal), Distances::azimuth(vert.normal),
						vert.color.x, vert.color.y, vert.color.z, vert.color.w;


					if ((fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() < sqMaxDist)	//vert is in range
					{
						clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

						clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();

						cellVector.erase(it);
					}
					else
					{
						++it; 
					}
				}

				if (cellVector.empty())
				{
					vertMap.erase(currentCellIndex); 
				}

				if (rows < clusterVerts.rows()) //if we found at least one vert that goes into the cluster -> search adj cells
				{
					for (int i = 0; i < gridNeighboursAdj.size(); ++i)
					{
						int newIdx = currentCellIndex + gridNeighboursAdj[i];

						if (newIdx > 0 && newIdx < gridResolution*gridResolution*gridResolution && exploredNodes.find(newIdx) == exploredNodes.end() && vertMap.find(currentCellIndex) != vertMap.end())
						{
							exploredNodes.insert(newIdx);
							frontier.push(newIdx);
						}
					}
				}


			}

		}//END last iteration

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
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows();

			Eigen::MatrixX3f spacialMat = clusterVerts.leftCols<3>();

			clusterVerts.resize(0, 0);

			spacialMat = spacialMat.rowwise() - centroid.head(3).transpose();

			Eigen::Matrix3f spacialCovariance = spacialMat.transpose()* spacialMat;
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca(spacialCovariance);


			newVert.radius = pca.eigenvalues()(0);
		}


		

		pNode->data.push_back(newVert); 

	}//vertMap.empty()

}