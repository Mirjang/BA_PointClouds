#include "NestedOctree.h"

#include <Eigen\dense>
#include <unordered_set>
#include "../global/Distances.h"
#include <queue>
#include <cstdlib>    
#include <ctime>      

#define VERBOSE 
#undef VERBOSE

#define RND_SEED 42

#define MIN_CENTROID_SQDIFFERENCE 0.000001f


using namespace DirectX; 

auto hash_ID = [](UINT32 x)->size_t {return x; };
static UINT64 fuckupCTR = 0;


inline void findVerticesInCell(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist,  const UINT32 cellidx,
	const std::unordered_map < UINT32, std::vector<SphereVertex>*, decltype(hash_ID)>& vertMap, MatX9f& clusterVerts)
{
	auto result = vertMap.find(cellidx);

	
	if (result != vertMap.end())
	{
		std::vector<SphereVertex>& cellVector = *result->second;

		for (const SphereVertex& vert : cellVector)
		{
			//add normals in polar coords	
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;


			float dist = (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm();

#ifdef VERBOSE
			std::cout << "cellIdx: " << cellidx << std::endl;
			std::cout << (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm() << " | " << sqMaxDist << std::endl;
#endif // VERBOSE

			if (dist < sqMaxDist)	//vert is in range
			{
				clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

				clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
			}
		}

	}//result != vertMap.end()
}


inline void findVerticesInCellAndRemove(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist, const UINT32 cellidx,
	std::unordered_map < UINT32, std::vector<SphereVertex>*, decltype(hash_ID)>& vertMap, MatX9f& clusterVerts)
{
	auto result = vertMap.find(cellidx);


	if (result != vertMap.end())
	{
		std::vector<SphereVertex>* cellVector = result->second;

		for (auto it = cellVector->begin(); it!=cellVector->end(); )
		{
			const SphereVertex& vert = *it; 
			//add normals in polar coords	
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;


			float dist = (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm();

#ifdef VERBOSE
			std::cout << "cellIdx: " << cellidx << std::endl;
			std::cout << dist << " | " << sqMaxDist << "   " << (dist < sqMaxDist) << std::endl;
#endif // VERBOSE

			if (dist < sqMaxDist)	//vert is in range
			{
				clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

				clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
				it = cellVector->erase(it); 
			}
			else
			{
				++it; 
			}
		}

		if (cellVector->empty())
		{
			vertMap.erase(cellidx); 
		}


	}//result != vertMap.end()
}

//TODO: split this up into template function since only the last part actually depends on wether Sphere or Elliptical Verts are used
void NestedOctree<SphereVertex>::createRegionGrowing(NestedOctreeNode<SphereVertex>* pNode, XMVECTOR gridStart, size_t depth)
{

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
	std::unordered_map < UINT32, std::vector<SphereVertex>*, decltype(hash_ID)> vertMap(gridResolution, hash_ID);

	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	Eigen::Vector3f evGridStart;
	evGridStart << gridStart.m128_f32[0], gridStart.m128_f32[1], gridStart.m128_f32[2];

	Eigen::Vector3f evCellsize;
	evCellsize << cellsizeForDepth[depth].x, cellsizeForDepth[depth].y, cellsizeForDepth[depth].z;
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, gridResolution, gridResolution*gridResolution;


	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				lowerBound = lowerBound.cwiseMin(fv);
				upperBound = upperBound.cwiseMax(fv);

				//point location in the inscribed grid (gridresolution) <<-- make this float safe
				UINT32 gridIndex = (fv.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ); 

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
	
	Vec9f invNormalisationConst = (upperBound - lowerBound);


	//default: search half a grid
	float sqMaxDist = regionConstants.maxDist * (invNormalisationConst / (gridResolution * 2)).squaredNorm(); 

	Vec9f normalisationConsts = invNormalisationConst.cwiseInverse();
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan

	Vec9f normalisationConstScaling = normalisationConsts.cwiseProduct(regionConstants.scaling); 



	while (!vertMap.empty())
	{
		// prob.better to use existing vert as centroid
		//Vec9f centroid = ((Vec9f::Random() + Vec9f::Ones()) / 2.0f).cwiseProduct(invNormalisationConst) - lowerBound;

		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<SphereVertex>*, decltype(hash_ID)>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<SphereVertex>& centroidCell = *it->second;

		assert(!centroidCell.empty()); 
		UINT32 centroidIndex = rand() % centroidCell.size();

		const SphereVertex& centVertex = centroidCell[centroidIndex];

		Vec9f centroid;
		centroid << centVertex.pos.x, centVertex.pos.y, centVertex.pos.z, centVertex.normal.x, centVertex.normal.y, centVertex.normal.z,
			centVertex.color.x, centVertex.color.y, centVertex.color.z;

		Vec9f lastCentroid = Vec9f::Ones() * (upperBound.squaredNorm()+24); // so 1st checl is passed ... using FLT_MAX apperently causes overflow

		MatX9f clusterVerts; 


		//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
		for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
		{
			//std::unordered_set<UINT32, decltype(hash_ID)> exploredNodes(30, hash_ID); 
#ifdef VERBOSE
			std::cout << (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>() << std::endl; 
#endif // VERBOSE

			centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

			findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, centroidCellIndex, vertMap, clusterVerts); 


			for (int i = 0; i < gridNeighboursAdj.size(); ++i)
			{
				int currentIDX = centroidCellIndex + gridNeighboursAdj[i];

				if (currentIDX > 0 && currentIDX < gridResolution*gridResolution*gridResolution)	//check neighbourhood
				{
//					exploredNodes.insert(centroidCellIndex);
					findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
				}
			}


			//END vertexFinding

			if (clusterVerts.rows() == 0)	//this should nevernot happen if everything goes according to plan... BUT IT DOES FUUUUU
			{
#ifdef VERBOSE
				std::cout << "rows: " << clusterVerts.rows() << std::endl;

				std::cout << "searching EVERYTHING FFS" << std::endl; 
					
				for (auto mapIt = vertMap.begin(); mapIt != vertMap.end(); ++mapIt)
				{
					findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, mapIt->first , vertMap, clusterVerts);
				}


				std::cout << "rows: " << clusterVerts.rows() << std::endl;

				++fuckupCTR;
				std::cout << fuckupCTR << " FUUUUCK " << std::endl;
				std::cout << lastCentroid.transpose() << "\n" << centroid.transpose() << std::endl;

				std::cout << "last idx: " << (lastCentroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().transpose() << std::endl;
				std::cout << "new idx : " << (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().transpose() << std::endl;
				std::cout << fuckupCTR << " FUUUUCK " << std::endl;
#endif // VERBOSE
				continue; 
				
			}

			lastCentroid = centroid;
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows(); 
			clusterVerts.resize(0, 9); 

#ifdef VERBOSE
			std::cout << "------------endit -------------------------" << std::endl; 
#endif

		}//END current iteration

		//last iteration
		centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

		findVerticesInCellAndRemove(centroid, normalisationConstScaling, sqMaxDist, centroidCellIndex, vertMap, clusterVerts);

		for (int i = 0; i < gridNeighboursAdj.size(); ++i)
		{
			int currentIDX = centroidCellIndex + gridNeighboursAdj[i];

			if (currentIDX > 0 && currentIDX < gridResolution*gridResolution*gridResolution)	//check neighbourhood
			{
				findVerticesInCellAndRemove(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);

			}
		
		}//END last iteration


		if (clusterVerts.rows() == 0)
		{
			++fuckupCTR; 
#ifdef VERBOSE
			std::cout << fuckupCTR << " FUUUUCK " << std::endl;
#endif // VERBOSE
			continue; 
		}

		centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
		SphereVertex newVert;
		newVert.pos.x = centroid(0);
		newVert.pos.y = centroid(1);
		newVert.pos.z = centroid(2);
		newVert.normal.x = centroid(3);
		newVert.normal.y = centroid(4);
		newVert.normal.z = centroid(5);
		newVert.color.x = centroid(6);
		newVert.color.y = centroid(7);
		newVert.color.z = centroid(8);
		newVert.color.w = 1.0f; //ignoring alpha




		if (clusterVerts.rows() == 1)	//isolated vertex
		{
			newVert.radius = 1.0f; 
		}
		else
		{
			//calc new supervert

			
			Eigen::MatrixX3f spacialMat = clusterVerts.leftCols<3>();
			clusterVerts.resize(0, 9);

			spacialMat = spacialMat.rowwise() - centroid.head<3>().transpose();


			/**  // i could also just do a search...
			Eigen::Matrix3f spacialCovariance = spacialMat.transpose()* spacialMat;
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca(spacialCovariance);


			newVert.radius = pca.eigenvalues().maxCoeff();
			/**/
			newVert.radius = 1.0f + sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff());

		}

		pNode->data.push_back(newVert); 

	}//vertMap.empty()

}