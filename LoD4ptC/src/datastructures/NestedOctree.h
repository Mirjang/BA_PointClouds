#pragma once
#pragma once

#include <DirectXMath.h>
#include <cmath>
#include <cassert>
#include <vector>
#include <unordered_map>
#include <basetsd.h>
#include <queue>
#include <random>
#include <Eigen\dense>
#include <unordered_set>
#include <queue>
#include <cstdlib>    
#include <ctime>      
#include <thread>

#include "../rendering/Vertex.h"
#include "../global/Distances.h"
#include "Kmeans.h"
#include "../global/utils.h"
#include "../global/Semaphore.h"

//flags indicating if child is at _POS 0 = left/down/front

#define X0Y0Z0 0b00000001
#define X1Y0Z0 0b00000010
#define X0Y1Z0 0b00000100
#define X1Y1Z0 0b00001000
#define X0Y0Z1 0b00010000
#define X1Y0Z1 0b00100000
#define X0Y1Z1 0b01000000
#define X1Y1Z1 0b10000000

#define GridIndex(x,y,z, resolution) x + y*resolution + z*resolution*resolution

#define RND_SEED 42

#define MAX_THREADS 7 
static Semaphore runNodeCalculations(MAX_THREADS);

#define MIN_CENTROID_SQDIFFERENCE 0.000001f

struct hashID
{
	size_t operator() (UINT32 x) const
	{
		return x; 
	}
};

using namespace DirectX;

typedef bool(*DistanceCheck)(const XMVECTOR&, const XMVECTOR&, const XMVECTOR&);

enum OctreeFlags
{
	createCube = 0x00,
	createAdaptive = 0x01 << 0,

	dfEuclididan = 0x01 << 1,
	dfManhattan = 0x01 << 2,

	neighbourhoodSimple = 0x01 << 3,
	neighbourhoodFull = 0x01 << 4,

	normalsUse = 0x01 << 5,
	normalsGenerate = 0x01 << 6,

	defaultFlags = createCube | dfEuclididan | neighbourhoodFull | normalsUse
};


enum OctreeCreationMode
{
	CreateAndPushDown,
	CreateNaiveAverage,
	CreatePossionDisk,
//	CreateRegionGrowingTopDown, //called seperatly because other parameters
	CreateNoAction,
};

template<class Type>
struct NestedOctreeNode
{

	NestedOctreeNode()
	{
		for (int i = 0; i < 8; ++i)
		{
			children[i] = nullptr;
		}
	}

	~NestedOctreeNode()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i])
			{
				delete children[i]; 
				children[i] = nullptr;
			}
		}
	}

	bool allChildrenLeafOrMarked()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i] && !(children[i]->marked || children[i]->isLeaf()))
				return false;
		}
		return true;
	}

	bool allChildrenMarked()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i] && !children[i]->marked)
				return false;
		}
		return true;
	}

	bool allChildrenLeafs()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i] && !children[i]->isLeaf())
				return false; 
		}
		return true; 
	}

	bool isLeaf()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (children[i]) return false; 
		}
		return true; 
	}

	//000 = left/down/front
	NestedOctreeNode<Type>* children[8];
	std::vector<Type> data; //data tbd via subsampling/averaging/pca etc.. or actual input (at leaves)--- (Mis)used as buffer during creation !

	bool marked = false; 

};

template <class NodeData>
struct OctreeVectorNode
{
	NodeData data; 
	UINT8 children = 0; 
	UINT32 firstChildIndex = 0; 
};

template<class Type>
class NestedOctree
{
	static_assert(std::is_base_of<Vertex, Type>::value, "Type needs to be derived from vertex");

public:
	
	//int64 just in case, since we are using UINT32 for girdresolution 
	std::vector<int> gridNeighboursAdj; 

	NestedOctree(const std::vector<Type>& data, UINT32 gridResolution, UINT32 expansionThreshold, UINT32 maxDepth, OctreeCreationMode mode = OctreeCreationMode::CreateAndPushDown, UINT64 flags = OctreeFlags::defaultFlags)
		: gridResolution(gridResolution),expansionThreshold(expansionThreshold), maxDepth(maxDepth), flags(flags)
	{
		if (!data.size()) //empty array
		{
			std::cout << "Octree create called with emply array" << std::endl;
			return;
		}

		gridNeighboursAdj.push_back(GridIndex(1, 0, 0, gridResolution));
		gridNeighboursAdj.push_back(GridIndex(-1, 0, 0, gridResolution));
		gridNeighboursAdj.push_back(GridIndex(0, 1, 0, gridResolution));
		gridNeighboursAdj.push_back(GridIndex(0, -1, 0, gridResolution));
		gridNeighboursAdj.push_back(GridIndex(0, 0, 1, gridResolution));
		gridNeighboursAdj.push_back(GridIndex(0, 0, -1, gridResolution));

		if (flags&OctreeFlags::neighbourhoodFull)
		{
			//edges
			gridNeighboursAdj.push_back(GridIndex(1, 1, 0, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, 1, 0, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(1, -1, 0, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, -1, 0, gridResolution));

			gridNeighboursAdj.push_back(GridIndex(1, 0, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, 0, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(1, 0, -1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, 0, -1, gridResolution));

			gridNeighboursAdj.push_back(GridIndex(0, 1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(0, -1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(0, 1, -1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(0, -1, -1, gridResolution));
			//corners

			gridNeighboursAdj.push_back(GridIndex(-1, 1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, -1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, 1, -1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(-1, -1, -1, gridResolution));

			gridNeighboursAdj.push_back(GridIndex(1, 1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(1, -1, 1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(1, 1, -1, gridResolution));
			gridNeighboursAdj.push_back(GridIndex(1, -1, -1, gridResolution));
		}

		root = new NestedOctreeNode<Type>();
		++numNodes;

		XMVECTOR vmin = XMLoadFloat3(&data[0].pos);
		XMVECTOR vmax = XMLoadFloat3(&data[0].pos);

		for (auto it : data)
		{
			XMVECTOR pos = XMLoadFloat3(&it.pos);
			vmin = XMVectorMin(pos, vmin);
			vmax = XMVectorMax(pos, vmax);
		}

		vmax *= 1.001f; // make grid a bit bigger so edge cases can be ignored

		if (flags & OctreeFlags::createCube)
		{
			float cubeBounds = max(vmax.m128_f32[0], max(vmax.m128_f32[1], vmax.m128_f32[2])); 
			vmax = XMVectorSet(cubeBounds, cubeBounds, cubeBounds, 0); 
		}

		if (flags & OctreeFlags::dfManhattan)
		{
			distanceCheck = &Distances::distanceCheckManhattan; 
		}

		XMVECTOR vrange = vmax - vmin;
		XMVECTOR vcenter = vmin + vrange / 2; 

		diagonal = XMVector3Length(vrange).m128_f32[0]; 
		diagonalSq = XMVector3LengthSq(vrange).m128_f32[0];

		cellsizeForDepth.resize(1); 
		XMStoreFloat3(&cellsizeForDepth[0], vrange/gridResolution);		
		cellMidpointForDepth.resize(1); 
		XMStoreFloat3(&cellMidpointForDepth[0], vrange / 2);
		//vrange *= 1.0001; 
		
		XMStoreFloat3(&boundsMin, vmin);
		XMStoreFloat3(&boundsMax, vmax);
		XMStoreFloat3(&range, vrange);
		XMStoreFloat3(&center, vcenter);

		switch (mode)
		{
		case CreateAndPushDown:
		{
			createAndPushDown(root, data, vmin);
			break;
		}
		case CreateNaiveAverage:
		{
			createNaiveAverage(root, data, vmin);
			break;
		}
		case CreatePossionDisk:
		{
			createPossionDisk(root, data, vmin);
			break; 
		}
		default:
			break;
		}
	}

	~NestedOctree() { delete root; }

	std::vector<UINT32> getCellHull(UINT32 res, UINT32 index = 0)
	{
		std::vector<UINT32> hull; 
		hull.push_back(index + GridIndex(0, 0, 0, res));

		hull.push_back(index + GridIndex(1, 0, 0, res));
		hull.push_back(index + GridIndex(-1, 0, 0, res));
		hull.push_back(index + GridIndex(0, 1, 0, res));
		hull.push_back(index + GridIndex(0, -1, 0, res));
		hull.push_back(index + GridIndex(0, 0, 1, res));
		hull.push_back(index + GridIndex(0, 0, -1, res));

		//edges
		hull.push_back(index + GridIndex(1, 1, 0, res));
		hull.push_back(index + GridIndex(-1, 1, 0, res));
		hull.push_back(index + GridIndex(1, -1, 0, res));
		hull.push_back(index + GridIndex(-1, -1, 0, res));

		hull.push_back(index + GridIndex(1, 0, 1, res));
		hull.push_back(index + GridIndex(-1, 0, 1, res));
		hull.push_back(index + GridIndex(1, 0, -1, res));
		hull.push_back(index + GridIndex(-1, 0, -1, res));

		hull.push_back(index + GridIndex(0, 1, 1, res));
		hull.push_back(index + GridIndex(0, -1, 1, res));
		hull.push_back(index + GridIndex(0, 1, -1, res));
		hull.push_back(index + GridIndex(0, -1, -1, res));

		//corners
		hull.push_back(index + GridIndex(-1, 1, 1, res));
		hull.push_back(index + GridIndex(-1, -1, 1, res));
		hull.push_back(index + GridIndex(-1, 1, -1, res));
		hull.push_back(index + GridIndex(-1, -1, -1, res));

		hull.push_back(index + GridIndex(1, 1, 1, res));
		hull.push_back(index + GridIndex(1, -1, 1, res));
		hull.push_back(index + GridIndex(1, 1, -1, res));
		hull.push_back(index + GridIndex(1, -1, -1, res));

		return hull; 
	}

	/*
	* returns structure of the octree as std::vector with every node holding NodeData and indices of childnodes
	* structure is built breadth first
	* implemented with just the right amount of unnecessary complexity
	*/
	template<class NodeData, typename... AssignFunctionArgs>
	void getStructureAsVector(std::vector<OctreeVectorNode<NodeData>> &out, NodeData (*assignFunction)(NestedOctreeNode<Type>*, AssignFunctionArgs...)=nullptr, AssignFunctionArgs... assignFunctionArgs)
	{
		UINT currenIndex = 0; 
		out.resize(numNodes); 
		
		std::queue<NestedOctreeNode<Type>*> nodebuffer;

		nodebuffer.push(root); 

		while(!nodebuffer.empty())
		{
			NestedOctreeNode<Type>* pNode = nodebuffer.front(); 
			nodebuffer.pop();
			OctreeVectorNode<NodeData> currentNode;

			if (assignFunction)
			{
				currentNode.data = assignFunction(pNode, assignFunctionArgs...);
			}
			currentNode.firstChildIndex = currenIndex + nodebuffer.size() + 1;

			int childCounter = 0; 
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i]&&!pNode->children[i]->data.empty())
				{
					currentNode.children |= 1 << i; //set flag at child location 

					nodebuffer.push(pNode->children[i]); 
				}
			}

			out[currenIndex] = currentNode; 
			++currenIndex;
		}
	}

	inline UINT32 calculateSubgridIndex(XMVECTOR relPos, size_t depth)
	{
		XMVECTOR vsubgridIndex = XMVectorLess(relPos, XMLoadFloat3(&cellMidpointForDepth[depth]));
		return (XMVectorGetX(vsubgridIndex) ? 0x00 : 0x01) //x
			| (XMVectorGetY(vsubgridIndex) ? 0x00 : 0x02)  //y
			| (XMVectorGetZ(vsubgridIndex) ? 0x00 : 0x04); //z
	}

	//creates Regions bottom up --> tree should be initialized w/ flag: Create and Pushdown
	void createRegionGrowing(float maxFeatureDist = 1.0f, float wPos = 1.0f, float wNor = 0.0f, float wCol = 0.0f, UINT32 maxIterations = 10)
	{
		UINT32 oldRes = gridResolution; //use lower res for more efficient search?
		regionConstants.scaling(1) = wPos;
		regionConstants.scaling(2) = wPos;
		regionConstants.scaling(3) = wPos;
		regionConstants.scaling(4) = wNor;
		regionConstants.scaling(5) = wNor;
		regionConstants.scaling(6) = wNor;
		regionConstants.scaling(7) = wCol;
		regionConstants.scaling(8) = wCol;
		regionConstants.scaling(9) = wCol;

		regionConstants.scaling /= regionConstants.scaling.norm(); //normalized weights

		regionConstants.maxDistScaling = maxFeatureDist;
		regionConstants.maxIterations = maxIterations;

		createRegionGrowing(root, 0);

		gridResolution = oldRes;
	}

	/** //buggy and prob. unnecessary? 
	void createRegionGrowingTopDown(const std::vector<Type>& data, float maxFeatureDist = 1.0f, float featureScaling[9] = { 1,1,1,1,1,1,1,1,1 }, UINT32 maxIterations = 10)
	{
		for (int i = 0; i < 9; ++i)
		{
			regionConstants.scaling(i) = featureScaling[i];
		}
		regionConstants.maxDistScaling = maxFeatureDist;
		regionConstants.maxIterations = maxIterations;

		createRegionGrowingTopDown(root, data, 0);
	}
	/**/
	
	NestedOctreeNode<Type>* root = nullptr;
	size_t numNodes = 0;
	size_t reachedDepth = 0;
	size_t maxDepth = 32; 
	UINT32 gridResolution; //res of the inscribed grid (lower res->deeper tree)
	UINT32 expansionThreshold; //only expand node after this many overlaps
	UINT32 upsamplingFactor; //combine $factor nodes to one higher level node 

	std::vector<XMFLOAT3> cellsizeForDepth;
	std::vector<XMFLOAT3> cellMidpointForDepth;

	XMFLOAT3 boundsMin, boundsMax, range, center;
	float diagonal, diagonalSq;

private:
	
	UINT64 flags = 0x0; 
	DistanceCheck distanceCheck = &Distances::distanceCheckEuclidian; 

	struct RegionGrowingConstants
	{
		Vec9f scaling; 
		float maxDistScaling;
		UINT32 maxIterations; 
	};

	RegionGrowingConstants regionConstants;

	void createRegionGrowingTopDown(NestedOctreeNode<Type>* pNode, const std::vector<Type>& data, size_t depth = 0)
	{
		if (data.size() < expansionThreshold || depth == maxDepth) // leaf
		{
			pNode->data = data; 
			return; 
		}

		srand(RND_SEED);

		//
		Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
		Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);
		for (auto vert : data)
		{
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;

			lowerBound = lowerBound.cwiseMin(fv);
			upperBound = upperBound.cwiseMax(fv);

		}
		upperBound += upperBound.cwiseAbs() * 0.025;
		Vec9f nodeRange = (upperBound - lowerBound);
		//calulates clusters for current node
		// gridRes^3 hashmap w/ chaining
		// use id as hash function, since we already compute the key (= grid index) 
		std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID> vertMap(gridResolution*gridResolution);

		Eigen::Vector3f evGridStart = lowerBound.head<3>();
		Eigen::Vector3f evCellsize = Eigen::Vector3f::Ones() * (upperBound - lowerBound).head<3>().maxCoeff() / gridResolution;
		Eigen::Vector3i oneGridGridSQ;
		oneGridGridSQ << 1, gridResolution, gridResolution*gridResolution;
		//default: search half a cell
		float sqMaxDist = regionConstants.maxDistScaling * regionConstants.maxDistScaling * evCellsize.squaredNorm() * (flags&OctreeFlags::normalsUse?3:2);
		Vec9f normalisationConstScaling = nodeRange.cwiseInverse().cwiseProduct(regionConstants.scaling);
		normalisationConstScaling = normalisationConstScaling.unaryExpr([](float f) {return (std::isnan(f) || std::isinf(f) )? 0 : f; }); //if an entire col is 0 Inverse will result in nan

		//sort all child verts into grid for --hopefully-- reasonable speed
		for (auto vert : data)
		{
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;

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

		//clustering
		while (!vertMap.empty())
		{
			// prob.better to use existing vert as centroid
			//Vec9f centroid = ((Vec9f::Random() + Vec9f::Ones()) / 2.0f).cwiseProduct(invNormalisationConst) - lowerBound;

			UINT32 centroidCellIndex = rand() % vertMap.size();
			std::unordered_map < UINT32, std::vector<Type>*, hashID>::iterator it(vertMap.begin());
			std::advance(it, centroidCellIndex);
			std::vector<Type>& centroidCell = *it->second;

			assert(!centroidCell.empty());
			UINT32 centroidIndex = rand() % centroidCell.size();

			const Type& centVertex = centroidCell[centroidIndex];

			Vec9f centroid;
			centroid << centVertex.pos.x, centVertex.pos.y, centVertex.pos.z, centVertex.normal.x, centVertex.normal.y, centVertex.normal.z,
				centVertex.color.x, centVertex.color.y, centVertex.color.z;

			Vec9f lastCentroid = Vec9f::Ones() * (upperBound.squaredNorm() * 24); // so 1st check is passed ... using FLT_MAX apperently causes some sort of overflow

			MatX9f clusterVerts;

			//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
			for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
			{
				centroidCellIndex = (centroid.head<3>() - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
				int cells = 0;

				std::queue<UINT32> frontier;
				std::unordered_set<UINT32, hashID> exploredNodes;
				exploredNodes.insert(centroidCellIndex);

				frontier.push(centroidCellIndex); 

				while (!frontier.empty())
				{
					++cells; 
					UINT32 currentIDX = frontier.front();
					frontier.pop();

					bool oneInRange = findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
					if (oneInRange) // found at least one vert -> explore neighbours 
					{
						auto hull = getCellHull(gridResolution, currentIDX);
						for (UINT32 idx : hull)
						{
							if (idx < gridResolution*gridResolution*gridResolution && exploredNodes.find(idx) == exploredNodes.end())
							{
								exploredNodes.insert(idx);
								frontier.push(idx);
							}
						}
					}
				}

				lastCentroid = centroid;
				centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
				clusterVerts.resize(0, 9);
			}//END current iteration

			 //last iteration
			centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
			std::queue<UINT32> frontier;
			std::unordered_set<UINT32, hashID> exploredNodes;
			frontier.push(centroidCellIndex);
			exploredNodes.insert(centroidCellIndex);

			while (!frontier.empty())
			{
				UINT32 currentIDX = frontier.front();
				frontier.pop();

				bool oneInRange = findVerticesInCellAndRemove(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
				if (oneInRange) // found at least one vert -> explore neighbours 
				{
					auto hull = getCellHull(gridResolution, currentIDX);
					for (UINT32 idx : hull)
					{
						if (idx < gridResolution*gridResolution*gridResolution && exploredNodes.find(idx) == exploredNodes.end())
						{
							exploredNodes.insert(idx);
							frontier.push(idx);
						}
					}
				}
			}
			pNode->data.push_back(getVertFromCluster(clusterVerts));

		}//vertMap.empty()


		//expandNode
		std::vector<Type> childData[8]; 

		for (int i = 0; i < 8; ++i)
		{
			childData[i] = std::vector<Type>(); 
		}

		Eigen::Vector3f midPoint = nodeRange.head<3>() / 2; 

		for (auto vert : data)
		{
			Eigen::Vector3f pos; 
			pos << vert.pos.x, vert.pos.y, vert.pos.z; 
			pos = pos - lowerBound.head<3>() - nodeRange.head<3>();
			int index = 0; 
			if (pos.x() > 0)
			{
				index |= 1; 
			}
			if (pos.y() > 0)
			{
				index |= 2;
			}
			if (pos.z() > 0)
			{
				index |= 4;
			}

			childData[index].push_back(vert); 

		}


		for (int i = 0; i < 8; ++i)
		{
			pNode->children[i] = new NestedOctreeNode<Type>(); 

			createRegionGrowingTopDown(pNode->children[i], childData[i], depth + 1); 
			childData[i].clear(); 
		}
		/**/

	}

	void createRegionGrowing(NestedOctreeNode<Type>* pNode, size_t depth);

	void createAndPushDown(NestedOctreeNode<Type>* pNode, const std::vector<Type>& data, XMVECTOR gridStart, size_t depth = 0)
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
		std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)> insertMap[8] =
		{
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID)
		};

		UINT32 subGridOverlapps[8] = { 0 };

		std::vector<Type> subGridData[8]; //Stores verts in case the tree needs to be expanded

		XMVECTOR cellsize = XMLoadFloat3(&cellsizeForDepth[depth]);


		for (auto vert : data)
		{
			XMVECTOR relPos = XMLoadFloat3(&vert.pos) - gridStart;

			XMVECTOR cellIndex = relPos / cellsize;


			//point location in the inscribed grid (gridresolution) <<-- make this float safe
			UINT32 gridIndex = static_cast<UINT32>(XMVectorGetX(cellIndex))
				+ gridResolution * static_cast<UINT32>(XMVectorGetY(cellIndex))
				+ gridResolution * gridResolution * static_cast<UINT32>(XMVectorGetZ(cellIndex));

			//0-8 which node this vert will go to if the tree is expanded here
			UINT32 subGridIndex = calculateSubgridIndex(relPos, depth);

			auto result = insertMap[subGridIndex].find(gridIndex);
			if (result != insertMap[subGridIndex].end())
			{
				result->second.push_back(vert);
			}
			else
			{
				std::vector<Type> newVec;
				newVec.push_back(vert);
				insertMap[subGridIndex].insert(std::pair<UINT32, std::vector<Type>>(gridIndex, newVec));
			}
		}

		for (int i = 0; i < 8; ++i)
		{
			for (auto it : insertMap[i])
			{

				for (Type& vert : it.second)
				{
					//Type& vert = it->second;

					subGridData[i].push_back(vert);
				}
				subGridOverlapps[i] += it.second.size() - 1; //first element in grid is ok
			}
			insertMap[i].clear();
		}


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
					pNode->children[i] = new NestedOctreeNode<Type>();
					++numNodes;

					if (subGridData[i].size() > expansionThreshold)	//these nodes migth have to bee expanded even further
					{
						XMFLOAT3 gridStart3f, gridMidpoint3f;

						XMStoreFloat3(&gridMidpoint3f, gridStart + XMLoadFloat3(&range) / (2 << depth));
						XMStoreFloat3(&gridStart3f, gridStart);


						XMVECTOR subridstart = gridStart +
							(XMLoadFloat3(&range) / (2 << depth))*XMVectorSet(i & 0x01 ? 1 : 0, i & 0x02 ? 1 : 0, i & 0x04 ? 1 : 0, 0);

						createAndPushDown(pNode->children[i], subGridData[i], subridstart, depth + 1); //recurse one level 

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

	void createNaiveAverage(NestedOctreeNode<Type>* pNode, const std::vector<Type>& data, XMVECTOR gridStart, size_t depth = 0)
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
		std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)> insertMap[8] =
		{ 
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID),
			std::unordered_map < UINT32, std::vector<Type>, decltype(hash_ID)>(gridResolution*gridResolution / 8, hash_ID)
		};

		UINT32 subGridOverlapps[8] = { 0 };

		std::vector<Type> subGridData[8]; //Stores verts in case the tree needs to be expanded

		XMVECTOR cellsize = XMLoadFloat3(&cellsizeForDepth[depth]); 


		for (auto vert : data)
		{
			XMVECTOR relPos = XMLoadFloat3(&vert.pos) - gridStart;

			XMVECTOR cellIndex =  relPos / cellsize;


			//point location in the inscribed grid (gridresolution) <<-- make this float safe
			UINT32 gridIndex = static_cast<UINT32>(XMVectorGetX(cellIndex))
				+ gridResolution * static_cast<UINT32>(XMVectorGetY(cellIndex))
				+ gridResolution * gridResolution * static_cast<UINT32>(XMVectorGetZ(cellIndex));

			//0-8 which node this vert will go to if the tree is expanded here
			UINT32 subGridIndex = calculateSubgridIndex(relPos, depth);

			auto result = insertMap[subGridIndex].find(gridIndex);
			if ( result != insertMap[subGridIndex].end())
			{
				result->second.push_back(vert);
			}
			else
			{
				std::vector<Type> newVec; 
				newVec.push_back(vert); 
				insertMap[subGridIndex].insert(std::pair<UINT32, std::vector<Type>>(gridIndex, newVec));
			}
		}

		for (int i = 0; i < 8; ++i)
		{

			//XMFLOAT4 newcol = colors[i];
			/**
			newcol.x = (rand() % 255) / 255.0f;
			newcol.y = (rand() % 255) / 255.0f;
			newcol.z = (rand() % 255) / 255.0f;
			*/

			for (auto it : insertMap[i])
			{
				XMVECTOR sumPos = XMVectorSet(0, 0, 0, 0);
				XMVECTOR sumNor = XMVectorSet(0, 0, 0, 0);
				XMVECTOR sumCol = XMVectorSet(0, 0, 0, 0);

				for(Type& vert : it.second)
				{
					//Type& vert = it->second;

					subGridData[i].push_back(vert);

					sumPos += XMLoadFloat3(&vert.pos);
					sumNor += XMLoadFloat3(&vert.normal);
					sumCol += XMLoadFloat4(&vert.color);
				}

				Type avgVert;

				size_t numElements = it.second.size(); 

				XMVECTOR avgpos = sumPos / numElements;

				XMStoreFloat3(&avgVert.pos, avgpos);
				XMStoreFloat3(&avgVert.normal, sumNor / numElements);
				XMStoreFloat4(&avgVert.color, sumCol / numElements);


				//TEST
				//avgVert.color = newcol;
				//END TEST

				pNode->data.push_back(avgVert);

				subGridOverlapps[i] += it.second.size() - 1; //first element in grid is ok
				
			}
			insertMap[i].clear();
		}


		bool expandNode = false; 
		for (int i = 0; i < 8; ++i)
		{
			if (subGridOverlapps[i] > expansionThreshold)
			{
				expandNode = true; 
				break; 
			}
		}

		if (expandNode&&depth!=maxDepth)
		{
			for (int i = 0; i < 8; ++i)// expand node(s) where to many overlapps occured
			{
				if (!subGridData[i].empty())
				{
					pNode->children[i] = new NestedOctreeNode<Type>();
					++numNodes;

					if (subGridData[i].size() > expansionThreshold)	//these nodes migth have to bee expanded even further
					{
						XMFLOAT3 gridStart3f, gridMidpoint3f;

						XMStoreFloat3(&gridMidpoint3f, gridStart + XMLoadFloat3(&range) / (2 << depth));
						XMStoreFloat3(&gridStart3f, gridStart);


						XMVECTOR subridstart = gridStart + 
							(XMLoadFloat3(&range) / (2 << depth))*XMVectorSet(i & 0x01? 1 : 0 , i & 0x02? 1 : 0, i & 0x04? 1 : 0, 0);

						createNaiveAverage(pNode->children[i], subGridData[i], subridstart, depth + 1); //recurse one level 

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

	void createPossionDisk(NestedOctreeNode<Type>* pNode, const std::vector<Type>& data, XMVECTOR gridStart, size_t depth = 0)
	{

		if (depth > reachedDepth)
		{
			reachedDepth = depth;
			XMVECTOR newCellsize = XMLoadFloat3(&range) / ((1 << depth) * gridResolution);	

			cellsizeForDepth.push_back(XMFLOAT3());
			XMStoreFloat3(&cellsizeForDepth[depth], newCellsize);

			cellMidpointForDepth.push_back(XMFLOAT3());

			XMStoreFloat3(&cellMidpointForDepth[depth], XMLoadFloat3(&range) / (2 << depth));
		}

		// gridRes^3 hashmap w/ chaining
		//starting of with one big map to reduce boundry condition check 
		// use id as hash function, since we already compute the key (= grid index) 
		auto hash_ID = [](UINT32 x)->size_t {return x; };
		std::unordered_map<UINT32, Type, decltype(hash_ID)> insertMap(gridResolution*gridResolution, hash_ID);


		std::vector<Type> expansionBuffer[8]; //Stores verts in case the tree needs to be expanded

		XMVECTOR cellsize = XMLoadFloat3(&cellsizeForDepth[depth]);

		
		for (auto vert : data)
		{
			XMVECTOR pos = XMLoadFloat3(&vert.pos); 

			XMVECTOR cellIndex = (pos - gridStart) / cellsize;

			//point location in the inscribed grid (gridresolution) <<-- make this float safe


			UINT32 x = static_cast<UINT32>(XMVectorGetX(cellIndex));
			UINT32 y = static_cast<UINT32>(XMVectorGetY(cellIndex));
			UINT32 z = static_cast<UINT32>(XMVectorGetZ(cellIndex));

			//point location in the inscribed grid (gridresolution)
			UINT32 gridIndex = GridIndex(x, y, z, gridResolution);

			auto END = insertMap.end(); //doesnt change... hopefully


			auto result = insertMap.find(gridIndex);
			if (result != END) // current grid already allocated -> min distance rule broken
			{
				expansionBuffer[calculateSubgridIndex(pos - gridStart, depth)].push_back(vert); 
			}
			else	// current grid is free, check neighbours
			{
				bool canInsert = true; 
				for (int i = 0; i < gridNeighboursAdj.size() && canInsert; ++i)
				{
					INT64 index = gridIndex + gridNeighboursAdj[i]; 

					if (index<0 || index>gridResolution*gridResolution*gridResolution) //out of bounds
					{
						continue; 
					}

					auto neigbour = insertMap.find(static_cast<UINT32>(index));
					if (neigbour != END && !distanceCheck(pos, XMLoadFloat3(&neigbour->second.pos), cellsize))
					{
						canInsert = false; 
					}
				}

				if (canInsert)
				{
					insertMap.insert(std::pair<UINT32, Type>(gridIndex, vert));
					pNode->data.push_back(vert); 
				}
				else
				{
					expansionBuffer[calculateSubgridIndex(pos - gridStart, depth)].push_back(vert);
				}

			}
		}



		bool expandNode = false;
		for (int i = 0; i < 8; ++i)
		{
			if (expansionBuffer[i].size() > expansionThreshold)
			{
				expandNode = true;
				break;
			}
		}

		if (expandNode&&depth != maxDepth)
		{
			for (int i = 0; i < 8; ++i)// expand node(s) where to many overlapps occured
			{
				if (!expansionBuffer[i].empty())
				{
					pNode->children[i] = new NestedOctreeNode<Type>();
					++numNodes;

					if (expansionBuffer[i].size() > expansionThreshold)	//these nodes migth have to bee expanded even further
					{
						XMFLOAT3 gridStart3f, gridMidpoint3f;

						XMStoreFloat3(&gridMidpoint3f, gridStart + XMLoadFloat3(&range) / (2 << depth));
						XMStoreFloat3(&gridStart3f, gridStart);


						XMVECTOR subridstart = gridStart +
							(XMLoadFloat3(&range) / (2 << depth))*XMVectorSet(i & 0x01 ? 1 : 0, i & 0x02 ? 1 : 0, i & 0x04 ? 1 : 0, 0);

						createPossionDisk(pNode->children[i], expansionBuffer[i], subridstart, depth + 1); //recurse one level 

					}
					else   //this will not be expanded -> leaf node -> no need for additional traversal 
					{
						//this node should be empty
						pNode->children[i]->data.insert(pNode->children[i]->data.end(), expansionBuffer[i].begin(), expansionBuffer[i].end());
					}
				}

				expansionBuffer[i].clear();	//release unneeded space asap
										//	delete subGridData[i];
			}
		}
		else
		{
			pNode->data.clear();
			pNode->data = data;

			for (int i = 0; i < 8; ++i)
			{
				expansionBuffer[i].clear();
				//delete subGridData[i];
			}

		}
	}

	inline bool findVerticesInCell(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist, const UINT32 cellidx,
		const std::unordered_map < UINT32, std::vector<Type>*, hashID>& vertMap, MatX9f& clusterVerts)
	{
		bool oneInRange = false; 
		auto result = vertMap.find(cellidx);

		if (result != vertMap.end())
		{
			std::vector<Type>& cellVector = *result->second;
			for (const Type& vert : cellVector)
			{
				//add normals in polar coords	
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;
				float dist = (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm();
				if (dist < sqMaxDist)	//vert is in range
				{
					clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);
					clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();
					oneInRange = true;
				}
			}
		}//result != vertMap.end()
		return oneInRange;
	}

	bool findVerticesInCellAndRemove(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist, const UINT32 cellidx,
		std::unordered_map < UINT32, std::vector<Type>*, hashID>& vertMap, MatX9f& clusterVerts, std::vector<Type>* boundaryVerts = nullptr);

	Type getVertFromCluster(const MatX9f& clusterVerts, UINT32 depth,  const std::vector<Type>* boundaryVerts = nullptr);

};
