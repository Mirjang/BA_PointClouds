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
#include "ClusterMath.h"
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

	std::vector<int> getCellNeighbours(UINT32 res, UINT32 index = 0)
	{
		std::vector<int> hull; 
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
	void createRegionGrowing(float maxFeatureDist = 1.0f, float wPos = 1.0f, float wNor = 0.0f, float wCol = 0.0f, CenteringMode centeringMode = CenteringMode::SPACIAL)
	{
		UINT32 oldRes = gridResolution; //use lower res for more efficient search?
		regionConstants.scaling(0) = wPos;
		regionConstants.scaling(1) = wPos;
		regionConstants.scaling(2) = wPos;
		regionConstants.scaling(3) = wNor;
		regionConstants.scaling(4) = wNor;
		regionConstants.scaling(5) = wNor;
		regionConstants.scaling(6) = wCol;
		regionConstants.scaling(7) = wCol;
		regionConstants.scaling(8) = wCol;

		regionConstants.maxDist = maxFeatureDist;
		regionConstants.maxNorAngle = wNor; 
		regionConstants.maxColDist = wCol; 

		//regionConstants.scaling /= regionConstants.scaling.norm(); //normalized weights
		regionConstants.centeringMode = centeringMode; 
		regionConstants.maxDistScaling = maxFeatureDist;

		createRegionGrowing(root, 0);

		gridResolution = oldRes;
	}

	void createRegionGrowing2(float maxFeatureDist = 1.0f, float wPos = 1.0f, float wNor = 0.0f, float wCol = 0.0f, CenteringMode centeringMode = CenteringMode::AMEAN)
	{
		UINT32 oldRes = gridResolution; //use lower res for more efficient search?
		regionConstants.scaling(0) = wPos;
		regionConstants.scaling(1) = wPos;
		regionConstants.scaling(2) = wPos;
		regionConstants.scaling(3) = wNor;
		regionConstants.scaling(4) = wNor;
		regionConstants.scaling(5) = wNor;
		regionConstants.scaling(6) = wCol;
		regionConstants.scaling(7) = wCol;
		regionConstants.scaling(8) = wCol;

		regionConstants.maxDist = maxFeatureDist;
		regionConstants.maxNorAngle = wNor;
		regionConstants.maxColDist = wCol;

		//regionConstants.scaling /= regionConstants.scaling.norm(); //normalized weights
		regionConstants.centeringMode = centeringMode;
		regionConstants.maxDistScaling = maxFeatureDist;

		createRegionGrowing2(root, 0);

		gridResolution = oldRes;
	}

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
		CenteringMode centeringMode; 
		float maxDist, maxNorAngle, maxColDist; 

	};

	RegionGrowingConstants regionConstants;

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

	bool findVerticesInCell(Cluster<Type>& cluster, const UINT32 cellidx,
		std::unordered_map < UINT32, std::vector<Type>*, hashID>& vertMap)
	{
		size_t clusterSize = cluster.verts.size(); 
		auto result = vertMap.find(cellidx);
		if (result != vertMap.end())
		{
			std::vector<Type>& cellVector = *result->second;
			for (const Type& vert : cellVector)
			{
				cluster.checkAdd(vert); 
			}
		}//result != vertMap.end()
		return cluster.verts.size()!=clusterSize; // a vert has been added
	}

	bool findVerticesInCellAndRemove(Cluster<Type>& cluster, const UINT32 cellidx,
		std::unordered_map < UINT32, std::vector<Type>*, hashID>& vertMap)
	{
		size_t clusterSize = cluster.verts.size();
		auto result = vertMap.find(cellidx);
		if (result != vertMap.end())
		{
			std::vector<Type>* cellVector = result->second;

	//		cluster.debug = cellidx == 73 && cellVector->size()==1;

			for (std::vector<Type>::iterator it = cellVector->begin();it != cellVector->end(); )
			{
				if (cluster.checkAdd(*it))	//vert is in range
				{
					*it = std::move(cellVector->back()); 
					cellVector->pop_back(); 
				}
				else
				{
					++it;
				}
			}
			if (cellVector->empty())
			{
				delete cellVector;
				vertMap.erase(cellidx);
			}
		}//result != vertMap.end()
		return cluster.verts.size() != clusterSize; // a vert has been added
	}


	void createRegionGrowing2(NestedOctreeNode<Type>* pNode, size_t depth)
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
							new std::thread([=] {createRegionGrowing2(pNode->children[i], depth + 1); })//next level  
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
						createRegionGrowing2(pNode->children[i], depth + 1); //next level  
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
		UINT32 searchResolution = static_cast<UINT32>(nodeRange.head<3>().maxCoeff() / (sqrt(maxDistSq)));
		searchResolution = max(1U, searchResolution);

		//UINT32 searchResolution = gridResolution;

		//calulates clusters for current node
		// gridRes^3 hashmap w/ chaining
		// use id as hash function, since we already compute the key (= grid index) 
		std::unordered_map < UINT32, std::vector<Type>*, hashID> vertMap(searchResolution*searchResolution);

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
						std::vector<Type>* newVec = new std::vector<Type>();
						newVec->push_back(vert);
						vertMap.insert(std::pair<UINT32, std::vector<Type>*>(gridIndex, newVec));
					}
				}
			}
		}
		
		//all possible verts that could be added to the current cluster ( dist^2 < maxDistSq ) 
		std::vector<std::pair<float, Type>> candidates;

		//clustering
		UINT32 itcount = 0;
		while (!vertMap.empty())
		{
			++itcount;

			Type centVertex; 
			UINT32 centroidCellIndex; 
			if (candidates.empty()) // start at new seed pt
			{
				centroidCellIndex = rand() % vertMap.size();
				std::unordered_map < UINT32, std::vector<Type>*, hashID>::iterator it(vertMap.begin());
				std::advance(it, centroidCellIndex);
				std::vector<Type>& centroidCell = *it->second;
				centroidCellIndex = it->first;

				//assert(!centroidCell.empty()); 
				UINT32 centroidIndex = rand() % centroidCell.size();
				
				centVertex = centroidCell[centroidIndex];
				centroidCell[centroidIndex] = centroidCell.back();
				centroidCell.pop_back(); 

				if (centroidCell.empty())
				{
					it->second; 
					vertMap.erase(centroidIndex);
				}
			}
			else // continue at border
			{
				centVertex = candidates[candidates.size()-1].second; 
				candidates.pop_back(); 
				centroidCellIndex = (xmfloat2Eigen(centVertex.pos) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

			}

			Cluster2<Type> cluster;
			cluster.initCentroid(centVertex, maxDistSq, maxAngle, maxColSq, regionConstants.centeringMode);

			// recompute distances to current centroid for border of last centroid
			for (auto it = candidates.begin(); it!=candidates.end(); ++it)
			{
				it->first = (xmfloat2Eigen(it->second.pos) - cluster.centroid.head<3>()).squaredNorm();
			}

			//get all verts within maxDistSq; 
			auto hull = getCellNeighbours(searchResolution, centroidCellIndex);
			for (int idx : hull)
			{
				if (idx >= 0 && idx < searchResolution*searchResolution*searchResolution)
				{
					auto result = vertMap.find(idx);
					if (result != vertMap.end())
					{
						std::vector<Type>* cellVector = result->second;

						for (size_t it = 0;it != cellVector->size(); )
						{
							std::pair<float, Type> candidate; 
							candidate.first = (xmfloat2Eigen((*cellVector)[it].pos) - cluster.centroid.head<3>()).squaredNorm();

							if (candidate.first < maxDistSq)	//vert is in range
							{
								candidate.second = (*cellVector)[it]; 
								candidates.push_back(candidate);

								(*cellVector)[it] = cellVector->back();
								cellVector->pop_back(); // size--
							}
							else
							{
								++it;
							}
						}
						if (cellVector->empty())
						{
							delete cellVector;
							vertMap.erase(idx);
						}
					}//result != vertMap.end()
				}
			}
			struct pairComparator
			{
				bool operator() (const std::pair<float, Type> & left, const std::pair<float, Type> & right)
				{
					return left.first < right.first;
				}
				bool operator() (const std::pair<float, Type> & left, float right)
				{
					return left.first < right;
				}
				bool operator() (float left, const std::pair<float, Type> & right)
				{
					return left < right.first;
				}
			}comp;
			//sort candidates by dist to vert
			std::sort(candidates.begin(), candidates.end(), [](auto &left, auto &right) {
				return left.first < right.first;
			});

			
			auto lb = std::upper_bound(candidates.begin(), candidates.end(), maxDistSq*1.1f, comp); 
			auto it = lb; 
			for (; it != candidates.end();++it) // rehash rest verts
			{
				auto vert = it->second; 
				
				Eigen::Vector3f pos;
				pos << vert.pos.x, vert.pos.y, vert.pos.z;
				//point location in the inscribed grid (searchResolution) <<-- make this float safe
				UINT32 gridIndex = (pos - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second->push_back(vert);
				}
				else
				{
					std::vector<Type>* newVec = new std::vector<Type>();
					newVec->push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<Type>*>(gridIndex, newVec));
				}
				
			}
			std::vector<std::pair<float, Type>>(candidates.begin(), lb).swap(candidates); // cuts all Vertsfurther away than maxDistSq*1.1f // doesnt happan on first it


			addVertsToCluster(&candidates, cluster); 
			cluster.center();

			Type sv = cluster.getSplat();
			pNode->data.push_back(sv);
		}//vertMap.empty()

		 //	std::cout << itcount << std::endl; 

		if (g_lodSettings.useThreads) //  MT
		{
			runNodeCalculations.release();
		}
	}

	void addVertsToCluster(std::vector<std::pair<float, Type>>* candidates, Cluster2<Type>& cluster);

};
