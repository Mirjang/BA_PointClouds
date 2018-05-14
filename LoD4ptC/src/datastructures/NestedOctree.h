#pragma once
#pragma once

#include <DirectXMath.h>
#include <cmath>
#include <cassert>
#include <vector>
#include <unordered_map>
#include <basetsd.h>

#include "../rendering/Vertex.h"





//flags indicating if child is at _POS 0 = left/down/front

#define X0Y0Z0 0b00000001
#define X1Y0Z0 0b00000010
#define X0Y1Z0 0b00000100
#define X1Y1Z0 0b00001000
#define X0Y0Z1 0b00010000
#define X1Y0Z1 0b00100000
#define X0Y1Z1 0b01000000
#define X1Y1Z1 0b10000000

#define ChildAt_X0Y0Z0(p) p&X0Y0Z0
#define ChildAt_X1Y0Z0(p) p&X1Y0Z0
#define ChildAt_X0Y1Z0(p) p&X0Y1Z0
#define ChildAt_X1Y1Z0(p) p&X1Y1Z0
#define ChildAt_X0Y0Z1(p) p&X0Y0Z1
#define ChildAt_X1Y0Z1(p) p&X1Y0Z1
#define ChildAt_X0Y1Z1(p) p&X0Y1Z1
#define ChildAt_X1Y1Z1(p) p&X1Y1Z1


using namespace DirectX; 

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


	//000 = left/down/front
	NestedOctreeNode<Type>* children[8];
	std::vector<Type> data; //data tbd via subsampling/averaging/pca etc.. or actual input (at leaves)--- (Mis)used as buffer during creation !



	std::unordered_map<UINT32,Type>  insertData; // used to store data in grid and determine when to subdivide

};


template<class Type>
class NestedOctree
{
	static_assert(std::is_base_of<Vertex, Type>::value, "Type needs to be derived from vertex");

public:

	NestedOctree(const std::vector<Type>& data, UINT32 gridResolution, UINT32 expansionThreshold) : gridResolution(gridResolution),expansionThreshold(expansionThreshold)
	{
		if (!data.size()) return; //empty array

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

		XMVECTOR vrange = vmax - vmin;

		XMStoreFloat3(&range, vrange); 
		XMStoreFloat3(&boundsMin, vmin);
		XMStoreFloat3(&boundsMin, vmax);

		for (auto it : data)
		{
			insert(root, it, vmin); 
		}

	}

	~NestedOctree() { delete root; }



	NestedOctreeNode<Type>* root = nullptr;
	size_t numNodes = 0;
	size_t reachedDepth = 0;
	UINT32 gridResolution; //res of the inscribed grid (lower res->deeper tree)
	UINT32 expansionThreshold; 

	UINT32 calculateSubgridIndex(XMVECTOR relPos, XMVECTOR gridStart, size_t depth)
	{
		XMVECTOR gridMidpoint = gridStart + XMLoadFloat3(&range) / (2 << depth);

		XMVECTOR vsubgridIndex = XMVectorLess(relPos, gridMidpoint / 2);

		return (XMVectorGetX(vsubgridIndex) ? 0x00 : 0x01) |	//x
			(XMVectorGetY(vsubgridIndex) ? 0x00 : 0x02) |			//y
			(XMVectorGetZ(vsubgridIndex) ? 0x00 : 0x04); 			//z

	}


private:


	void insert(NestedOctreeNode<Type>* pNode, const Type& data, XMVECTOR gridStart, size_t depth = 0)
	{
		reachedDepth = max(depth, reachedDepth);


		XMVECTOR gridMidpoint = gridStart + XMLoadFloat3(&range) / (2 << depth);

		XMVECTOR relPos = XMLoadFloat3(&data.pos) - gridStart;

		
		int subGridIndex = calculateSubgridIndex(relPos, gridStart, depth); 
		
		//grid has already been expanded -> go lower  (no insert on internal nodes)
		if (pNode->children[subGridIndex])	
		{
			XMFLOAT3 gridStart3f, gridMidpoint3f; // couldnt find a way to do this on simd -> divide gridIndex by 0xffffffff ?  
			XMStoreFloat3(&gridStart3f, gridStart); 
			XMStoreFloat3(&gridMidpoint3f, gridMidpoint); 

			gridStart = XMVectorSet(
				subGridIndex & 0x01 ? gridMidpoint3f.x : gridStart3f.x,
				subGridIndex & 0x02 ? gridMidpoint3f.y : gridStart3f.y,
				subGridIndex & 0x04 ? gridMidpoint3f.z : gridStart3f.z, 0);
			insert(pNode->children[subGridIndex], data, gridStart , depth + 1);
		}
		else  //there is no point at the location -> either grid has already been expanded or first entry
		{
			XMVECTOR cellsize = (XMLoadFloat3(&range) / depth) / gridResolution;

			XMVECTOR cellIndex = relPos / cellsize;

			//point location in the inscribed grid (gridresolution)
			UINT32 index = static_cast<UINT32>(XMVectorGetX(cellIndex))
				+ static_cast<UINT32>(XMVectorGetY(cellIndex)) * gridResolution
				+ static_cast<UINT32>(XMVectorGetY(cellIndex)) * gridResolution * gridResolution;

			//there is already a point at the given gridpos --> expand and go lower
			auto result = pNode->insertData.find(index);
			if (result != pNode->insertData.end())	
			{
				pNode->data.push_back(data); //(Mis)use as buffer during creation

				if (pNode->data.size() > expansionThreshold)
				{

					size_t depthPlus1 = depth + 1; //dont wana have to calc this every time and still need original depth... 

					XMFLOAT3 gridStart3f, gridMidpoint3f; // couldnt find a way to do this on simd -> divide gridIndex by 0xffffffff to get indicator vector?  
					XMStoreFloat3(&gridStart3f, gridStart);
					XMStoreFloat3(&gridMidpoint3f, gridMidpoint);

					XMVECTOR subGridStart[8];
					//				XMVECTOR subGridMax[8];

					for (int i = 0; i < 8; ++i)
					{
						pNode->children[i] = new NestedOctreeNode<Type>();


						subGridStart[i] = XMVectorSet(
							i & 0x01 ? gridMidpoint3f.x : gridStart3f.x,
							i & 0x02 ? gridMidpoint3f.y : gridStart3f.y,
							i & 0x04 ? gridMidpoint3f.z : gridStart3f.z, 0);

						//					subGridMax[i]= subGridStart + XMLoadFloat3(&range) / (2 << depth);


					}

					numNodes += 8;
					insert(pNode->children[subGridIndex], data, subGridStart[subGridIndex], depthPlus1); //insert current data


					////remove all data from expanding grid and insert in next level node
					for (auto it : pNode->insertData)
					{
						int itIndex = calculateSubgridIndex(XMLoadFloat3(&it.second.pos), gridStart, depth);
						insert(pNode->children[itIndex], it.second, subGridStart[itIndex], depthPlus1); //insert current data

					}

					pNode->insertData.clear();

					for (auto vert : pNode->data)
					{
						int itIndex = calculateSubgridIndex(XMLoadFloat3(&vert.pos), gridStart, depth);
						insert(pNode->children[itIndex], vert, subGridStart[itIndex], depthPlus1); //insert current data
					}
					pNode->data.clear(); 
				}

			}
			// first point at that position
			else 
			{
				pNode->insertData.insert(std::pair<const UINT32, Type>(index, data));
			}
		}

	}

	XMFLOAT3 boundsMin, boundsMax, range;

};
