#pragma once

#include <DirectXMath.h>
#include <cmath>
#include <cassert>
#include <vector>

#include "../rendering/Vertex.h"

using namespace DirectX;

namespace OctreeInternal
{
	template<class Type>
	struct OctreeNode 
	{
		inline virtual bool isInternal() = 0;
	};


	template<class Type>
	struct OctreeInternalNode : OctreeNode<Type>	//internal Node
	{
		//000 = lowX lowY lowZ, 111 = highX highY highZ
		std::vector<OctreeNode<Type>*> children;
		Type data;

		OctreeInternalNode() { 
			children.resize(8); 

			for (int i = 0; i < 8; ++i)
			{
				children[i] = nullptr; 
			}

		}
		OctreeInternalNode(Type d) : data(d) { OctreeInternalNode();  }
		~OctreeInternalNode()
		{
			for (auto it : children)
			{
				delete it; 
			}
			children.clear(); 
		}

		bool isInternal() { return true; }

	};

	template<class Type>
	struct OctreeLeafNode : OctreeNode<Type>	//leaf Node
	{
		bool isInternal() { return false; }

		OctreeLeafNode() {}

		OctreeLeafNode(const Type& d)
		{
			data.push_back(d); 
		}

		std::vector<Type> data; 
	};

}

template<class Type>
class Octree
{
	static_assert(std::is_base_of<Vertex, Type>::value, "Type needs to be derived from vertex");

public:
	//maxdepth = -1 --> keep expanding octree indefinately
	Octree(XMFLOAT3 boundsMin, XMFLOAT3 boundsMax, int maxDepth = -1, bool insertInternal = false) : boundsMin(boundsMin), boundsMin(boundsMax), maxDepth(maxDepth)
	{
		XMStoreFloat3(&range, XMLoadFloat3(&boundsMax) - XMLoadFloat3(&boundsMin));
	}

	~Octree() { delete root;  }
	void insert(const Type& data) 
	{
		++numNodes; 
		if (!root)
		{
			root = new OctreeInternal::OctreeInternalNode<Type>(data);
		}
		else
		{
			insert(root, data);
		}
	}

	OctreeInternal::OctreeInternalNode<Type>* root = nullptr;
	int numNodes = 0; 
	int reachedDepth = 0; 


private: 

	void insert(OctreeInternal::OctreeNode<Type>* pNode, const Type& data, int depth = 0, OctreeInternal::OctreeInternalNode<Type>* pParent = nullptr, int parentIndex = 0)
	{
		reachedDepth = max(depth, reachedDepth); 
		size_t cells = 1 << depth;	//number of cells per axis on current level

		XMVECTOR cellsize = XMLoadFloat3(&range) / cells;

		XMVECTOR pos = XMLoadFloat3(&data.pos);
		XMVECTOR relPos = pos - XMLoadFloat3(&boundsMin);
	
		XMVECTOR pointInCellPos = pos - XMVectorFloor(relPos / cellsize) * cellsize;

		XMVECTOR pointInCellIDX = XMVectorLess(pointInCellPos, cellsize / 2); 	

		int index = pointInCellIDX.m128_i32[0] ? 0 : 1 |	//x
			pointInCellIDX.m128_i32[1] ? 0 : 2 |			//y
			pointInCellIDX.m128_i32[2] ? 0 : 4; 			//z

		if (pNode->isInternal())
		{
			OctreeInternal::OctreeInternalNode<Type>* intNode = static_cast<OctreeInternal::OctreeInternalNode<Type>*>(pNode);
			
			if (!intNode->children[index]) // location of current point is unallocated
			{
				if (insertInternal && depth != maxDepth) //store points at internal nodes (naive subsampling) and maxDepth not reached
				{
					intNode->children[index] = new OctreeInternal::OctreeInternalNode(data);
				}
				else //max depth reached -> store all following pts in single leaf node
				{
					intNode->children[index] = new OctreeInternal::OctreeLeafNode(data);
				}
			}
			else	//go one step deeper
			{
				insert(intNode->children[index], data, depth+1, intNode, index);
			}
		}
		else	// we are at a leaf node
		{
			OctreeInternal::OctreeLeafNode<Type>* leafNode = static_cast<OctreeInternal::OctreeLeafNode<Type>*>(pNode);

			if (depth != maxDepth) //turn leaf node into internal node 
			{
				if (insertInternal)
					pParent->children[parentIndex] = new OctreeInternal::OctreeInternalNode(leafNode->data.pop_back); 
				else
					pParent->children[parentIndex] = new OctreeInternal::OctreeInternalNode();

				for (auto elem : leafNode->data)
				{
					insert(pParent->children[parentIndex], elem, depth, pParent, parentIndex);
				}

				delete leafNode; 
			}
			else	//max depth -> just add to leaf node
			{
				leafNode->data.push_back(data); 
			}
		}

	}

	//set at instantiaton
	XMFLOAT3 boundsMin, boundsMax, range;
	int maxDepth;
	bool insertInternal; // store input at internal nodes

};
