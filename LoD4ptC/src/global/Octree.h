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
		OctreeNode<Type>* parent = nullptr; 
		bool marked = false; //mark nodes on traversal for various uses.. counter in case multiple traversals are needed... who cares about performance anyways? 
		/*
		* internal nodes and insertInternal == true:  this is the value stored at the internal node
		* leaf nodes or insertInternal == false: stores various averaging/subsampling info
		* should only be used if the node is already marked
		*/
		Type data;		

		inline virtual bool isInternal() = 0;
	};


	template<class Type>
	struct OctreeInternalNode : OctreeNode<Type>	//internal Node
	{
		//000 = lowX lowY lowZ, 111 = highX highY highZ
		std::vector<OctreeNode<Type>*> children;

		OctreeInternalNode(OctreeNode<Type>* parent)
		{
			this->parent = parent;

			children.resize(8); 

			for (int i = 0; i < 8; ++i)
			{
				children[i] = nullptr; 
			}

		}

		OctreeInternalNode(OctreeNode<Type>* parent, Type d)
		{ 
			this->parent = parent;
			this->data = d; 

			children.resize(8);

			for (int i = 0; i < 8; ++i)
			{
				children[i] = nullptr;
			}
		}
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

		OctreeLeafNode(OctreeNode<Type>* parent)
		{
			this->parent = parent; 
		}

		OctreeLeafNode(OctreeNode<Type>* parent , const Type& d)
		{
			this->parent = parent;
			verts.push_back(d); 
		}

		std::vector<Type> verts; 
	};

}

template<class Type>
class Octree
{
	static_assert(std::is_base_of<Vertex, Type>::value, "Type needs to be derived from vertex");

public:
	//maxdepth = -1 --> keep expanding octree indefinately
	Octree(XMFLOAT3 min3f, XMFLOAT3 max3f, int maxDepth = -1, bool insertInternal = false) : boundsMin(min3f), boundsMax(max3f), maxDepth(maxDepth), insertInternal(insertInternal)
	{
		XMStoreFloat3(&range, XMLoadFloat3(&boundsMax) - XMLoadFloat3(&boundsMin));
	}

	~Octree() { delete root;  }
	void insert(const Type& data) 
	{
		++numNodes; 
		if (!root)
		{
			if (insertInternal)
			{
				root = new OctreeInternal::OctreeInternalNode<Type>(nullptr, data);
			}
			else
			{
				root = new OctreeInternal::OctreeInternalNode<Type>(nullptr);
				insert(root, data);
			}
		}
		else
		{
			insert(root, data);
		}
	}

	OctreeInternal::OctreeNode<Type>* root = nullptr;
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


		int index = (XMVectorGetX(pointInCellIDX)	? 0x00 : 0x01) |	//x
			(XMVectorGetY(pointInCellIDX)	? 0x00 : 0x02) |			//y
			(XMVectorGetZ(pointInCellIDX)	? 0x00 : 0x04); 			//z

		if (pNode->isInternal())
		{
			OctreeInternal::OctreeInternalNode<Type>* intNode = static_cast<OctreeInternal::OctreeInternalNode<Type>*>(pNode);
			
			if (!intNode->children[index]) // location of current point is unallocated, creafe new leaf
			{
				if (insertInternal) //store points at internal nodes (naive subsampling) vs store all pts at leafs (-> generate new pts at internal later) 
				{
					intNode->children[index] = new OctreeInternal::OctreeLeafNode<Type>(pNode, data);
				}
				else 
				{
					intNode->children[index] = new OctreeInternal::OctreeLeafNode<Type>(pNode);
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
				assert(pParent);

				/**
				if (!pParent) // 2nd element: we are at root and root is leaf node
				{
					//shouldnt happen af

				}
				else
				{
				}*/

			
				//current node leaf->internal
				pParent->children[parentIndex] = new OctreeInternal::OctreeInternalNode<Type>(pParent);

				if (insertInternal && leafNode->verts.size())
				{ 
					pParent->children[index]->data = leafNode->verts[0];
				}

				for (int i = insertInternal?1:0; i< leafNode->verts.size(); ++i)
				{
					insert(pParent->children[index], leafNode->verts[i], depth, pParent, parentIndex);
				}

				delete leafNode; 
			}
			else	//max depth -> just add to leaf node
			{
				leafNode->verts.push_back(data); 
			}
		}

	}

	//set at instantiaton
	XMFLOAT3 boundsMin, boundsMax, range;
	int maxDepth;
	bool insertInternal; // store input at internal nodes

};
