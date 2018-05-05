#pragma once

#include "../rendering/Vertex.h"

template<class Type>
struct OctreeNode	//internal Rep
{
	//000 = lowX lowY lowZ, 111 = highX highY highZ
	OctreeNode<Type> children[2][2][2] = { 0 };
	Type data;

	OctreeNode(){}
	~OctreeNode
	{
		delete[] children; 
	}

};

template<class Type>
class Octree
{
	static_assert(std::is_base_of<Vertex, typename Type::value_type>::value, "Type needs to be derived from vertex");

public:

	Octree(float rangeX, float rangeY, float rangeZ) : rangeX(rangeX), rangeY(rangeY), rangeZ(rangeZ)
	{

	}
	~Octree() { delete root;  }
	void insert(const Type& data)
	{
		insert(root, data); 
	}


	OctreeNode<Type>* root = nullptr;
	int numNodes = 0; 
	int depth = 0; 

private: 
	void insert(OctreeNode<Type>* node, const Type& data, int depth = 0); 

	float rangeX, rangeY, rangeZ;


};

