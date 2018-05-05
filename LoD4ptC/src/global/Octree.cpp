#include "Octree.h"

template<class Type>
void Octree::insert(OctreeNode<Type>* node, const Type& data, int depth)
{
	int posX = nodeX / rangeX; 
	int posY = nodeY / rangeY;
	int posZ = nodeZ / rangeZ;


	int index = (node.x < rangeX / depth) ? 1 : 0
		| (node.y < rangeY / depth) ? 2 : 0
		| (node.z < rangeZ / depth) ? 4 : 0;
}
