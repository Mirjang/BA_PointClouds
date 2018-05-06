#pragma once
#include <iostream>
#include <sstream>

#include "LOD.h"
#include "../global/Octree.h"


class Octree_Naive_Avg :
	public LOD
{
public:
	Octree_Naive_Avg();
	virtual ~Octree_Naive_Avg();
	
	virtual void create(const ID3D11Device* device, vector<Vertex>& vertices) override;
	virtual void recreate(const ID3D11Device* device, vector<Vertex>& vertices) override;
	virtual void draw(const ID3D11DeviceContext* context) override;
	static TwBar* setUpTweakBar();

	

private: 

	void traverseAndAverageOctree(OctreeInternal::OctreeNode<Vertex>* pNode); 

	struct TweakSettings
	{
		//----creation---
		int maxDepth = -1; 
		
		//----rendering---
		int fixedDepth = 0; 
		bool drawFixedDepth = false; 

	};

	static TweakSettings settings; 

	Octree<Vertex>* octree; //fixed size determined via depth in octree

};

