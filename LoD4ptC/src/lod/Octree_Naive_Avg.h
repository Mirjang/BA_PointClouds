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
	
	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void draw(ID3D11DeviceContext* const context) override;
	static TwBar* setUpTweakBar();

	

private: 

	void traverseAndAverageOctree(OctreeInternal::OctreeNode<Vertex>* pNode); 

	struct TweakSettings
	{
		//----creation---
		int maxDepth = 16; 
		
		//----rendering---
		int fixedDepth = 0; 
		bool drawFixedDepth = false; 

	};

	static TweakSettings settings; 

	Octree<Vertex>* octree; //fixed size determined via depth in octree

};

