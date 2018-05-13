#pragma once
#include <iostream>
#include <sstream>

#include "LOD.h"
#include "../datastructures/Octree.h"


/*
*	THIS HAS BEEN PUT ON ICE FOR NOW
*	...since I apparently have no clue what I´m doing
*	...goto Nested_Octree....
*/

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

	inline void traverseAndAverageOctree(OctreeInternal::OctreeNode<Vertex>* pNode); 

	struct TweakSettings
	{
		//----creation---
		int maxDepth = 16; 
		
		//----rendering---
		int fixedDrawDepth = 0; 
		bool drawFixedDepth = false; 

	};
	static TweakSettings settings;


	__declspec(align(16))
	struct cbLODPerFrame
	{

		UINT32 fixedLODdepth;
	};
	cbLODPerFrame cbPerFrame; 

	ID3D11Buffer* cbPerFrameBuffer = nullptr; 


	ID3D11Buffer* vertexBuffer = nullptr;
	UINT strides;


	Octree<Vertex>* octree; //fixed size determined via depth in octree

};

