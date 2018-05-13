#pragma once
#include "LOD.h"
#include "../datastructures/NestedOctree.h"

class Nested_Octree_Naive_Avg : public LOD
{
public:
	Nested_Octree_Naive_Avg();
	virtual ~Nested_Octree_Naive_Avg();

	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static TwBar* setUpTweakBar();
	virtual void draw(ID3D11DeviceContext* const context) override;

	inline void traverseAndAverageOctree(NestedOctreeNode<Vertex>* pNode);

private: 
	struct TweakSettings
	{
		//----creation---
		UINT32 gridResolution = 128; 
		//----rendering---
		int fixedDepth = 0;
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


	NestedOctree<Vertex>* octree; //fixed size determined via depth in octree



};

