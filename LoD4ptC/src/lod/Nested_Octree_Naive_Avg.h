#pragma once
#include "LOD.h"
#include "../datastructures/NestedOctree.h"

#include "LodUtils.h"

class Nested_Octree_Naive_Avg : public LOD
{
public:
	Nested_Octree_Naive_Avg();
	virtual ~Nested_Octree_Naive_Avg();

	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static TwBar* setUpTweakBar();
	virtual void draw(ID3D11DeviceContext* const context) override;

	//unused -> maybe useful for actual sampling techniques
	inline void traverseAndUpsampleOctree(NestedOctreeNode<Vertex>* pNode);

private: 


	void drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIntex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth);

	void drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth); 

	struct TweakSettings
	{
		//----creation---
		UINT32 gridResolution = 128; 
		/*
		* expand node after this many vertices have allocated a duplicate position  
		* a flat surface in a 128^3 grid would have roughly 16k (128^2) verts) 
		*/
		UINT32 expansionThreshold = 12000; 

		/*
		* number of nodes that will be averaged/upsampled into one higher-level node
		*/
		UINT32 upsampleRate = 4; 

		//----rendering---
		int fixedDepth = 0;
		bool drawFixedDepth = false;


		//info
		int LOD = 0; 
		int nodesDrawn = 0; 

	};
	static TweakSettings settings;


	__declspec(align(16))
	struct cbLODPerFrame
	{
		UINT32 fixedLODdepth;
	};
	cbLODPerFrame cbPerFrame;

	ID3D11Buffer* cbPerFrameBuffer = nullptr;

	std::vector<OctreeVectorNode<LOD_Utils::VertexBuffer>> vertexBuffers; 

	NestedOctree<Vertex>* octree; //fixed size determined via depth in octree

	struct DrawConstants
	{
		float slope; 
		float heightDiv2DivSlope; 
		UINT strides = sizeof(Vertex); 
		UINT offset = 0;
		
	} drawConstants;


};

