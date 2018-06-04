#pragma once
#include "LOD.h"
#include "../datastructures/NestedOctree.h"

#include "LodUtils.h"
#include <DirectXMath.h>

struct char2
{
	char r, g;
	char2() {}
	char2(char r, char g) : r(r), g(g) {}

};

class Nested_Octree_PossionDisk :
	public LOD
{
public:
	Nested_Octree_PossionDisk();
	virtual ~Nested_Octree_PossionDisk();

	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static TwBar* setUpTweakBar();
	virtual void draw(ID3D11DeviceContext* const context) override;

private: 

	void traverseTreeAndMarkVisibleNodes(XMVECTOR& center, const XMVECTOR& cameraPos);
	std::vector<LOD_Utils::VertexBuffer> visibleNodes; 
	std::vector<char2> visibleNodesBlob; 
	ID3D11Texture1D* visibleNodesTexture;


	void drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth);


	void drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth);

	struct TweakSettings
	{
		
		//----creation---
		UINT32 gridResolution = 128;
		/*
		* expand node after this many vertices have allocated a duplicate position
		* a flat surface in a 128^3 grid would have roughly 16k (128^2) verts)
		*/
		UINT32 expansionThreshold = 2048;
		//maximum depth for octree 
		UINT32 maxDepth = 16;

		UINT64 distanceFunction = OctreeFlags::dfEuclididan; 

		//----rendering---
		int fixedDepth = 0;
		bool drawFixedDepth = false;


		//info
		int LOD = 0;
		int nodesDrawn = 0;

	};
	static TweakSettings settings;

	std::vector<OctreeVectorNode<LOD_Utils::VertexBuffer>> vertexBuffers;


	ID3D11Device* device = nullptr;  //ok, not how i planned my architecture, but ....

	NestedOctree<Vertex>* octree; //fixed size determined via depth in octree

	struct DrawConstants
	{
		float slope;
		float heightDiv2DivSlope;
		UINT strides = sizeof(Vertex);
		UINT offset = 0;

	} drawConstants;


};

