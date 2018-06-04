#pragma once

#include "LOD.h"

#include "LodUtils.h"

#include <DirectXMath.h>


class Kmeans_ClusterSplats :
	public LOD
{
public:
	Kmeans_ClusterSplats();
	~Kmeans_ClusterSplats();


	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static TwBar* setUpTweakBar();
	virtual void draw(ID3D11DeviceContext* const context) override;

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


	struct DrawConstants
	{
		float slope;
		float heightDiv2DivSlope;
		UINT strides = sizeof(Vertex);
		UINT offset = 0;

	} drawConstants;



};

