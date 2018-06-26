#pragma once

#include <Eigen/dense>
#include <DirectXMath.h>



#include "LOD.h"
#include "LodUtils.h"
#include "../datastructures/NestedOctree.h"
#include "../global/utils.h"


class Regions_Ellipses :
	public LOD
{
public:
	Regions_Ellipses();
	virtual ~Regions_Ellipses();

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

		float maxFeatureDist = 1.0f;

		float weightPos = 1.0f;
		float weightNormal = 1.0f;
		float weightColor = 1.0f;

		float clusterSplatScale; 
		UINT32 maxCentroidsPerNode = 7500;
		UINT32 iterations = 8;
		bool simpleDistance = true;

		/*
		* expand node after this many vertices have allocated a duplicate position
		* a flat surface in a 128^3 grid would have roughly 16k (128^2) verts)
		*/
		UINT32 expansionThreshold = 2500;
		//maximum depth for octree 
		UINT32 maxDepth = 16;

		//UINT64 distanceFunction = OctreeFlags::dfEuclididan;

		//----rendering---
		int fixedDepth = 0;
		bool drawFixedDepth = false;


		//info
		int LOD = 0;
		int nodesDrawn = 0;

	};
	static TweakSettings settings;

	std::vector<OctreeVectorNode<LOD_Utils::EllipticalVertexBuffer>> vertexBuffers;

	NestedOctree<EllipticalVertex>* octree;

	struct DrawConstants
	{
		float slope;
		float pixelSizeConstant;
		UINT strides = sizeof(EllipticalVertex);
		UINT offset = 0;

	} drawConstants;
};

