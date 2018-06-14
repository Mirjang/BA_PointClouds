#pragma once
#include <Eigen/dense>
#include <DirectXMath.h>

#include "LOD.h"
#include "LodUtils.h"
#include "../datastructures/NestedOctree.h"

#include "../datastructures/Kmeans.h"


class Kmeans_Spheres :
	public LOD
{
public:
	Kmeans_Spheres();
	~Kmeans_Spheres();


	virtual void create(ID3D11Device* const device, vector<Vertex>& vertices) override;
	virtual void recreate(ID3D11Device* const device, vector<Vertex>& vertices) override;
	static TwBar* setUpTweakBar();
	virtual void draw(ID3D11DeviceContext* const context) override;

private:

	inline void runKMEANS(std::vector<Vec9f>& verts, std::vector<SphereVertex>& outVec);

	inline void centroidsToSphereSplats(const std::vector<Centroid>& centroids, std::vector<Vec9f>&verts, const std::vector<UINT32>& vertCentroidTable, std::vector<SphereVertex>& outVerts);

	void drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIntex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth);

	void drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth);

	struct TweakSettings
	{
		
		//----creation---
		UINT32 gridResolution = 128;

		UINT32 iterations = 10; 
		UINT32 upsampleRate = 8; 
		UINT32 maxCentroidsPerNode = 7500; 

		bool simpleDistance = true; 

		/*
		* expand node after this many vertices have allocated a duplicate position
		* a flat surface in a 128^3 grid would have roughly 16k (128^2) verts)
		*/
		UINT32 expansionThreshold = 5000;
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

	NestedOctree<SphereVertex>* octree; 

	struct DrawConstants
	{
		float slope;
		float pixelSizeConstant;
		UINT strides = sizeof(SphereVertex);
		UINT offset = 0;

	} drawConstants;


};

