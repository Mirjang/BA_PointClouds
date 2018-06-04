#include "Kmeans_ClusterSplats.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>

#include "../rendering/Effects.h"

using namespace DirectX; 


Kmeans_ClusterSplats::Kmeans_ClusterSplats()
{
}


Kmeans_ClusterSplats::~Kmeans_ClusterSplats()
{
}



Kmeans_ClusterSplats::TweakSettings Kmeans_ClusterSplats::settings;
TwBar* Kmeans_ClusterSplats::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("Octree Possion Disk");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Kmeans_ClusterSplats::settings.gridResolution, NULL);
	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Kmeans_ClusterSplats::settings.expansionThreshold, NULL);
	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Kmeans_ClusterSplats::settings.maxDepth, NULL);

	TwEnumVal distanceFunctionEV[] = { { OctreeFlags::dfEuclididan, "Euclidian distance" },{ OctreeFlags::dfManhattan, "Manhattan distance" } };
	TwType twDistanceFunction = TwDefineEnum("Distance Function", distanceFunctionEV, ARRAYSIZE(distanceFunctionEV));
	TwAddVarRW(tweakBar, "Distance Function", twDistanceFunction, &settings.distanceFunction, NULL);


	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Kmeans_ClusterSplats::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Kmeans_ClusterSplats::settings.drawFixedDepth, NULL);

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Kmeans_ClusterSplats::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Kmeans_ClusterSplats::settings.nodesDrawn, NULL);


	return tweakBar;

}

// pray for -O3
void Kmeans_ClusterSplats::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT hr;

	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating Octree_PossionDisk" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();

	


	//create here





	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
//	std::cout << "Created Octree w/ Possion Disk Sampling with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes << std::endl;
	std::cout << "Took: " << elapsed.count() << "ms" << std::endl;



	//init GPU stuff

	//load to GPU

	std::cout << "uploading relevant octree data to gpu" << std::endl;



	//Effects::cbShaderSettings.maxLOD = octree->reachedDepth;
	//g_statistics.maxDepth = octree->reachedDepth;
	std::cout << "=======================DONE========================\n" << std::endl;

}

void Kmeans_ClusterSplats::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();

	create(device, vertices);
}

void Kmeans_ClusterSplats::draw(ID3D11DeviceContext* const context)
{

	settings.nodesDrawn = 0;
	settings.LOD = 0;

	Effects::g_pCurrentPass->apply(context);



	drawConstants.slope = tan(g_screenParams.fov / 2);
	drawConstants.heightDiv2DivSlope = g_screenParams.height / (2.0f * drawConstants.slope);

	if (settings.drawFixedDepth)
	{
		drawRecursiveFixedDepth(context, 0, 0);
	}
	else
	{
		drawRecursive(context, 0, XMLoadFloat4(&Effects::cbPerObj.camPos), 0);
	}


}

void Kmeans_ClusterSplats::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, const XMVECTOR& cameraPos, int depth)
{
	settings.LOD = max(settings.LOD, depth);


	//thats how you turn an AABB into BS

	XMMATRIX worldMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat);
	XMVECTOR octreeCenterWorldpos = XMVector3Transform(center, worldMat);

	XMFLOAT3& cellsize3f = octree->cellsizeForDepth[depth];
	XMVECTOR cellsize = XMLoadFloat3(&cellsize3f);

	XMVECTOR distance = octreeCenterWorldpos - cameraPos;


	/*
	if (XMVector3Dot(distance, XMLoadFloat4(&Effects::cbPerObj.camDir)).m128_f32[0] < worldradius) //z coord is behind camera
	{
	return;
	}
	*/

	float worldradius = g_renderSettings.splatSize * (1 << octree->reachedDepth - depth);

	float pixelsize = (worldradius* drawConstants.heightDiv2DivSlope) / XMVector3Length(distance).m128_f32[0];

	settings.nodesDrawn++;

	Effects::SetSplatSize(worldradius);
	Effects::cbPerLOD.currentLOD = depth;
	Effects::UpdatePerLODBuffer(context);

	LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
	context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
	context->Draw(vb.size, 0);
	g_statistics.verticesDrawn += vb.size;

	if (pixelsize > g_lodSettings.pixelThreshhold && vertexBuffers[nodeIndex].children)
	{
		XMVECTOR nextLevelCenterOffset = XMLoadFloat3(&octree->cellsizeForDepth[depth + 1]) / 2;
		UINT8 numchildren = 0;

		for (int i = 0; i < 8; ++i)
		{
			if (vertexBuffers[nodeIndex].children & (0x01 << i))	//ist ith flag set T->the child exists
			{
				XMVECTOR offset = nextLevelCenterOffset * LOD_Utils::signVector(i);
				drawRecursive(context, vertexBuffers[nodeIndex].firstChildIndex + numchildren, center + offset, cameraPos, depth + 1);

				++numchildren;
			}
		}

	}

}