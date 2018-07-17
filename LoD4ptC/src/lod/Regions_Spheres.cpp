#include "Regions_Spheres.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include <cfloat>
#include <stack>

#include <Eigen/Eigenvalues>

#include "../rendering/Effects.h"

using namespace DirectX;


Regions_Spheres::Regions_Spheres()
{
}


Regions_Spheres::~Regions_Spheres()
{
	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer); 
	}
	delete octree;
}



Regions_Spheres::TweakSettings Regions_Spheres::settings;
TwBar* Regions_Spheres::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("Regions Spheres");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Regions_Spheres::settings.gridResolution, NULL);
//	TwAddVarRW(tweakBar, "Wpos", TW_TYPE_FLOAT, &Regions_Spheres::settings.weightPos, "min=0.001 step=0.0005");
	TwAddVarRW(tweakBar, "max Normal Angle(deg)", TW_TYPE_FLOAT, &Regions_Spheres::settings.weightNormal, "min=0.00 max=361 step=0.05");
	TwAddVarRW(tweakBar, "max Col Dist", TW_TYPE_FLOAT, &Regions_Spheres::settings.weightColor, "min=0.0001 step=0.0005");

	TwAddVarRW(tweakBar, "Max Feature Dist", TW_TYPE_FLOAT, &Regions_Spheres::settings.maxFeatureDist, "min=0.0001 step=0.05");
	TwEnumVal centeringEV[] = {
		{ CenteringMode::KEEP_SEED, "None" },
	{ CenteringMode::AMEAN, "amean" },
	{ CenteringMode::SPACIAL, "Spacial" },
	{ CenteringMode::SPACIAL_POS_REST_AMEAN, "pos Spacial, Rest amean" },	
	};
	TwType twCenteringMode = TwDefineEnum("Center", centeringEV, ARRAYSIZE(centeringEV));
	TwAddVarRW(tweakBar, "Center", twCenteringMode, &Regions_Spheres::settings.centeringMode, NULL);

	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Regions_Spheres::settings.expansionThreshold, NULL);

	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Regions_Spheres::settings.maxDepth, NULL);

	//TwEnumVal distanceFunctionEV[] = { { OctreeFlags::dfEuclididan, "Euclidian distance" },{ OctreeFlags::dfManhattan, "Manhattan distance" } };
	//TwType twDistanceFunction = TwDefineEnum("Distance Function", distanceFunctionEV, ARRAYSIZE(distanceFunctionEV));
	//	TwAddVarRW(tweakBar, "Distance Function", twDistanceFunction, &settings.distanceFunction, NULL);

	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Regions_Spheres::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Regions_Spheres::settings.drawFixedDepth, NULL);
	TwAddVarRW(tweakBar, "Cluster Scale", TW_TYPE_FLOAT, &Regions_Spheres::settings.clusterSplatScale, "min = 0.00 step = 0.005");

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Regions_Spheres::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Regions_Spheres::settings.nodesDrawn, NULL);
	return tweakBar;
}

// pray for -O3
void Regions_Spheres::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating regions_spheres" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();

	//create here

	std::vector<SphereVertex> initalEllpticalVerts;	//at max LOD verts = circles (ellipsoid w/ major/minor  unit length and orthogonal to normal 

	initalEllpticalVerts.reserve(vertices.size());

	for (auto vert : vertices)
	{
		initalEllpticalVerts.push_back(SphereVertex(vert));
	}

	octree = new NestedOctree<SphereVertex>(initalEllpticalVerts, settings.gridResolution, settings.expansionThreshold, settings.maxDepth, OctreeCreationMode::CreateAndPushDown, OctreeFlags::createCube | OctreeFlags::neighbourhoodFull);	
	initalEllpticalVerts.clear();

	octree->createRegionGrowing(settings.maxFeatureDist, settings.weightPos, XMConvertToRadians(settings.weightNormal), settings.weightColor, settings.centeringMode);

	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	std::cout << "Created Octree w/ Regions spheres with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes << std::endl;
	std::cout << "Took: " << elapsed.count() << "s" << std::endl;

	//init GPU stuff

	//load to GPU

	std::cout << "uploading relevant octree data to gpu" << std::endl;

	octree->getStructureAsVector<LOD_Utils::EllipticalVertexBuffer, ID3D11Device*>(vertexBuffers, &LOD_Utils::createSphereVertexBufferFromNode, device);

	Effects::cbShaderSettings.maxLOD = octree->reachedDepth;
	g_statistics.maxDepth = octree->reachedDepth;
	std::cout << "=======================DONE========================\n" << std::endl;
}


void Regions_Spheres::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();
	delete octree;

	create(device, vertices);
}

void Regions_Spheres::draw(ID3D11DeviceContext* const context)
{
	settings.nodesDrawn = 0;
	settings.LOD = 0;

	Effects::g_pCurrentPass->apply(context);

	drawConstants.slope = tan(g_screenParams.fov / 2);
	drawConstants.pixelSizeConstant = g_screenParams.height / (2.0f * drawConstants.slope);

	Effects::SetSplatSize(settings.clusterSplatScale);
	Effects::UpdatePerLODBuffer(context);


	if (settings.drawFixedDepth)
	{
		drawRecursiveFixedDepth(context, 0, 0);
	}
	else
	{

		drawRecursive(context, 0, XMLoadFloat3(&octree->center), XMLoadFloat4(&Effects::cbPerObj.camPos), 0);
	}
}

void Regions_Spheres::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth)
{
	settings.LOD = max(settings.LOD, depth);
	Effects::cbPerLOD.currentLOD = depth; 
	Effects::UpdatePerLODBuffer(context);
	//clipping 
	XMVECTOR octreeCenterCamSpace = XMVector4Transform(center, XMLoadFloat4x4(&Effects::cbPerObj.worldMat));

	float camDist = XMVector3Length(octreeCenterCamSpace - cameraPos).m128_f32[0];

	//clipping 
	float dist = octree->range.x / (1 << depth);
	if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
	{
	//	return;
	}

	float worldradius = vertexBuffers[nodeIndex].data.maxWorldspaceSize;
	float pixelsize = (worldradius * drawConstants.pixelSizeConstant) / camDist;
	if (!vertexBuffers[nodeIndex].children ||pixelsize < g_lodSettings.pixelThreshhold)
	{

		settings.nodesDrawn++;

		LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
		context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
		context->Draw(vb.size, 0);
		g_statistics.verticesDrawn += vb.size;

	}
	else
	{
		XMVECTOR nextLevelCenterOffset = XMLoadFloat3(&octree->range) / (4 << depth);
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


void Regions_Spheres::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{
	settings.LOD = max(settings.LOD, depth);

	if (depth == settings.fixedDepth || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;
		LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;

		context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
		context->Draw(vb.size, 0);
		g_statistics.verticesDrawn += vb.size;
	}
	if (depth != settings.fixedDepth)
	{
		UINT8 numchildren = 0;

		for (int i = 0; i < 8; ++i)
		{
			if (vertexBuffers[nodeIndex].children & (0x01 << i))	//ist ith flag set T->the child exists
			{
				drawRecursiveFixedDepth(context, vertexBuffers[nodeIndex].firstChildIndex + numchildren, depth + 1);
				++numchildren;
			}
		}
	}
}

