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
}



Regions_Spheres::TweakSettings Regions_Spheres::settings;
TwBar* Regions_Spheres::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("K-Means Spheres");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Regions_Spheres::settings.gridResolution, NULL);
	TwAddVarRW(tweakBar, "Max Feature Dist", TW_TYPE_FLOAT, &Regions_Spheres::settings.maxFeatureDist, NULL);
	TwAddVarRW(tweakBar, "Max Centroids per node", TW_TYPE_UINT32, &Regions_Spheres::settings.maxCentroidsPerNode, NULL);
	TwAddVarRW(tweakBar, "Max. Iterations", TW_TYPE_UINT32, &Regions_Spheres::settings.iterations, NULL);

	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Regions_Spheres::settings.expansionThreshold, NULL);

	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Regions_Spheres::settings.maxDepth, NULL);

	TwEnumVal distanceFunctionEV[] = { { OctreeFlags::dfEuclididan, "Euclidian distance" },{ OctreeFlags::dfManhattan, "Manhattan distance" } };
	TwType twDistanceFunction = TwDefineEnum("Distance Function", distanceFunctionEV, ARRAYSIZE(distanceFunctionEV));
	//	TwAddVarRW(tweakBar, "Distance Function", twDistanceFunction, &settings.distanceFunction, NULL);


	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Regions_Spheres::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Regions_Spheres::settings.drawFixedDepth, NULL);

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Regions_Spheres::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Regions_Spheres::settings.nodesDrawn, NULL);


	return tweakBar;

}

// pray for -O3
void Regions_Spheres::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating kmeans_spheres" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();

	//create here

	std::vector<SphereVertex> initalEllpticalVerts;	//at max LOD verts = circles (ellipsoid w/ major/minor  unit length and orthogonal to normal 

	initalEllpticalVerts.reserve(vertices.size());

	for (auto vert : vertices)
	{
		initalEllpticalVerts.push_back(SphereVertex(vert));
	}

	/*
	* Create and push down is broken and i wana test the kmeans impl
	* this HAS to be changed later
	* as it will rape the performance
	* not that the kmeans will have good perf anyawys, but this NEEDS TO BE FIXED, DO YOU HEAR ME FUTURE ME?!?
	*
	* This is future me, i think i fixed it some time ago, cant remember :O
	*/
	octree = new NestedOctree<SphereVertex>(initalEllpticalVerts, settings.gridResolution, settings.expansionThreshold, settings.maxDepth, OctreeCreationMode::CreateAndPushDown, OctreeFlags::createCube);


	initalEllpticalVerts.clear();

	float scaling[9] = { 1,1,1,0,0,0,0,0,0 }; 

	octree->createRegionGrowing(settings.maxFeatureDist, scaling, settings.iterations);


	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	std::cout << "Created Octree w/ Kmeans spheres with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes << std::endl;
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

	create(device, vertices);
}

void Regions_Spheres::centroidsToSphereSplats(const std::vector<Centroid>& centroids, MatX9f& verts, const std::vector<UINT32>& vertCentroidTable, std::vector<SphereVertex>& outVerts)
{
	std::vector<Eigen::Matrix<float, Eigen::Dynamic, 3>> vertsPerCentroid;


	for (int i = 0; i < centroids.size(); ++i)
	{
		vertsPerCentroid.push_back(Eigen::Matrix<float, Eigen::Dynamic, 3>());
	}

	//sort verts according to correstponding centroid
	for (int i = 0; i < verts.rows(); ++i)
	{
		Eigen::Matrix<float, Eigen::Dynamic, 3>& mat = vertsPerCentroid[vertCentroidTable[i]];

		mat.conservativeResize(mat.rows() + 1, 3);
		mat.row(mat.rows() - 1) = verts.row(i).head(3).transpose(); //pox.xyz per vertex
	}
	//replace set of all child verts w/ calculated cluster splats
	verts.resize(0, 0);

	for (int i = 0; i < centroids.size(); ++i)
	{
		if (!(vertsPerCentroid[i].rows() - 1)) continue; //very degenerate cluster? immpossibru unless clusters>verts? 
		outVerts.push_back(SphereVertex());

		const Centroid& cent = centroids[i];


		SphereVertex& newVert = outVerts[i];
		newVert.pos.x = cent.features(0);
		newVert.pos.y = cent.features(1);
		newVert.pos.z = cent.features(2);
		XMStoreFloat3(&newVert.normal, LOD_Utils::polarToCartNormal(XMVectorSet(cent.features(3), cent.features(4), 0, 0)));
		newVert.color.x = cent.features(5);
		newVert.color.y = cent.features(6);
		newVert.color.z = cent.features(7);
		newVert.color.w = cent.features(8);


		//PCA to determine major and minor of the resultion elliptical splat
		Eigen::Vector3f centroid;

		centroid << cent.features(0), cent.features(1), cent.features(2);

		// centering
		Eigen::MatrixXf centeredVerts = vertsPerCentroid[i].rowwise() - centroid.transpose();

		Eigen::Matrix<float, 3, 3> covariance = centeredVerts.adjoint() * centeredVerts;

		covariance /= (centeredVerts.rows() - 1);

		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca(covariance);


		newVert.radius = pca.eigenvalues()(0);

	}

}



void Regions_Spheres::draw(ID3D11DeviceContext* const context)
{
	settings.nodesDrawn = 0;
	settings.LOD = 0;

	Effects::g_pCurrentPass->apply(context);

	drawConstants.slope = tan(g_screenParams.fov / 2);
	drawConstants.pixelSizeConstant = g_screenParams.height / (2.0f * drawConstants.slope) * g_renderSettings.splatSize;

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



	//thats how you turn an AABB into BS

	XMMATRIX worldMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat);
	XMVECTOR octreeCenterWorldpos = XMVector3Transform(center, worldMat);

	XMFLOAT3& cellsize3f = octree->cellsizeForDepth[depth];
	XMVECTOR cellsize = XMLoadFloat3(&cellsize3f);

	XMVECTOR distance = octreeCenterWorldpos - cameraPos;


	//clipping 
	XMVECTOR octreeCenterCamSpace = XMVector4Transform(center, XMLoadFloat4x4(&Effects::cbPerObj.wvpMat));

	//clipping 
	float dist = octree->range.x / (1 << depth);
	if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2] / octreeCenterWorldpos.m128_f32[3]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
	{
		return;
	}

	//float worldradius = g_renderSettings.splatSize * (1 << (octree->reachedDepth - depth));

	float pixelsize = (vertexBuffers[nodeIndex].data.maxPixelWorldSize * drawConstants.pixelSizeConstant) / abs(octreeCenterCamSpace.m128_f32[2]);


	if (pixelsize <  g_lodSettings.pixelThreshhold || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;
		Effects::cbPerLOD.currentLOD = depth;
		Effects::UpdatePerLODBuffer(context);

		LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
		context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
		context->Draw(vb.size, 0);
		g_statistics.verticesDrawn += vb.size;

	}
	else
	{
		XMVECTOR nextLevelCenterOffset = XMLoadFloat3(&octree->range) / (2 << depth);
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

	if (depth == settings.fixedDepth || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;

		float worldradius = g_renderSettings.splatSize * (1 << (octree->reachedDepth - depth));
		Effects::SetSplatSize(worldradius);
		Effects::cbPerLOD.currentLOD = depth;
		Effects::UpdatePerLODBuffer(context);


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

