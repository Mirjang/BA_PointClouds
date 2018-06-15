#include "Kmeans_Spheres.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>
#include <random>
#include <cfloat>
#include <stack>

#include <Eigen/Eigenvalues>

#include "../rendering/Effects.h"

using namespace DirectX; 


Kmeans_Spheres::Kmeans_Spheres()
{
}


Kmeans_Spheres::~Kmeans_Spheres()
{
}



Kmeans_Spheres::TweakSettings Kmeans_Spheres::settings;
TwBar* Kmeans_Spheres::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("K-Means Spheres");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Kmeans_Spheres::settings.gridResolution, NULL);
	TwAddVarRW(tweakBar, "Max. Iterations", TW_TYPE_UINT32, &Kmeans_Spheres::settings.iterations, NULL);
	TwAddVarRW(tweakBar, "Upsample Rate", TW_TYPE_UINT32, &Kmeans_Spheres::settings.upsampleRate, NULL);
	TwAddVarRW(tweakBar, "Max Centroids per node", TW_TYPE_UINT32, &Kmeans_Spheres::settings.maxCentroidsPerNode, NULL);
	TwAddVarRW(tweakBar, "simpleDistance", TW_TYPE_BOOLCPP, &Kmeans_Spheres::settings.simpleDistance, NULL);

	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Kmeans_Spheres::settings.expansionThreshold, NULL);

	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Kmeans_Spheres::settings.maxDepth, NULL);

	TwEnumVal distanceFunctionEV[] = { { OctreeFlags::dfEuclididan, "Euclidian distance" },{ OctreeFlags::dfManhattan, "Manhattan distance" } };
	TwType twDistanceFunction = TwDefineEnum("Distance Function", distanceFunctionEV, ARRAYSIZE(distanceFunctionEV));
//	TwAddVarRW(tweakBar, "Distance Function", twDistanceFunction, &settings.distanceFunction, NULL);


	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Kmeans_Spheres::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Kmeans_Spheres::settings.drawFixedDepth, NULL);

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Kmeans_Spheres::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Kmeans_Spheres::settings.nodesDrawn, NULL);


	return tweakBar;

}

// pray for -O3
void Kmeans_Spheres::create(ID3D11Device* const device, vector<Vertex>& vertices)
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

	//run kmeans per octree node BOTTOM UP (for reasons of improving quality later)(performance will be crap tho) 
	//there is no clustering in leaf nodes ATM to preserve original samples at max LOD
	std::stack<NestedOctreeNode<SphereVertex>*> stack;
	if(!octree->root->isLeaf()) //point cloud too small -> LOD is unnecessary
		stack.push(octree->root); 

	while (!stack.empty())
	{
		NestedOctreeNode<SphereVertex>* node = stack.top(); 

		if (node->allChildrenLeafOrMarked())
		{
			stack.pop();

			MatX9f featureVecs; 


			UINT32 vertOffset = 0; 

			for (int i = 0; i < 8; ++i) //collect all child verts
			{
				NestedOctreeNode<SphereVertex>* child = node->children[i];
				if (child)
				{
					child->marked = false; 

					featureVecs.conservativeResize(featureVecs.rows() + child->data.size(), featureVecs.cols()); 


					UINT32 vertIndex = 0;
					for(; vertIndex < child->data.size(); ++vertIndex)
					{

						const SphereVertex& vert = child->data[vertIndex];

						//add normals in polar coords
						XMVECTOR polarcoords =  LOD_Utils::cartToPolarNormal(XMLoadFloat3(&vert.normal));

						Vec9f fv;
						fv << vert.pos.x, vert.pos.y, vert.pos.z, polarcoords.m128_f32[0], polarcoords.m128_f32[1],
							vert.color.x, vert.color.y, vert.color.z, vert.color.w;

						featureVecs.row(vertOffset + vertIndex) = fv;

						 
					}

					vertOffset += vertIndex; 
				}
			}

			Kmeans kmeansCalc(featureVecs, Vec9f::Ones());

			//if for some reason we have very large nodes
			UINT32 numCentroids = node->allChildrenLeafs() ?
				min(vertOffset / settings.upsampleRate, settings.maxCentroidsPerNode)
				: vertOffset / settings.upsampleRate;

			//MAGIC
			kmeansCalc.runKMEANS(featureVecs, numCentroids, settings.iterations);
			node->marked = true; 

			// ok so im really hacking around here.. this node should be empty unless during expansion one of the child nodes didnt get enough verts
			//in this case we will prob. delete vital verts that cannot be restored ... this is just a temporary hack until i fix the fucking octree insertion
			node->data.clear();

			centroidsToSphereSplats(kmeansCalc.centroids, featureVecs, kmeansCalc.vertCentroidTable, node->data);
			featureVecs.resize(0, 0); // release memory

		}
		else	//add all children to queue w/ higher priority
		{
			for (int i = 0; i < 8; ++i) //collect all child verts
			{
				NestedOctreeNode<SphereVertex>* child = node->children[i];
				if (child && !child->isLeaf())
				{
					stack.push(child); 
				}
			}

		}
	}




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

void Kmeans_Spheres::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();

	create(device, vertices);
}

void Kmeans_Spheres::centroidsToSphereSplats(const std::vector<Centroid>& centroids, MatX9f& verts, const std::vector<UINT32>& vertCentroidTable, std::vector<SphereVertex>& outVerts)
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



void Kmeans_Spheres::draw(ID3D11DeviceContext* const context)
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

void Kmeans_Spheres::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth)
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


void Kmeans_Spheres::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
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

