#include "Nested_Octree_Naive_Avg.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>

#include "../rendering/Effects.h"


Nested_Octree_Naive_Avg::Nested_Octree_Naive_Avg()
{

}


Nested_Octree_Naive_Avg::~Nested_Octree_Naive_Avg()
{

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();

	delete octree;
}

Nested_Octree_Naive_Avg::TweakSettings Nested_Octree_Naive_Avg::settings;
TwBar* Nested_Octree_Naive_Avg::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("Octree Naive Avg");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Nested_Octree_Naive_Avg::settings.gridResolution, NULL);
	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Nested_Octree_Naive_Avg::settings.expansionThreshold, NULL);
	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Nested_Octree_Naive_Avg::settings.maxDepth, NULL);


	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Nested_Octree_Naive_Avg::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Nested_Octree_Naive_Avg::settings.drawFixedDepth, NULL);

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Nested_Octree_Naive_Avg::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Nested_Octree_Naive_Avg::settings.nodesDrawn, NULL);


	return tweakBar;

}

// pray for -O3
void Nested_Octree_Naive_Avg::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT hr;

	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating Octree_Naive_Avg" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();

	//render loop

	octree = new NestedOctree<Vertex>(vertices, Nested_Octree_Naive_Avg::settings.gridResolution, Nested_Octree_Naive_Avg::settings.expansionThreshold, Nested_Octree_Naive_Avg::settings.maxDepth, OctreeCreationMode::CreateNaiveAverage, OctreeFlags::createCube);	//depending on vert count this may take a while

	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	std::cout << "Created OctreeV1 with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes<< std::endl;
	std::cout << "Took: " << elapsed.count() << "s" << std::endl; 


	//load to GPU

	std::cout << "uploading relevant octree data to gpu" << std::endl;

	octree->getStructureAsVector<LOD_Utils::VertexBuffer, ID3D11Device*>(vertexBuffers, &LOD_Utils::createVertexBufferFromNode, device);

	//delete octree; //remove this mb later if i wana do more stuff with the same tree --> delete internal nodes and apply different upsampling

	//LOD_Utils::printTreeStructure(vertexBuffers); 

	g_renderSettings.splatSize = octree->cellsizeForDepth[octree->reachedDepth].x; // so we are at least close
	Effects::cbShaderSettings.maxLOD = octree->reachedDepth;
	g_statistics.maxDepth = octree->reachedDepth;


	std::cout << "===================================================\n" << std::endl;

}

//unused
/*/
void Nested_Octree_Naive_Avg::traverseAndUpsampleOctree(NestedOctreeNode<Vertex>* pNode)
{
	int numchildren = 0;

	for (int i = 0; i < 8; ++i)	//averaging has to be done bottom up
	{
		if (pNode->children[i]) 
		{
			++numchildren; 
			if (!pNode->children[i]->marked)	// marked should be false	anyways but hey
			{
				traverseAndUpsampleOctree(pNode->children[i]);
			}
		}
	}


	XMVECTOR sumPos = XMVectorSet(0, 0, 0, 0); 
	XMVECTOR sumNor = XMVectorSet(0, 0, 0, 0);
	XMVECTOR sumCol = XMVectorSet(0, 0, 0, 0);


	int numVerts = 0; 

	for (int i = 0; i < 8; ++i) 	//subsample from children
	{

		if (numVerts == settings.upsampleRate)
		{
			XMFLOAT3 avgpos;
			XMFLOAT3 avgnor; 
			XMFLOAT4 avgcol; 

			XMStoreFloat3(&avgpos, sumPos / numVerts); 
			XMStoreFloat3(&avgnor, sumNor / numVerts); 
			XMStoreFloat4(&avgcol, sumCol / numVerts); 

			numVerts = 0; 
			pNode->data.push_back(Vertex(avgpos, avgnor, avgcol));

			sumPos = XMVectorSet(0, 0, 0, 0);
			sumNor = XMVectorSet(0, 0, 0, 0);
			sumCol = XMVectorSet(0, 0, 0, 0);
		}


		if (pNode->children[i])
		{
			// index in parrent array
			size_t parentX = (0x01 & i) * octree->gridResolution / 2;
			size_t parentY = (0x02 & i) * octree->gridResolution / 2;
			size_t parentZ = (0x04 & i) * octree->gridResolution / 2;

			for (size_t x = 0; x < octree->gridResolution; x+=2)
			{
				for (size_t y = 0; y < octree->gridResolution; y+=2)
				{
					for ( size_t z =0 ; z < octree->gridResolution; z+=2)
					{




					}
				}
			}
		}
	}
}
*/

void Nested_Octree_Naive_Avg::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	delete octree;

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer); 
	}
	vertexBuffers.clear(); 

	create(device, vertices);
}

void Nested_Octree_Naive_Avg::draw(ID3D11DeviceContext* const context)
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
		drawRecursive(context, 0, XMLoadFloat3(&octree->center), XMLoadFloat4(&Effects::cbPerObj.camPos), 0);
	}


}

void Nested_Octree_Naive_Avg::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth)
{
	settings.LOD = max(settings.LOD, depth);



	//thats how you turn an AABB into BS

	XMMATRIX worldMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat);
	XMVECTOR octreeCenterWorldpos = XMVector3Transform(center, worldMat);

	XMFLOAT3& cellsize3f = octree->cellsizeForDepth[depth];
	XMVECTOR cellsize = XMLoadFloat3(&cellsize3f);

	XMVECTOR distance = octreeCenterWorldpos - cameraPos;


	//clipping 
	XMVECTOR octreeCenterCamSpace = XMVector4Transform(octreeCenterWorldpos, XMLoadFloat4x4(&Effects::cbPerObj.wvpMat));

	//clipping 
	float dist = octree->range.x / (1 << depth);
	if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2] / octreeCenterWorldpos.m128_f32[3]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
	{
		return;
	}

	float worldradius = g_renderSettings.splatSize * (1<<(octree->reachedDepth - depth)); 

	float pixelsize = (worldradius* drawConstants.heightDiv2DivSlope) / XMVector3Length(distance).m128_f32[0]; 



	if (pixelsize < g_lodSettings.pixelThreshhold || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;
		Effects::SetSplatSize(worldradius);
		Effects::cbPerLOD.currentLOD = depth;
		Effects::UpdatePerLODBuffer(context);

		LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
		context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset); 
		context->Draw(vb.size, 0); 
		g_statistics.verticesDrawn +=vb.size;

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


void Nested_Octree_Naive_Avg::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{

	if (depth == settings.fixedDepth ||!vertexBuffers[nodeIndex].children)
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
	if(depth != settings.fixedDepth)
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

