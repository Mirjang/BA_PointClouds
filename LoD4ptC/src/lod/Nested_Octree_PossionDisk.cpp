#include "Nested_Octree_PossionDisk.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <chrono>

#include "../rendering/Effects.h"



Nested_Octree_PossionDisk::Nested_Octree_PossionDisk()
{
}


Nested_Octree_PossionDisk::~Nested_Octree_PossionDisk()
{
	SafeRelease(cbPerFrameBuffer);

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();

	delete octree;
}


Nested_Octree_PossionDisk::TweakSettings Nested_Octree_PossionDisk::settings;
TwBar* Nested_Octree_PossionDisk::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("Octree Possion Disk");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Nested_Octree_PossionDisk::settings.gridResolution, NULL);
	TwAddVarRW(tweakBar, "Expansion Threshold", TW_TYPE_UINT32, &Nested_Octree_PossionDisk::settings.expansionThreshold, NULL);
	TwAddVarRW(tweakBar, "Max. Depth", TW_TYPE_UINT32, &Nested_Octree_PossionDisk::settings.maxDepth, NULL);


	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Nested_Octree_PossionDisk::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Nested_Octree_PossionDisk::settings.drawFixedDepth, NULL);

	TwAddSeparator(tweakBar, "sep2", NULL);
	TwAddVarRO(tweakBar, "LOD", TW_TYPE_INT32, &Nested_Octree_PossionDisk::settings.LOD, NULL);
	TwAddVarRO(tweakBar, "Nodes drawn", TW_TYPE_INT32, &Nested_Octree_PossionDisk::settings.nodesDrawn, NULL);


	return tweakBar;

}

// pray for -O3
void Nested_Octree_PossionDisk::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT hr;

	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating Octree_PossionDisk" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();

	//render loop



	octree = new NestedOctree<Vertex>(vertices, Nested_Octree_PossionDisk::settings.gridResolution, Nested_Octree_PossionDisk::settings.expansionThreshold, Nested_Octree_PossionDisk::settings.maxDepth, OctreeCreationMode::CreateAndAverage);	//depending on vert count this may take a while




	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	std::cout << "Created Octree w/ Possion Disk Sampling with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes << std::endl;
	std::cout << "Took: " << elapsed.count() << "ms" << std::endl;



	//init GPU stuff

	D3D11_BUFFER_DESC desc_perFrameBuffer;
	ZeroMemory(&desc_perFrameBuffer, sizeof(D3D11_BUFFER_DESC));
	desc_perFrameBuffer.Usage = D3D11_USAGE_DEFAULT;
	desc_perFrameBuffer.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	desc_perFrameBuffer.ByteWidth = sizeof(cbLODPerFrame);

	hr = device->CreateBuffer(&desc_perFrameBuffer, NULL, &cbPerFrameBuffer);
	if (FAILED(hr))
	{
		if (FAILED(hr))
		{
			std::cout << "failed to create constant buffer " << hr << std::endl;
			std::cin.get();
		}
	}

	//load to GPU

	std::cout << "uploading relevant octree data to gpu" << std::endl;

	octree->getStructureAsVector<LOD_Utils::VertexBuffer, ID3D11Device*>(vertexBuffers, &LOD_Utils::createVertexBufferFromNode, device);

	//delete octree; //remove this mb later if i wana do more stuff with the same tree --> delete internal nodes and apply different upsampling

	//LOD_Utils::printTreeStructure(vertexBuffers); 

	std::cout << "===================================================\n" << std::endl;

}

void Nested_Octree_PossionDisk::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	delete octree;

	SafeRelease(cbPerFrameBuffer);

	for (auto it : vertexBuffers)
	{
		SafeRelease(it.data.buffer);
	}
	vertexBuffers.clear();

	create(device, vertices);
}

void Nested_Octree_PossionDisk::draw(ID3D11DeviceContext* const context)
{

	settings.nodesDrawn = 0;
	settings.LOD = 0;

	Effects::g_pCurrentPass->apply(context);

	cbPerFrame.fixedLODdepth = Nested_Octree_PossionDisk::settings.drawFixedDepth ? Nested_Octree_PossionDisk::settings.fixedDepth : 0;
	context->UpdateSubresource(cbPerFrameBuffer, 0, NULL, &cbPerFrame, 0, 0);
	context->GSSetConstantBuffers(1, 1, &cbPerFrameBuffer);

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

void Nested_Octree_PossionDisk::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, XMVECTOR& center, const XMVECTOR& cameraPos, int depth)
{
	settings.LOD = max(settings.LOD, depth);



	//thats how you turn an AABB into BS

	XMMATRIX worldMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat);
	XMVECTOR octreeCenterWorldpos = XMVector3Transform(center, worldMat);

	XMFLOAT3& cellsize3f = octree->cellsizeForDepth[depth];
	XMVECTOR cellsize = XMLoadFloat3(&cellsize3f);

	XMVECTOR distance = octreeCenterWorldpos - cameraPos;




	float worldradius = max(max(cellsize3f.x, cellsize3f.y), cellsize3f.z);

	cbPerFrame.splatSize = worldradius;
	context->UpdateSubresource(cbPerFrameBuffer, 0, NULL, &cbPerFrame, 0, 0);	//unnecessary updates->precompute


																				/*
																				if (XMVector3Dot(distance, XMLoadFloat4(&Effects::cbPerObj.camDir)).m128_f32[0] < worldradius) //z coord is behind camera
																				{
																				return;
																				}
																				*/

	worldradius = g_renderSettings.splatSize * (octree->reachedDepth - depth);

	float pixelsize = (worldradius* drawConstants.heightDiv2DivSlope) / XMVector3Length(distance).m128_f32[0];

	if (pixelsize < g_lodSettings.pixelThreshhold || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;

		LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
		context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
		context->Draw(vb.size, 0);
		g_statistics.verticesDrawn += vb.size;

	}
	else
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


void Nested_Octree_PossionDisk::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{

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
