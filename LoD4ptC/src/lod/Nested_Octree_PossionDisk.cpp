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

	TwEnumVal distanceFunctionEV[] = { { OctreeFlags::dfEuclididan, "Euclidian distance" },{ OctreeFlags::dfManhattan, "Manhattan distance" } };
	TwType twDistanceFunction = TwDefineEnum("Distance Function", distanceFunctionEV, ARRAYSIZE(distanceFunctionEV));
	TwAddVarRW(tweakBar, "Distance Function", twDistanceFunction, &settings.distanceFunction, NULL);


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



	octree = new NestedOctree<Vertex>(vertices, 
		Nested_Octree_PossionDisk::settings.gridResolution,
		Nested_Octree_PossionDisk::settings.expansionThreshold, 
		Nested_Octree_PossionDisk::settings.maxDepth,
		OctreeCreationMode::CreatePossionDisk,
		OctreeFlags::createCube | settings.distanceFunction);	//depending on vert count this may take a while




	std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
	std::cout << "Created Octree w/ Possion Disk Sampling with Depth: " << octree->reachedDepth << " and #nodes: " << octree->numNodes << std::endl;
	std::cout << "Took: " << elapsed.count() << "ms" << std::endl;



	//init GPU stuff

	//load to GPU

	std::cout << "uploading relevant octree data to gpu" << std::endl;

	octree->getStructureAsVector<LOD_Utils::VertexBuffer, ID3D11Device*>(vertexBuffers, &LOD_Utils::createVertexBufferFromNode, device);

	//delete octree; //remove this mb later if i wana do more stuff with the same tree --> delete internal nodes and apply different upsampling

	//LOD_Utils::printTreeStructure(vertexBuffers); 

	g_renderSettings.splatSize = octree->cellsizeForDepth[octree->reachedDepth].x; // basic sice recomendation

	Effects::cbShaderSettings.maxLOD = octree->reachedDepth; 
	Effects::cbShaderSettings.octreeSize = octree->range; 
	Effects::cbShaderSettings.octreeMin = octree->boundsMin; 
	g_statistics.maxDepth = octree->reachedDepth; 
	std::cout << "=======================DONE========================\n" << std::endl;

}

void Nested_Octree_PossionDisk::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	delete octree;

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



	drawConstants.slope = tan(g_screenParams.fov / 2);
	drawConstants.heightDiv2DivSlope = g_screenParams.height / (2.0f * drawConstants.slope);

	if (settings.drawFixedDepth)
	{


		drawRecursive(context, 0, XMLoadFloat3(&octree->center), XMLoadFloat4(&Effects::cbPerObj.camPos), 0); 
		/*
		char mark = (vertexBuffers[0].data.marked + 1)%256; 
		traverseTreeAndMarkVisibleNodes(XMLoadFloat3(&octree->center), XMLoadFloat4(&Effects::cbPerObj.camPos)); 

		D3D11_TEXTURE1D_DESC desc = { 0 };
		desc.ArraySize = 1; 
		desc.Width = visibleNodesBlob.size(); 
		desc.Format = DXGI_FORMAT_R32_TYPELESS; 
		desc.Usage = D3D11_USAGE_DEFAULT; 
	
		D3D11_SUBRESOURCE_DATA data = { 0 }; 
		data.pSysMem = visibleNodesBlob.data(); 

		device->CreateTexture1D(&desc, &data, &visibleNodesTexture); 

		for (auto node : vertexBuffers)
		{
			LOD_Utils::VertexBuffer& vb = node.data; 

			if (vb.marked == mark)
			{
				context->IASetVertexBuffers(0, 1, &vb.buffer, &drawConstants.strides, &drawConstants.offset);
				context->Draw(vb.size, 0);
				g_statistics.verticesDrawn += vb.size;
			}
		}

		visibleNodesBlob.clear();
		SafeRelease(visibleNodesTexture); 
		*/
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

	octreeCenterWorldpos.m128_f32[3] = 1.0f; 

	

	XMVECTOR octreeCenterCamSpace = XMVector4Transform(octreeCenterWorldpos, XMLoadFloat4x4(&Effects::cbPerObj.wvpMat));

	//clipping 
	float dist = octree->range.x / (1 << depth);
	if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2] / octreeCenterWorldpos.m128_f32[3]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
	{
		return;
	}

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


void Nested_Octree_PossionDisk::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{

	if (depth == settings.fixedDepth || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;

		float worldradius = g_renderSettings.splatSize * (1<<octree->reachedDepth - depth);


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


void Nested_Octree_PossionDisk::traverseTreeAndMarkVisibleNodes(XMVECTOR& center, const XMVECTOR& cameraPos)
{

	std::queue<XMFLOAT3> centerBuffer; 
	std::queue<UINT32> indexbuffer;

	UINT32 currentDepth = 0; 

	UINT32 nextLevelIndex = 0; 

	indexbuffer.push(0);
	XMFLOAT3 center3f;
	XMStoreFloat3(&center3f, center); 
	centerBuffer.push(center3f); 

	while (!indexbuffer.empty())
	{

		UINT32 currentIndex = indexbuffer.front();
		indexbuffer.pop();

		center3f = centerBuffer.front(); 
		center = XMLoadFloat3(&center3f); 
		centerBuffer.pop();


		if (currentIndex == nextLevelIndex)
		{
			++currentDepth; 
			nextLevelIndex = 0; 
		}

		OctreeVectorNode<LOD_Utils::VertexBuffer>& node = vertexBuffers[currentIndex]; 
		node.data.marked = (node.data.marked + 1) % 256;

		PerNodeData nodeDesc;
		nodeDesc.firstChildOffset = indexbuffer.size(); 

		int childCounter = 0;
		XMVECTOR nextLevelCenterOffset = XMLoadFloat3(&octree->range) / (2 << currentDepth);


		for (int i = 0; i < 8; ++i)
		{
			if ( node.children&(1<<i)) //child exists
			{
				// thats how you turn an AABB into BS
				XMVECTOR childCenter = center + nextLevelCenterOffset * LOD_Utils::signVector(i); 
				XMMATRIX worldMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat);
				XMVECTOR octreeCenterWorldpos = XMVector3Transform(childCenter, worldMat);

				XMFLOAT3& cellsize3f = octree->cellsizeForDepth[currentDepth];
				XMVECTOR cellsize = XMLoadFloat3(&cellsize3f);

				XMVECTOR distance = octreeCenterWorldpos - cameraPos;

				childCenter.m128_f32[3] = 1.0f;


				XMVECTOR octreeCenterCamSpace = XMVector4Transform(childCenter, XMLoadFloat4x4(&Effects::cbPerObj.wvpMat));

				//clipping 
				float dist = octree->range.x / (1 << currentDepth);

				
				//check if child lies in ViewFrustrum
				if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2] / octreeCenterWorldpos.m128_f32[3]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
				{

					float worldradius = g_renderSettings.splatSize * (1 << octree->reachedDepth - currentDepth);

					float pixelsize = (worldradius* drawConstants.heightDiv2DivSlope) / XMVector3Length(distance).m128_f32[0];

					settings.nodesDrawn++;

					if (pixelsize > g_lodSettings.pixelThreshhold && node.children)
					{

						nodeDesc.childBits |= 1 << i; //set flag at child location 

						if (!nodeDesc.firstChildOffset)//only set for the first child (other children will be directly after first child (Breadth first)
							nodeDesc.firstChildOffset = indexbuffer.size() + childCounter;
						childCounter++;

						if (!nextLevelIndex)
						{
							nextLevelIndex = currentIndex + nodeDesc.firstChildOffset;
						}

						indexbuffer.push(currentIndex + nodeDesc.firstChildOffset);
						XMFLOAT3 childCenter3f;
						XMStoreFloat3(&childCenter3f, childCenter);
						centerBuffer.push(childCenter3f);
					}

				}
			}
		}
		
		visibleNodesBlob.push_back(nodeDesc);

	}



}
