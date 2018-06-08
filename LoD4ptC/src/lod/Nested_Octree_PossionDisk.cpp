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

	visibleNodesBlob.clear();
	SafeRelease(visibleNodesSRV);
	SafeRelease(visibleNodesTexture);

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

	this->device = device; 

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
	XMStoreFloat4(&Effects::cbShaderSettings.octreeSize, XMLoadFloat3(&octree->range)); 
	XMStoreFloat4(&Effects::cbShaderSettings.octreeMin, XMLoadFloat3(&octree->boundsMin));

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

	visibleNodesBlob.clear();
	SafeRelease(visibleNodesSRV);
	SafeRelease(visibleNodesTexture);

	create(device, vertices);
}


void Nested_Octree_PossionDisk::draw(ID3D11DeviceContext* const context)
{

	settings.nodesDrawn = 0;
	settings.LOD = 0;

	Effects::g_pCurrentPass->apply(context);



	drawConstants.slope = tan(g_screenParams.fov / 2);
	drawConstants.pixelSizeConstant = g_screenParams.height / 2.0f;


	traverseTreeAndMarkVisibleNodes(XMLoadFloat3(&octree->center), XMLoadFloat4(&Effects::cbPerObj.camPos));


	if (!visibleNodesBlob.size()) //object is not visible
	{
		return; 
	}

	D3D11_TEXTURE1D_DESC desc = { 0 };
	desc.ArraySize = 1;
	desc.Width = visibleNodesBlob.size();
	desc.Format = DXGI_FORMAT_R16G16_UINT;
	desc.Usage = D3D11_USAGE_IMMUTABLE;
	desc.MipLevels = 1; 
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE; 

	



	D3D11_SUBRESOURCE_DATA data = { 0 };
	data.pSysMem = visibleNodesBlob.data();

	HRESULT hr; 

	hr = device->CreateTexture1D(&desc, &data, &visibleNodesTexture);

	if (FAILED(hr))
	{
		std::cout << "Failed to upload visible node structure" << std::endl; 

		std::cin.get();
	}


	/*
	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc; 
	srvDesc.Format = DXGI_FORMAT_R16G16_UINT;
	srvDesc.Texture1D.MipLevels = 1;
	srvDesc.Texture1D.MostDetailedMip = 0; 
	srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1D; */

	hr = device->CreateShaderResourceView(visibleNodesTexture, NULL, &visibleNodesSRV); 

	if (FAILED(hr))
	{
		std::cout << "Failed to create visible node structure SRV" << std::endl;

		std::cin.get();
	}

	context->GSSetShaderResources(0, 1, &visibleNodesSRV); 

	if (settings.drawFixedDepth)
	{
		drawRecursiveFixedDepth(context, 0, 0);
	}
	else
	{
		drawRecursive(context, 0, 0);
	}

	visibleNodesBlob.clear();
	SafeRelease(visibleNodesSRV);
	SafeRelease(visibleNodesTexture);
	


}

void Nested_Octree_PossionDisk::drawRecursive(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{
	OctreeVectorNode<LOD_Utils::VertexBuffer>& node = vertexBuffers[nodeIndex]; 

	if (node.data.marked)
	{
		//for completeness, these cb vars are not used in adaptive splatsize calculation
		settings.nodesDrawn++;
		Effects::SetSplatSize(g_renderSettings.splatSize);	
		Effects::cbPerLOD.currentLOD = depth;
		Effects::UpdatePerLODBuffer(context);



		settings.LOD = max(settings.LOD, depth);

		node.data.marked = false; 
		++settings.nodesDrawn;
		context->IASetVertexBuffers(0, 1, &node.data.buffer, &drawConstants.strides, &drawConstants.offset);
		context->Draw(node.data.size, 0);
		g_statistics.verticesDrawn += node.data.size;
		char numChildren = 0; 
		for (int i = 0; i < 8; ++i)
		{
			if (node.children&(1 << i)) //child exists
			{
				drawRecursive(context, node.firstChildIndex + numChildren, depth+1);
				++numChildren;
			}
		}

	}

}


void Nested_Octree_PossionDisk::drawRecursiveFixedDepth(ID3D11DeviceContext* const context, UINT32 nodeIndex, int depth)
{
	LOD_Utils::VertexBuffer& vb = vertexBuffers[nodeIndex].data;
	
	if (!vb.marked)
	{
		return; 
	}

	vb.marked = false; 

	if (depth <= settings.fixedDepth || !vertexBuffers[nodeIndex].children)
	{
		settings.nodesDrawn++;

		float worldradius = g_renderSettings.splatSize * (1<<(octree->reachedDepth - depth));


		Effects::SetSplatSize(worldradius);
		Effects::cbPerLOD.currentLOD = depth;
		Effects::UpdatePerLODBuffer(context);



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
	XMMATRIX wvpMat = XMLoadFloat4x4(&Effects::cbPerObj.wvpMat); 

	wvpMat = XMLoadFloat4x4(&Effects::cbPerObj.worldMat) * XMLoadFloat4x4(&Effects::cbPerObj.viewMat);

	center.m128_f32[3] = 1.0f;


	{	//check if object is in view frustrum 
		XMVECTOR octreeCenterCamSpace = XMVector4Transform(center, wvpMat);

		//clipping 
		float dist = octree->range.x;

		//is BS in front of camera? 
		if (- dist * sqrt(2.0f) > octreeCenterCamSpace.m128_f32[2]) //object is behind camera
		{
			return;
		}

	}


	std::queue<XMFLOAT3> centerBuffer; 
	std::queue<UINT32> indexbuffer;

	UINT32 currentDepth = 0; 

	UINT32 nextLevelIndex = vertexBuffers[0].firstChildIndex -1; //-1 so we can use > instead of >= when checking if we are at the next lvl

	indexbuffer.push(0);
	XMFLOAT3 rootCenter3f;
	XMStoreFloat3(&rootCenter3f, center);
	centerBuffer.push(rootCenter3f);


	while (!indexbuffer.empty())
	{
		UINT32 currentIndex = indexbuffer.front();
		indexbuffer.pop();

		XMFLOAT3& center3f = centerBuffer.front();
		center = XMLoadFloat3(&center3f); 
		centerBuffer.pop();

		OctreeVectorNode<LOD_Utils::VertexBuffer>& node = vertexBuffers[currentIndex];
		node.data.marked = true;


		if (currentIndex > nextLevelIndex)
		{
			++currentDepth;
			nextLevelIndex = node.firstChildIndex - 1;	//-1 so we can use > instead of >= when checking if we are at the next lvl
		}
	//	std::cout << currentIndex << std::endl; 


		PerNodeData nodeDesc;


		if (node.children)
		{
			char childCounter = 0;
			XMVECTOR nextLevelCenterOffset = XMLoadFloat3(&octree->range) / (4 << currentDepth); //range/ (2^(depth+1)


			nodeDesc.firstChildOffset = indexbuffer.size() + 1;

			for (int i = 0; i < 8; ++i)
			{
				if (node.children&(1 << i)) //child exists
				{
					// thats how you turn an AABB into BS
					XMVECTOR childCenter = center + nextLevelCenterOffset * LOD_Utils::signVector(i);

					childCenter.m128_f32[3] = 1.0f;	//dont think i need to do perspective division but just in case


					XMVECTOR octreeCenterCamSpace = XMVector4Transform(childCenter, wvpMat);

					//clipping 
					float dist = octree->range.x / (2 << currentDepth); // 1<<depth+1

					float boundingDiameter = sqrt(2.0f) * dist;

					//TODO: check if child lies in ViewFrustrum

					bool frustumCheckFailed =
						abs(octreeCenterCamSpace.m128_f32[0] / octreeCenterCamSpace.m128_f32[3]) > boundingDiameter // left/right
						|| abs(octreeCenterCamSpace.m128_f32[1] / octreeCenterCamSpace.m128_f32[3]) > boundingDiameter // top/bottom
						|| - boundingDiameter > octreeCenterCamSpace.m128_f32[2] / octreeCenterCamSpace.m128_f32[3]; //behind camera

					if ( true || !frustumCheckFailed) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
					{
						//float distance = XMVector3Length(childCenter - cameraPos).m128_f32[0];

						float worldradius = g_renderSettings.splatSize * (1 << (octree->reachedDepth - currentDepth));
						worldradius = octree->cellsizeForDepth[currentDepth].x;

						float pixelsize = (worldradius * drawConstants.pixelSizeConstant) / abs(octreeCenterCamSpace.m128_f32[2]);

						if (pixelsize > g_lodSettings.pixelThreshhold) // --> expand node
						{

							nodeDesc.childBits |= 1 << i; //set flag at child location 

							indexbuffer.push(node.firstChildIndex + childCounter);

							XMFLOAT3 childCenter3f;
							XMStoreFloat3(&childCenter3f, childCenter);
							centerBuffer.push(childCenter3f);
						}

					}
					++childCounter;

				}//end foreach existing child
			}
		}
		else //leaf node
		{
			nodeDesc.childBits = 0xff00;
		}


		visibleNodesBlob.push_back(nodeDesc);
	}

}


/* old draw recursive code
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
if (4 * dist * dist < octreeCenterCamSpace.m128_f32[2] / octreeCenterCamSpace.m128_f32[3]) //object is behind camera // cellsize has same value in each component if octree was created w/ cube argument
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
*/