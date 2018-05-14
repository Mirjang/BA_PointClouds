#include "Nested_Octree_Naive_Avg.h"

#include <iostream>
#include <sstream>

#include "Octree_Naive_Avg.h"



Nested_Octree_Naive_Avg::Nested_Octree_Naive_Avg()
{

}


Nested_Octree_Naive_Avg::~Nested_Octree_Naive_Avg()
{
	SafeRelease(cbPerFrameBuffer);
	SafeRelease(vertexBuffer);
	delete octree;
}

Nested_Octree_Naive_Avg::TweakSettings Nested_Octree_Naive_Avg::settings;
TwBar* Nested_Octree_Naive_Avg::setUpTweakBar()
{
	TwBar* tweakBar = TwNewBar("Octree Naive Avg");
	TwAddVarRW(tweakBar, "Grid Resolution", TW_TYPE_UINT32, &Nested_Octree_Naive_Avg::settings.gridResolution, NULL);

	TwAddSeparator(tweakBar, "sep", NULL);
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Nested_Octree_Naive_Avg::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Nested_Octree_Naive_Avg::settings.drawFixedDepth, NULL);

	return tweakBar;

}

// pray for -O3
void Nested_Octree_Naive_Avg::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT hr;

	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating Octree_Naive_Avg" << std::endl;

	octree = new NestedOctree<Vertex>(vertices, Nested_Octree_Naive_Avg::settings.gridResolution, Nested_Octree_Naive_Avg::settings.expansionThreshold);	//depending on vert count this may take a while


	std::cout << "Created Octree with Depth: " << octree->reachedDepth << std::endl;

	traverseAndAverageOctree(octree->root);

	std::cout << "Finished traversing and averaging" << std::endl;


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





	std::cout << "===================================================\n" << std::endl;

}

void Nested_Octree_Naive_Avg::traverseAndAverageOctree(NestedOctreeNode<Vertex>* pNode)
{
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])	//subsample from child
		{
			size_t x = (0x01 & i) * octree->gridResolution / 2;
			size_t maxX = x + octree->gridResolution / 2;
			size_t y = (0x02 & i) * octree->gridResolution / 2;
			size_t maxY = y + octree->gridResolution / 2;
			size_t z = (0x04 & i) * octree->gridResolution / 2;
			size_t maxZ = z + octree->gridResolution / 2;

			for (; x < maxX; ++x)
			{
				for (y = maxY - octree->gridResolution/2; y < maxY; ++y)
				{
					for ( z = maxZ - octree->gridResolution/2 ; z < maxZ; ++z)
					{




					}
				}
			}


		}
	}




	




}


void Nested_Octree_Naive_Avg::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	delete octree;

	SafeRelease(cbPerFrameBuffer);
	SafeRelease(vertexBuffer);

	create(device, vertices);
}

void Nested_Octree_Naive_Avg::draw(ID3D11DeviceContext* const context)
{

	cbPerFrame.fixedLODdepth = Nested_Octree_Naive_Avg::settings.drawFixedDepth ? Nested_Octree_Naive_Avg::settings.fixedDepth : 0;
	context->UpdateSubresource(cbPerFrameBuffer, 0, NULL, &cbPerFrame, 0, 0);
	context->GSSetConstantBuffers(1, 1, &cbPerFrameBuffer);

}

