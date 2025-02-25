#include "Octree_Naive_Avg.h"



Octree_Naive_Avg::Octree_Naive_Avg()
{
	
}


Octree_Naive_Avg::~Octree_Naive_Avg()
{
	SafeRelease(cbPerFrameBuffer);
	SafeRelease(vertexBuffer);
	delete octree; 
}

Octree_Naive_Avg::TweakSettings Octree_Naive_Avg::settings;
TwBar* Octree_Naive_Avg::setUpTweakBar() 
{
	TwBar* tweakBar = TwNewBar("Octree Naive Avg");

	TwAddVarRW(tweakBar, "Max depth", TW_TYPE_INT32, &Octree_Naive_Avg::settings.maxDepth, NULL);
	TwAddSeparator(tweakBar, "sep", NULL); 
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Octree_Naive_Avg::settings.fixedDrawDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Octree_Naive_Avg::settings.drawFixedDepth, NULL);

	return tweakBar; 

}

// pray for -O3
void Octree_Naive_Avg::create(ID3D11Device* const device, vector<Vertex>& vertices)
{
	HRESULT hr;

	std::cout << "\n===================================================" << std::endl;
	std::cout << "Creating Octree_Naive_Avg" << std::endl; 

	XMFLOAT3 min3f, max3f; 

	XMVECTOR minv = XMLoadFloat3(&vertices[0].pos);
	XMVECTOR maxv = XMLoadFloat3(&vertices[0].pos);

	for (auto it : vertices)
	{
		XMVECTOR pos = XMLoadFloat3(&it.pos); 
		minv = XMVectorMin(pos, minv); 
		maxv = XMVectorMax(pos, maxv); 
	}

	XMStoreFloat3(&min3f, minv); 
	XMStoreFloat3(&max3f, maxv); 

	octree = new Octree<Vertex>(min3f, max3f, settings.maxDepth, false);


	for (auto it : vertices)
	{
		octree->insert(it); 
	}

	std::cout << "Created Octree with Depth: " << octree->reachedDepth << " max Depth = " << settings.maxDepth << std::endl;

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

	std::vector<Vertex> orderedVertices; 
	orderedVertices.resize(vertices.size()); 
	



	std::cout << "===================================================\n" << std::endl;

}

void Octree_Naive_Avg::traverseAndAverageOctree(OctreeInternal::OctreeNode<Vertex>* pNode)
{
#ifdef DEBUG
	if (pNode->marked)
	{
		std::cerr << "Octree traversal reached already marked node" << std::endl; 
	}
#endif // DEBUG


	if (pNode->isInternal)	//internal Node 
	{
		XMVECTOR avgPos = XMVectorSet(0, 0, 0, 0); 
		XMVECTOR avgNormal = XMVectorSet(0, 0, 0, 0);
		XMVECTOR avgColor = XMVectorSet(0, 0, 0, 0);

		size_t numChildren = 0; 
		pNode->marked = true;

		OctreeInternal::OctreeInternalNode<Vertex>* intNode = static_cast<OctreeInternal::OctreeInternalNode<Vertex>*>(pNode);
		for (auto child : intNode->children)
		{
			if (child)
			{
				if (!child->marked)
				{
					++numChildren;
					traverseAndAverageOctree(child);
				}

				avgPos += XMLoadFloat3(&child->data.pos);
				avgNormal += XMLoadFloat3(&child->data.normal);
				avgColor += XMLoadFloat4(&child->data.color);
			}
		}
		
		XMStoreFloat3(&pNode->data.pos, avgPos / numChildren); 
		XMStoreFloat3(&pNode->data.normal, avgNormal / numChildren);
		XMStoreFloat4(&pNode->data.color, avgColor / numChildren);

	}
	else	// leaf node
	{
		XMVECTOR avgPos = XMVectorSet(0, 0, 0, 0);
		XMVECTOR avgNormal = XMVectorSet(0, 0, 0, 0);
		XMVECTOR avgColor = XMVectorSet(0, 0, 0, 0);

		OctreeInternal::OctreeLeafNode<Vertex>* leafNode = static_cast<OctreeInternal::OctreeLeafNode<Vertex>*>(pNode);

		for (auto data : leafNode->verts)
		{
			avgPos += XMLoadFloat3(&data.pos);
			avgNormal += XMLoadFloat3(&data.normal);
			avgColor += XMLoadFloat4(&data.color);
		}


		int numChildren = leafNode->verts.size();
		XMStoreFloat3(&pNode->data.pos, avgPos / numChildren);
		XMStoreFloat3(&pNode->data.normal, avgNormal / numChildren);
		XMStoreFloat4(&pNode->data.color, avgColor / numChildren);

		pNode->marked = true;
	}
}


void Octree_Naive_Avg::recreate(ID3D11Device* const device, vector<Vertex>& vertices)
{
	delete octree; 

	SafeRelease(cbPerFrameBuffer);
	SafeRelease(vertexBuffer); 

	create(device, vertices); 
}

void Octree_Naive_Avg::draw(ID3D11DeviceContext* const context)
{

	cbPerFrame.fixedLODdepth = Octree_Naive_Avg::settings.drawFixedDepth ? Octree_Naive_Avg::settings.fixedDrawDepth : 0; 
	context->UpdateSubresource(cbPerFrameBuffer, 0, NULL, &cbPerFrame, 0, 0);
	context->GSSetConstantBuffers(1, 1, &cbPerFrameBuffer); 

}

