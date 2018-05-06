#include "Octree_Naive_Avg.h"



Octree_Naive_Avg::Octree_Naive_Avg()
{
	
}


Octree_Naive_Avg::~Octree_Naive_Avg()
{
	delete octree; 
}

Octree_Naive_Avg::TweakSettings Octree_Naive_Avg::settings;
TwBar* Octree_Naive_Avg::setUpTweakBar() 
{
	TwBar* tweakBar = TwNewBar("Octree Naive Avg");

	TwAddVarRW(tweakBar, "Max depth", TW_TYPE_INT32, &Octree_Naive_Avg::settings.maxDepth, NULL);
	TwAddSeparator(tweakBar, "sep", NULL); 
	TwAddVarRW(tweakBar, "Fixed depth", TW_TYPE_INT32, &Octree_Naive_Avg::settings.fixedDepth, NULL);
	TwAddVarRW(tweakBar, "Draw Fixed depth", TW_TYPE_BOOLCPP, &Octree_Naive_Avg::settings.drawFixedDepth, NULL);

	return tweakBar; 

}

void Octree_Naive_Avg::create(const ID3D11Device* device, vector<Vertex>& vertices) 
{

}

void Octree_Naive_Avg::recreate(const ID3D11Device* device, vector<Vertex>& vertices)
{
	delete octree; 
	create(device, vertices); 
}

void Octree_Naive_Avg::draw(const ID3D11DeviceContext* context)
{


}

