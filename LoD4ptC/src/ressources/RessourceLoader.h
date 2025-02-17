#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstring>

#include "../rendering/Vertex.h"
#include "../rendering/PointCloud.h"


class RessourceLoader
{
public: 

	//Fills PointCloud::vertices with vertices loaded from assets/name.format
	// returns: 0 on success,  != 0 on fail
	int loadAsset(const std::string& name, PointCloud* out); 

private: 

	int loadPLY(const std::string& name, PointCloud* out);
	int loadVertarr(const std::string& name, PointCloud* out);



};

extern RessourceLoader* g_RessourceLoader;
