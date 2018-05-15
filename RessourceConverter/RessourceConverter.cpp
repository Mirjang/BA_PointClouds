// RessourceConverter.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


#include <iostream>
#include <sstream>
#include <vector>
#include <string>

#include "RessourceLoader.h"

#include "PointCloud.h"



int main(int argc, char** argv)
{

	RessourceLoader* loader = new RessourceLoader(); 

	PointCloud* data = new PointCloud(); 

	if (argc < 2)
	{
		std::cout << "no file entered" << std::endl; 
		exit(-1); 
	}




    return 0;
}

