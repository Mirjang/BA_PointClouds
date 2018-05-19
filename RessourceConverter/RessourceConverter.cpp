// RessourceConverter.cpp : Defines the entry point for the console application.
//



#include <fstream>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <DirectXMath.h>
#include <Windows.h>

#include "Vertex.h"
#include "tinyply.h"


using namespace tinyply; 

// This software is in the public domain. Where that dedication is not
// recognized, you are granted a perpetual, irrevocable license to copy,
// distribute, and modify this file as you see fit.
// Authored in 2015 by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
// https://github.com/ddiakopoulos/tinyply
// Version 2.0
int loadPLY(const std::string& name, std::vector<Vertex>& out)
{
	try
	{
		// Read the file and create a std::istringstream suitable
		// for the lib -- tinyply does not perform any file i/o.
		std::ifstream ss(name, std::ios::binary);

		if (ss.fail())
		{
			throw std::runtime_error("failed to open " + name);
		}

		PlyFile file;

		file.parse_header(ss);

		std::cout << "================================================================\n";

		for (auto c : file.get_comments()) std::cout << "Comment: " << c << std::endl;

		for (auto e : file.get_elements())
		{
			std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
			for (auto p : e.properties)
			{
				std::cout << "\tproperty - " << p.name << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
			}
		}

		std::cout << "================================================================\n";

		// Tinyply 2.0 treats incoming data as untyped byte buffers. It's now
		// up to users to treat this data as they wish. See below for examples.
		std::shared_ptr<PlyData> vertices, normals, colors, alphas, faces, texcoords;


		// The header information can be used to programmatically extract properties on elements
		// known to exist in the file header prior to reading the data. For brevity of this sample, properties 
		// like vertex position are hard-coded: 
		try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		try { normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		try { colors = file.request_properties_from_element("vertex", { "red", "green", "blue" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		if (!colors)
		{
			//much special treatment
			try { colors = file.request_properties_from_element("vertex", { "diffuse_red", "diffuse_green", "diffuse_blue" }); }
			catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }
		}

		try { alphas = file.request_properties_from_element("vertex", { "alpha" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		/* // we dont need faces where we are going
		try { faces = file.request_properties_from_element("face", { "vertex_indices" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

		try { texcoords = file.request_properties_from_element("face", { "texcoord" }); }
		catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }
		*/


		//TODO: match pattern above to vertex struct

		file.read(ss);

		// Good place to put a breakpoint!
		if (vertices) std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
		if (normals) std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
		if (colors) std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
		if (alphas) std::cout << "\tRead " << alphas->count << " total vertex alphas " << std::endl;
		if (faces) std::cout << "\tRead " << faces->count << " total faces (triangles) " << std::endl;
		if (texcoords) std::cout << "\tRead " << texcoords->count << " total texcoords " << std::endl;

		// Example: type 'conversion' to your own native types - Option A
		{
			const size_t numVerticesBytes = vertices->buffer.size_bytes();
			struct float3 { float x, y, z; };
			std::vector<float3> verts(vertices->count);
			std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
			out.clear();
			out.resize(vertices->count);

			for (int i = 0; i < vertices->count; ++i)
			{
				std::memcpy(&out[i].pos, &verts[i], 3 * sizeof(float));
			}


			if (normals)
			{
				std::vector<float3> norms(vertices->count);
				std::memcpy(norms.data(), normals->buffer.get(), numVerticesBytes);
				for (int i = 0; i < vertices->count; ++i)
				{
					std::memcpy(&out[i].normal, &norms[i], 3 * sizeof(float));
				}
			}

			if (colors)
			{
				const size_t numColorsBytes = colors->buffer.size_bytes();
				struct UCHAR3 { UCHAR x, y, z; };
				std::vector<UCHAR3> cols(colors->count);
				std::memcpy(cols.data(), colors->buffer.get(), numColorsBytes);

				std::vector<UCHAR> alpha(alphas ? alphas->count : 0);
				if (alphas)
				{
					std::memcpy(alpha.data(), alphas->buffer.get(), alphas->buffer.size_bytes());
				}



				for (int i = 0; i < vertices->count; ++i)
				{
					DirectX::XMStoreFloat4(&out[i].color, DirectX::XMVectorScale(DirectX::XMVectorSet((float)cols[i].x, (float)cols[i].y, (float)cols[i].z, alphas ? (float)alpha[i] : 1), 1.0 / 255));
				}
			}

		}

	}
	catch (const std::exception & e)
	{
		std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
		return -1;
	}



	return 0;
}



int main(int argc, char** argv)
{

	if (argc < 2)
	{
		std::cout << "no file entered" << std::endl; 
		exit(-1); 
	}

	for (int i = 0; i < argc; ++i)
	{
		std::cout << argv[i] << std::endl; 
	}

	std::cin.get(); 

	std::vector<Vertex> out; 

	std::string filename = std::string(argv[1]); 

	loadPLY(filename, out); 

	filename.replace(filename.find(std::string(".ply")), std::string(".ply").size(), std::string(".vertarr"));

	//remove(filename.c_str()); 


	std::ofstream fs(filename, std::ios::binary);

	fs.write((char*)out.data(), out.size()*sizeof(Vertex)); 

	fs.close(); 


    return 0;
}
