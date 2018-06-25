#include "NestedOctree.h"




#define VERBOSE 
#undef VERBOSE

static UINT64 fuckupCTR = 0;

//TODO: split this up into template function since only the last part actually depends on wether Sphere or Elliptical Verts are used
void NestedOctree<SphereVertex>::createRegionGrowing(NestedOctreeNode<SphereVertex>* pNode, size_t depth)
{

	if (pNode->isLeaf()) return; 

	
	
	if (g_lodSettings.useThreads) // MT
	{
		std::vector<std::thread*> theadpool; 

		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{

					theadpool.push_back(
						new std::thread([=] {createRegionGrowing(pNode->children[i], depth + 1);})//next level  
					);

				}
			}
		}

		for (auto thread : theadpool)
		{
			thread->join(); 
			delete thread; 
		}
	}
	else
	{
		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{
					createRegionGrowing(pNode->children[i], depth + 1); //next level  
				}
			}
		}
	}
	
	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.aquire(); 
	}

	srand(RND_SEED);

	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				lowerBound = lowerBound.cwiseMin(fv).eval();
				upperBound = upperBound.cwiseMax(fv).eval();

			}
		}
	}

	lowerBound -= lowerBound.cwiseAbs() * 0.025;
	upperBound += upperBound.cwiseAbs() * 0.025;
	Vec9f nodeRange = upperBound - lowerBound;

	// diagonalSq / (GridRes)^3 / 2^depth === (spacial) size of one cell and rest 0
	// 
	float sqMaxDist = regionConstants.maxDistScaling * (diagonalSq / (gridResolution * gridResolution * gridResolution *(1 << depth)));
	Vec9f normalisationConsts = nodeRange.cwiseInverse();
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan

	Vec9f normalisationConstScaling = normalisationConsts.cwiseProduct(regionConstants.scaling);

	//unnecessary complicated ? -- works tho
	UINT32 searchResolution =static_cast<int>(nodeRange.head<3>().maxCoeff() / (sqrt(sqMaxDist)));
	searchResolution = (searchResolution + 7) >> 3;
	searchResolution = std::max(1U,searchResolution);

	//UINT32 searchResolution = gridResolution;

	//calulates clusters for current node
	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID> vertMap(searchResolution*searchResolution);

	Eigen::Vector3f evGridStart = lowerBound.head<3>(); 
	Eigen::Vector3f evCellsize = Eigen::Vector3f::Ones() * (upperBound - lowerBound).head<3>().maxCoeff() / searchResolution;
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, searchResolution, searchResolution*searchResolution;


	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				//point location in the inscribed grid (searchResolution) <<-- make this float safe
				UINT32 gridIndex = (fv.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ); 

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second->push_back(vert);
				}
				else
				{
					std::vector<SphereVertex>* newVec = new std::vector<SphereVertex>(); 
					newVec->push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<SphereVertex>*>(gridIndex, newVec));
				}
			}
		}
	}
	//clustering

	while (!vertMap.empty())
	{
		// prob.better to use existing vert as centroid
		//Vec9f centroid = ((Vec9f::Random() + Vec9f::Ones()) / 2.0f).cwiseProduct(invNormalisationConst) - lowerBound;

		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<SphereVertex>& centroidCell = *it->second;

		assert(!centroidCell.empty()); 
		UINT32 centroidIndex = rand() % centroidCell.size();

		const SphereVertex& centVertex = centroidCell[centroidIndex];

		Vec9f centroid;
		centroid << centVertex.pos.x, centVertex.pos.y, centVertex.pos.z, centVertex.normal.x, centVertex.normal.y, centVertex.normal.z,
			centVertex.color.x, centVertex.color.y, centVertex.color.z;

		Vec9f lastCentroid = Vec9f::Ones() * (upperBound.squaredNorm()+24); // so 1st checl is passed ... using FLT_MAX apperently causes overflow

		MatX9f clusterVerts; 


		//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
		for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
		{
			centroidCellIndex = (centroid.head<3>() - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
			int cells = 0;

			std::queue<UINT32> frontier;
			std::unordered_set<UINT32, hashID> exploredNodes;
			exploredNodes.insert(centroidCellIndex);

			frontier.push(centroidCellIndex);

			while (!frontier.empty())
			{
				++cells;
				UINT32 currentIDX = frontier.front();
				frontier.pop();

				bool oneInRange = findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
				if (oneInRange) // found at least one vert -> explore neighbours 
				{
					auto hull = getCellHull(searchResolution, currentIDX);
					for (UINT32 idx : hull)
					{							
						exploredNodes.insert(idx);
						if (idx < searchResolution*searchResolution*searchResolution && exploredNodes.find(idx) == exploredNodes.end())
						{
							frontier.push(idx);
						}
					}
				}
			}


			if (exploredNodes.size() > 100)
			{
				std::cout << exploredNodes.size() << std::endl;
			}

			lastCentroid = centroid;
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
			clusterVerts.resize(0, 9);
		}//END current iteration

		 //last iteration
		centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);
		std::queue<UINT32> frontier;
		std::unordered_set<UINT32, hashID> exploredNodes;
		frontier.push(centroidCellIndex);
		exploredNodes.insert(centroidCellIndex);


		std::vector<SphereVertex> deletedVerts; 

		while (!frontier.empty())
		{
			UINT32 currentIDX = frontier.front();
			frontier.pop();

			bool oneInRange = findVerticesInCellAndRemove(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts, &deletedVerts);
			if (oneInRange) // found at least one vert -> explore neighbours 
			{
				auto hull = getCellHull(searchResolution, currentIDX);
				for (UINT32 idx : hull)
				{
					if (idx < searchResolution*searchResolution*searchResolution && exploredNodes.find(idx) == exploredNodes.end())
					{
						exploredNodes.insert(idx);
						frontier.push(idx);
					}
				}
			}
		}

		pNode->data.push_back(getVertFromCluster(clusterVerts, depth, &deletedVerts));
	}//vertMap.empty()

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.release();
	}
}

//TODO: split this up into template function since only the last part actually depends on wether Sphere or Elliptical Verts are used
void NestedOctree<EllipticalVertex>::createRegionGrowing(NestedOctreeNode<EllipticalVertex>* pNode, size_t depth)
{

	if (pNode->isLeaf()) return;

	if (g_lodSettings.useThreads) // MT
	{
		std::vector<std::thread*> theadpool;

		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{

					theadpool.push_back(
						new std::thread([=] {createRegionGrowing(pNode->children[i], depth + 1); })//next level  
					);

				}
			}
		}

		for (auto thread : theadpool)
		{
			thread->join();
			delete thread;
		}
	}
	else
	{
		if (!pNode->allChildrenLeafs()) // Bottom up -> leafs first
		{
			for (int i = 0; i < 8; ++i)
			{
				if (pNode->children[i])
				{
					createRegionGrowing(pNode->children[i], depth + 1); //next level  
				}
			}
		}
	}

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.aquire();
	}

	srand(RND_SEED);



	Vec9f lowerBound = Vec9f::Ones() * FLT_MAX;
	Vec9f upperBound = Vec9f::Ones() * FLT_MAX * (-1);

	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				lowerBound = lowerBound.cwiseMin(fv).eval();
				upperBound = upperBound.cwiseMax(fv).eval();

			}
		}
	}

	upperBound += upperBound.cwiseAbs() * 0.025;

	Vec9f nodeRange = (upperBound - lowerBound);


	//default: search half a grid
	float sqMaxDist = regionConstants.maxDistScaling * (nodeRange / (gridResolution * 2)).squaredNorm() *(1 << depth);

	UINT32 searchResolution = std::max(1,
		static_cast<int>(nodeRange.head<3>().cwiseProduct(regionConstants.scaling.head<3>()).norm() / (sqrt(sqMaxDist)*2.5)));

	//round up to closest power of 8 : 
	searchResolution = ((searchResolution + 7) >> 3) << 3;
	//	searchResolution = 1; 

	std::vector<UINT32> cellHullIndices = getCellHull(searchResolution);

	//calulates clusters for current node
	// gridRes^3 hashmap w/ chaining
	// use id as hash function, since we already compute the key (= grid index) 
	std::unordered_map < UINT32, std::vector<EllipticalVertex>*, hashID> vertMap(searchResolution*searchResolution);

	Eigen::Vector3f evGridStart = lowerBound.head<3>();
	Eigen::Vector3f evCellsize = Eigen::Vector3f::Ones() * (upperBound - lowerBound).head<3>().maxCoeff() / searchResolution;
	Eigen::Vector3i oneGridGridSQ;
	oneGridGridSQ << 1, searchResolution, searchResolution*searchResolution;

	Vec9f normalisationConsts = nodeRange.cwiseInverse();
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isnan(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan
	normalisationConsts = normalisationConsts.unaryExpr([](float f) {return std::isinf(f) ? 0 : f; }); //if an entire col is 0 Inverse will result in nan

	Vec9f normalisationConstScaling = normalisationConsts.cwiseProduct(regionConstants.scaling);

	//sort all child verts into grid for --hopefully-- reasonable speed
	for (int i = 0; i < 8; ++i)
	{
		if (pNode->children[i])
		{
			for (auto vert : pNode->children[i]->data)
			{
				Vec9f fv;
				fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
					vert.color.x, vert.color.y, vert.color.z;

				//point location in the inscribed grid (searchResolution) <<-- make this float safe
				UINT32 gridIndex = (fv.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

				auto result = vertMap.find(gridIndex);
				if (result != vertMap.end())
				{
					result->second->push_back(vert);
				}
				else
				{
					std::vector<EllipticalVertex>* newVec = new std::vector<EllipticalVertex>();
					newVec->push_back(vert);
					vertMap.insert(std::pair<UINT32, std::vector<EllipticalVertex>*>(gridIndex, newVec));
				}
			}
		}
	}
	//clustering




	while (!vertMap.empty())
	{
		// prob.better to use existing vert as centroid
		//Vec9f centroid = ((Vec9f::Random() + Vec9f::Ones()) / 2.0f).cwiseProduct(invNormalisationConst) - lowerBound;

		UINT32 centroidCellIndex = rand() % vertMap.size();
		std::unordered_map < UINT32, std::vector<EllipticalVertex>*, hashID>::iterator it(vertMap.begin());
		std::advance(it, centroidCellIndex);
		std::vector<EllipticalVertex>& centroidCell = *it->second;

		assert(!centroidCell.empty());
		UINT32 centroidIndex = rand() % centroidCell.size();

		const SphereVertex& centVertex = centroidCell[centroidIndex];

		Vec9f centroid;
		centroid << centVertex.pos.x, centVertex.pos.y, centVertex.pos.z, centVertex.normal.x, centVertex.normal.y, centVertex.normal.z,
			centVertex.color.x, centVertex.color.y, centVertex.color.z;

		Vec9f lastCentroid = Vec9f::Ones() * (upperBound.squaredNorm() + 24); // so 1st checl is passed ... using FLT_MAX apperently causes overflow

		MatX9f clusterVerts;


		//it = 1 <--> last it not in loop, because it also removes vertices from vertMap
		for (size_t iteration = 1; iteration<regionConstants.maxIterations && (centroid - lastCentroid).squaredNorm()>MIN_CENTROID_SQDIFFERENCE; ++iteration)
		{
			//std::unordered_set<UINT32, hashID> exploredNodes(30, hash_ID); 
#ifdef VERBOSE
			std::cout << (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>() << std::endl;
#endif // VERBOSE

			centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

			for (int i = 0; i < cellHullIndices.size(); ++i)
			{
				int currentIDX = centroidCellIndex + cellHullIndices[i];

				if (currentIDX >= 0 && currentIDX < searchResolution*searchResolution*searchResolution)	//check neighbourhood
				{
					//					exploredNodes.insert(centroidCellIndex);
					findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
				}
			}


			//END vertexFinding
#ifdef VERBOSE

			if (clusterVerts.rows() == 0)	//this should nevernot happen if everything goes according to plan... BUT IT DOES FUUUUU
			{
				std::cout << "rows: " << clusterVerts.rows() << std::endl;

				std::cout << "searching EVERYTHING FFS" << std::endl;

				for (auto mapIt = vertMap.begin(); mapIt != vertMap.end(); ++mapIt)
				{
					findVerticesInCell(centroid, normalisationConstScaling, sqMaxDist, mapIt->first, vertMap, clusterVerts);
				}


				std::cout << "rows: " << clusterVerts.rows() << std::endl;

				++fuckupCTR;
				std::cout << fuckupCTR << " FUUUUCK " << std::endl;
				std::cout << lastCentroid.transpose() << "\n" << centroid.transpose() << std::endl;

				std::cout << "last idx: " << (lastCentroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().transpose() << std::endl;
				std::cout << "new idx : " << (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().transpose() << std::endl;
				std::cout << fuckupCTR << " FUUUUCK " << std::endl;
				continue;
			}
#endif // VERBOSE

			lastCentroid = centroid;
			centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
			clusterVerts.resize(0, 9);

#ifdef VERBOSE
			std::cout << "------------endit -------------------------" << std::endl;
#endif

		}//END current iteration

		 //last iteration
		centroidCellIndex = (centroid.head(3) - evGridStart).cwiseQuotient(evCellsize).cast<int>().dot(oneGridGridSQ);

		for (int i = 0; i < cellHullIndices.size(); ++i)
		{
			int currentIDX = centroidCellIndex + cellHullIndices[i];

			if (currentIDX >= 0 && currentIDX < searchResolution*searchResolution*searchResolution)	//check neighbourhood
			{
				findVerticesInCellAndRemove(centroid, normalisationConstScaling, sqMaxDist, currentIDX, vertMap, clusterVerts);
			}

		}//END last iteration

//#ifdef VERBOSE
		if (clusterVerts.rows() == 0)
		{
			++fuckupCTR;
			std::cout << fuckupCTR << " FUUUUCK " << std::endl;
			continue;
		}
//#endif // VERBOSE

		centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
		EllipticalVertex newVert;
		newVert.pos.x = centroid(0);
		newVert.pos.y = centroid(1);
		newVert.pos.z = centroid(2);
		newVert.normal.x = centroid(3);
		newVert.normal.y = centroid(4);
		newVert.normal.z = centroid(5);
		newVert.color.x = centroid(6);
		newVert.color.y = centroid(7);
		newVert.color.z = centroid(8);
		newVert.color.w = 1.0f; //ignoring alpha

		if (clusterVerts.rows() == 1)	//isolated vertex -> use axis normal to surfaceNormal as axis
		{
			XMVECTOR xmNormal = XMLoadFloat3(&newVert.normal); 
			XMVECTOR xmMajor = XMVector3Orthogonal(xmNormal); 

			XMStoreFloat3(&newVert.major, xmMajor); 
			XMStoreFloat3(&newVert.minor, XMVector3Cross(xmNormal, xmMajor));

		}
		else
		{
			//calc new supervert


			Eigen::MatrixX3f spacialMat = clusterVerts.leftCols<3>();
			clusterVerts.resize(0, 9);

			spacialMat = spacialMat.rowwise() - centroid.head<3>().transpose();


			Eigen::Matrix3f spacialCovariance = spacialMat.transpose()* spacialMat;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> pca(spacialCovariance);

			
		/* EIgen API: 
		*		
		*The eigenvalues are repeated according to their algebraic multiplicity, so there are as many eigenvalues as rows in the matrix. 
		*The eigenvalues are sorted in increasing order.
		*/	



			Eigen::Vector3f major = pca.eigenvectors().col(2) * pca.eigenvalues()(2);
			newVert.major.x = major.x(); 
			newVert.major.y = major.y(); 
			newVert.major.z = major.z(); 

			Eigen::Vector3f minor = pca.eigenvectors().col(1) * pca.eigenvalues()(1);
			newVert.minor.x = minor.x();
			newVert.minor.y = minor.y();
			newVert.minor.z = minor.z();
		}

		pNode->data.push_back(newVert);

	}//vertMap.empty()

	if (g_lodSettings.useThreads) //  MT
	{
		runNodeCalculations.release();
	}
}

template<>
SphereVertex NestedOctree<SphereVertex>::getVertFromCluster(const MatX9f& clusterVerts, UINT32 depth, const std::vector<SphereVertex>* boundaryVerts)
{
	Vec9f centroid = clusterVerts.colwise().sum() / clusterVerts.rows();
	SphereVertex newVert;
	newVert.pos.x = centroid(0);
	newVert.pos.y = centroid(1);
	newVert.pos.z = centroid(2);
	newVert.normal.x = centroid(3);
	newVert.normal.y = centroid(4);
	newVert.normal.z = centroid(5);
	newVert.color.x = centroid(6);
	newVert.color.y = centroid(7);
	newVert.color.z = centroid(8);
	newVert.color.w = 1.0f; //ignoring alpha

	if (clusterVerts.rows() == 1)	//isolated vertex --> if no normals are available this should prob. be added to closest cluster or smth
	{
		newVert.radius = 1 << (reachedDepth - depth);
	}
	else
	{
		Eigen::MatrixX3f spacialMat = clusterVerts.leftCols<3>();
		spacialMat = spacialMat.rowwise() - centroid.head<3>().transpose();
		if (flags&OctreeFlags::normalsGenerate)	//calc normals and check for too large discrepancy
		{
			// if verts live on a plane the smalles eigenvalue should be close to zero
			Eigen::Matrix3f spacialCovariance = spacialMat.transpose()* spacialMat;
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> pca(spacialCovariance);

			newVert.radius = pca.eigenvalues()(2); //largest
			Eigen::Vector3f nor = pca.eigenvectors().col(0);	//smallest

			newVert.normal.x = nor.x(); 
			newVert.normal.y = nor.y(); 
			newVert.normal.z = nor.z(); 

		}
		else //normals are already available any have been considered -> cluster is pretty much a disk 
		{
			if (boundaryVerts) // find radius of furthest vert
			{
//				assert(boundaryVerts->size() == 1); 
//				newVert.radius = sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff()) / (*boundaryVerts)[0].radius + (*boundaryVerts)[0].radius;
//				newVert.radius *= 1 << (reachedDepth - depth); 
				
				if (depth == reachedDepth - 1) // first level of clusters, no knowledge of radii of initial points? 
				{
					newVert.radius = sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff());
				}
				else
				{
					newVert.radius = sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff()) + (*boundaryVerts)[0].radius;
				}
				
			}
			else
			{
				newVert.radius =  sqrtf(spacialMat.rowwise().squaredNorm().maxCoeff());

				newVert.radius += 1 << (reachedDepth - depth);

			}
		}
	}
	return newVert; 
}

template<>
bool NestedOctree<SphereVertex>::findVerticesInCellAndRemove(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist, const UINT32 cellidx,
	std::unordered_map < UINT32, std::vector<SphereVertex>*, hashID>& vertMap, MatX9f& clusterVerts, std::vector<SphereVertex>* boundaryVerts)
{
	float maxDist = 0.0f; 
	if (boundaryVerts && !boundaryVerts->empty())
	{
		Eigen::Vector3f boundaryPos; 
		boundaryPos << (*boundaryVerts)[0].pos.x, (*boundaryVerts)[0].pos.y, (*boundaryVerts)[0].pos.z; 

		maxDist = (boundaryPos - centroid.head<3>()).norm() + (*boundaryVerts)[0].radius; // mb use (d+r)^2 = r^2+d^2+2dr to avoid sqrt calc? 
	}


	auto result = vertMap.find(cellidx);
	bool oneInRange = false;
	if (result != vertMap.end())
	{
		std::vector<SphereVertex>* cellVector = result->second;

		for (auto it = cellVector->begin(); it != cellVector->end(); )
		{
			const SphereVertex& vert = *it;
			//add normals in polar coords	
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;
			float dist = (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm();

			if (dist < sqMaxDist)	//vert is in range
			{
				clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

				clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();

				float radiusDist = (fv.head<3>()-centroid.head<3>()).norm() + vert.radius;
				if (boundaryVerts &&radiusDist>maxDist)
				{
					maxDist = radiusDist;
					boundaryVerts->resize(1); 
					(*boundaryVerts)[0] = vert; 
				}

				it = cellVector->erase(it);
				oneInRange = true;
			}
			else
			{
				++it;
			}
		}
		if (cellVector->empty())
		{
			delete cellVector;
			vertMap.erase(cellidx);
		}
	}//result != vertMap.end()
	return oneInRange;
}

template<>
bool NestedOctree<EllipticalVertex>::findVerticesInCellAndRemove(const Vec9f& centroid, const Vec9f& normalisationConstScaling, const float& sqMaxDist, const UINT32 cellidx,
	std::unordered_map < UINT32, std::vector<EllipticalVertex>*, hashID>& vertMap, MatX9f& clusterVerts, std::vector<EllipticalVertex>* boundaryVerts)
{
	float maxDist = 0.0f;
	if (boundaryVerts && !boundaryVerts->empty())
	{
		maxDist = XMVector3Length(XMLoadFloat3(&(*boundaryVerts)[0].pos)).m128_f32[0] + XMVector3Length(XMLoadFloat3(&(*boundaryVerts)[0].major)).m128_f32[0]; // mb use (d+r)^2 = r^2+d^2+2dr to avoid sqrt calc? 
	}


	auto result = vertMap.find(cellidx);
	bool oneInRange = false;
	if (result != vertMap.end())
	{
		std::vector<EllipticalVertex>* cellVector = result->second;

		for (auto it = cellVector->begin(); it != cellVector->end(); )
		{
			const EllipticalVertex& vert = *it;
			//add normals in polar coords	
			Vec9f fv;
			fv << vert.pos.x, vert.pos.y, vert.pos.z, vert.normal.x, vert.normal.y, vert.normal.z,
				vert.color.x, vert.color.y, vert.color.z;
			float dist = (fv - centroid).cwiseProduct(normalisationConstScaling).squaredNorm();

			if (dist < sqMaxDist)	//vert is in range
			{
				clusterVerts.conservativeResize(clusterVerts.rows() + 1, 9);

				clusterVerts.row(clusterVerts.rows() - 1) = fv.transpose();

				//todo detetmine dist: vert 
				float radiusDist = fv.head<3>().norm() + XMVector3Length(XMLoadFloat3(&vert.major)).m128_f32[0];
				if (boundaryVerts &&radiusDist>maxDist)
				{
					maxDist = radiusDist;
					boundaryVerts->resize(1);
					(*boundaryVerts)[0] = vert;
				}

				it = cellVector->erase(it);
				oneInRange = true;
			}
			else
			{
				++it;
			}
		}
		if (cellVector->empty())
		{
			delete cellVector;
			vertMap.erase(cellidx);
		}
	}//result != vertMap.end()
	return oneInRange;
}