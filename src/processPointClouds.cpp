// PCL lib Functions for processing point clouds 
//#pragma once
#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //Vale
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    region.setInputCloud(cloudRegion);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>

std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
//vale
typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

for(int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


 //template<typename PointXYZ>
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    
    /*
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    
    //Segment the largest plannar component from the remainig cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

 
    if (inliers->indices.size()==0)
    {
        std::cout << "Couldnt stimate a planar model " << std::endl;
    } */

// RANSAC PLANE

   
    std::unordered_set<int> inliers;

    while(maxIterations--){
		 //code 
		//Pick randomly 3 points
        
		std::unordered_set<int> inliers_work;
		while(inliers_work.size() < 3)
			inliers_work.insert(rand()%(cloud->points.size()));
		
		float x1, x2, y1, y2, x3, y3, z1, z2, z3;
		auto itr = inliers_work.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float i = (((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1)));
		float j = (((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1)));
		float k = (((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1)));

        float a = i;
		float b = j;
		float c = k;
		float d1 = ((-1)*((i*x1)+(j*y1)+(k*z1)));

		for(int index = 0; index < cloud->points.size(); index	++)
		{
			if(inliers_work.count(index)>0)
			continue;

			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs(a*x4+b*y4+c*z4+d1)/sqrt(a*a+b*b+c*c);

			if(d <= distanceThreshold)
				inliers_work.insert(index);

		}

        if(inliers_work.size()>inliers.size())
		{
			inliers = inliers_work;
		}
    }  

    
  // copy the final indices into the appropriate format
    pcl::PointIndices::Ptr inliers2 {new pcl::PointIndices};

    std::cout << "Converting from unordered_set to pcl::PointIndices"<<std::endl;
    
    for ( auto it = inliers.begin(); it != inliers.end(); ++it )
    {
        inliers2->indices.push_back(*it);
    }

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers2,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}



template<typename PointT>
//void clusterHelper1(int indice, const std::vector<std::vector<float>> points2, std::vector<int>& cluster2, std::vector<bool>& processed, KdTree* tree, float distanceTol )
//void clusterHelper(int indice, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol )
void ProcessPointClouds<PointT>::clusterHelper1(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol )
{
    /*
    std::vector<std::vector<float>> points2;
    for(int i = 0; i<cloud->points.size(); i++)
    {
        points2.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    } */

	processed[indice] = true;
	cluster.push_back(indice);

    std::cout << "Test Search Indice" <<  indice << " y valor process  "<< processed[indice] << std::endl;
    //Call to the KD-tree
	std::vector<int> nearest = tree->search({cloud->points[indice].x, cloud->points[indice].y, cloud->points[indice].z}, distanceTol);
	for(int index : nearest)
      std::cout << index << ",";
  	std::cout << std::endl;
    
    //std::vector<int> nearest = tree->search(points2[indice], distanceTol);
    //for(int id : nearest)
    //for(int id = 0; id < nearest.size(); id++)
	//for(int id = 0; id < nearest.size(); id++)
    for(int id : nearest)
	{
		// if(!processed[id])
            if(processed[id] == false ){
			clusterHelper1(id, cloud, cluster, processed, tree, distanceTol);
            }
	} 
} 


//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    
    std::vector<std::vector<float>> points1;
    //Converting
    for(int i = 0; i<cloud->points.size(); i++)
    {
        points1.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }
    
    KdTree* tree = new KdTree;
    //INSERT KD TREE 3D
    for (int i=0; i<cloud->points.size(); i++) 
        tree->insert(points1[i],i); 

    const std::vector<float> punto1 = points1[1];

   std::cout << "Pos X " << punto1[0];
   std::cout << " /Pos Y " << punto1[1];
   std::cout << " /Pos Z " << punto1[2];
   std::cout << " / AHORA DEL CLOUD " ;
   std::cout << "Pos X " << cloud->points[1].x;
   std::cout << " /" ;

    //PRUEBA DE 1 PUNTO PARA SEARCH
   	/*std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({-17.6, 3.43, -1.6},3.0);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;
    */ 
   //===============================================================

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //Creating a KDTree

    /*
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        
        clustersOut.push_back(cloudCluster);
    }  */


    //Euclidean por Vale
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersOut;
    //typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(cloud->points.size(), false);
	int i = 0;
	while(i < cloud->points.size())
	{
		//if(processed[i])
        if(processed[i] == true)
		{
			i++;
			continue;
		}

        std::vector<int> cluster;
		clusterHelper1(i, cloud, cluster, processed, tree, clusterTolerance);
		clusters.push_back(cluster);
		i++;
	} 

    std::cout << " CLOUD POINT SIZE " << cloud->points.size();

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    for(int i = 0; i<clusters.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int j = 0; j<clusters[i].size(); j++)
        cloudCluster->points.push_back(cloud->points[clusters[i][j]]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        
        clustersOut.push_back(cloudCluster);
    } 
    std::cout << " CLUSTER SIZE " << clusters.size();
    return clustersOut;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}