// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());

    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr box_filtered (new pcl::PointCloud<PointT> ());

    pcl::CropBox<PointT> crb(true);
    crb.setMin(minPoint);
    crb.setMax(maxPoint);
    crb.setInputCloud(cloud_filtered);
    crb.filter(*box_filtered);

    std::vector<int> indices;

    pcl::CropBox<PointT> top(true);
    top.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    top.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    top.setInputCloud(box_filtered);
    top.filter(indices); 

    pcl::PointIndices::Ptr inliners (new pcl::PointIndices);

    for( int point:indices)
    {
        inliners->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(box_filtered);
    extract.setIndices(inliners);
    extract.setNegative(true);
    extract.filter(*box_filtered);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return box_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (int index: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size()==0)
    {
        std::cout <<"Could not estimate the planer model for the given dataset" <<std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
// std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
		{
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
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


		std::vector<float> v1 {(x2-x1), (y2-y1), (z2-z1)};
		std::vector<float> v2 {(x3-x1), (y3-y1), (z3-z1)};
		float A, B ,C, D;
		A = v1[1]*v2[2] - v1[2]*v2[1];
		B = v1[2]*v2[0] - v1[0]*v2[2];
		C = v1[0]*v2[1] - v1[1]*v2[0];
		D = -(A*x1+B*y1+C*z1);
		
		// std::vector<float> cp {A, B, C, D};

		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
			{
				continue;
			}

			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float d = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);

			if(d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZI point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

}


// void clusterHelper(int i, std::vector<int>& cluster, const std::vector<std::vector<float>>& points, std::vector<bool>& processed ,KdTree* tree, float distanceTol)
template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int i, std::vector<int>& cluster, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& processed ,KdTree* tree, float distanceTol)
{
	processed[i] = true;
	cluster.push_back(i);

	// pcl::PointXYZI point = inputCloud->points[i];
	PointT point = cloud->points[i];
	std::vector<float> fpoint;
	fpoint.push_back(point.x);
	fpoint.push_back(point.y);
	fpoint.push_back(point.z);
	std::vector<int> nearest = tree->search(fpoint, distanceTol);

	for(int id:nearest)
	{
		if(!processed[id])
		{
			clusterHelper(id, cluster, cloud, processed, tree, distanceTol);
		}
	}
}
// std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
template <typename PointT>
typename std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false);

	int i= 0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, cluster, cloud, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KdClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float disTol, int min , int max)
{
    KdTree* tree = new KdTree;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  
    for (int i=0; i<cloud->points.size(); i++)
	{
		// std::cout<<"from cluster.cpp: x"<<points[i][0]<<" y:"<<points[i][1]<<std::endl;
		pcl::PointXYZI point = cloud->points[i];
		std::vector<float> fpoint;
		fpoint.push_back(point.x);
		fpoint.push_back(point.y);
		fpoint.push_back(point.z);
		// std::cout<< inputCloud->points[i]<<std::endl;
    	tree->insert(fpoint,i); 
	}
  	auto startTime = std::chrono::steady_clock::now();
	
  	std::vector<std::vector<int>> clustersIds = euclideanCluster(cloud, tree, disTol);

  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> clusterId : clustersIds)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: clusterId)
		{
			clusterCloud->points.push_back(cloud->points[indice]);
		}
		clusterCloud->width = clusterCloud->points.size(); 
		clusterCloud->height = 1;
		clusterCloud->is_dense = true;
        if((clusterCloud->width>=min) && (clusterCloud->width<=max))
        {
            clusters.push_back(clusterCloud);
        }
  		
  	}
      return clusters;

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