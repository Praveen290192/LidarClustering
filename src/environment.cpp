/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}
void projectCityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(cloud, 0.2, Eigen::Vector4f(-50,-5,-2,1), Eigen::Vector4f(25.0,7.0,5.0,1));       
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacCloud = pointProcessorI->RansacPlane(filteredCloud, 25, 0.2);
    renderPointCloud(viewer,RansacCloud.first,"outliers",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> kdClustering = pointProcessorI->KdClustering(RansacCloud.second,0.4, 10, 690);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: kdClustering )
    {
        renderPointCloud(viewer,cluster, "ObstCloud"+std::to_string(clusterId),colors[2]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);  
        ++ clusterId;
    }
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* PointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ProcessPointClouds<pcl::PointXYZI>* PointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = PointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = PointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-50,-5,-2,1), Eigen::Vector4f(25.0,7.0,5.0,1));
// filterCloud = pointProcessorI->FilterCloud(inputCloud, ? , Eigen::Vector4f (?, ?, ?, 1), Eigen::Vector4f ( ?, ?, ?, 1));
    renderPointCloud(viewer,filteredCloud,"filterCloud");
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = PointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustering = PointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 500);
     
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: clustering )
    {
        std::cout<<"clustering size";
        PointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster, "ObstCloud"+std::to_string(clusterId),colors[2]);
        Box box = PointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);  
        ++ clusterId;
    }

    // renderPointCloud(viewer,segmentCloud.first,"obstCloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,0));
    // renderPointCloud(viewer, filteredCloud, "inputCloud");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, input_scan);
    // renderPointCloud(viewer,inputCloud, "inputCloud");
    // TODO:: Create point processor
    //Creating point processor on heap
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //Creating point Processor on stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = SegmentPlane(inputCloud, 100, 0.2);
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud", Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.2, 0, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
    {
      
        std::cout<< "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster,"ObstCloud"+std::to_string(clusterId), colors[2]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);        
        ++clusterId;
        
    }
    renderPointCloud(viewer,segmentCloud.second, "PlaneCloud"); 
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    auto streamInterator = stream.begin();
    

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pointProcessorI->loadPcd((*streamInterator).string());
        // simpleHighway(viewer);
        // cityBlock(viewer,pointProcessorI, cloud);
        projectCityBlock(viewer, pointProcessorI, cloud);
  
        streamInterator ++;

        if(streamInterator == stream.end())
        {
            streamInterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}