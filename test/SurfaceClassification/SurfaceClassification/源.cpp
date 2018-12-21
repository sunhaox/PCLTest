/*
* Copyright (c) 2018 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 FPFH特征计算实例。加载PCD文件后显示，按住shift鼠标点击点，显示该点FPFH直方图。
*
*/

#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;
pcl::visualization::PCLPlotter plotter;
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	plotter.clearPlots();
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.clear();
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

	int num = event.getPointIndex();
	plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", num);
	plotter.plot();
}

void main()
{
	std::string filename("table_scene_lms400_downsampled.pcd");
	//visualizer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

	if (pcl::io::loadPCDFile(filename, *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
		return;
	}
	std::cout << cloud->points.size() << std::endl;

	cloud_mutex.lock();// for not overwriting the point cloud

	//直通滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fillter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*cloud_fillter);
	cloud_fillter = cloud;

	//降采样
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filltered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_fillter);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filltered);

	//计算法线
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_filltered);
	ne.setSearchSurface(cloud_fillter);
	ne.setNumberOfThreads(4);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);

	//计算FPFH特征
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud(cloud_filltered);
	fpfh.setInputNormals(cloud_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_1(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh.setSearchMethod(tree_1);
	fpfh.setRadiusSearch(0.05);
	fpfh.setNumberOfThreads(4);
	fpfh.compute(*fpfhs);


	// Display pointcloud:
	viewer->addPointCloud(cloud_filltered, "bunny");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

	// Add point picking callback to viewer:
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

	// Spin until 'Q' is pressed:
	viewer->spin();
	std::cout << "done." << std::endl;

	cloud_mutex.unlock();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
