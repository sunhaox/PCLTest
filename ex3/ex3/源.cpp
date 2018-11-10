/*
* Copyright (c) 2018 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 
*
*/

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/pfh.h>

using namespace std;
using namespace pcl;

int user_data;

//PCA估计法向并显示
//输入：无
//输出：正常结束0
int estimatingTheNormalsFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
	
	//读取点云数据
	PCDReader reader;
	reader.read("pcl.pcd", *cloud);

	//创建空kd树，进行临近点集搜索
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());

	//创建法线估计类
	NormalEstimation<PointXYZ, Normal> ne;
	ne.setInputCloud(cloud);	//设置输入点云数据
	ne.setSearchMethod(tree);	//设置空间搜索对象的数指针
	ne.setRadiusSearch(100);	//设置搜索邻居半径
	ne.compute(*cloud_normals);	//计算法线
	
	//显示结果
	boost::shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0.7);																	//设置背景颜色
	visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(cloud, 0, 255, 0);					//设置点云颜色
	viewer->addPointCloud<PointXYZ>(cloud, single_color, "sample cloud");									//添加需要显示的点云数据
	viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");	//设置点大小
	//参数1：点云原始数据
	//参数2：法向信息
	//参数3：显示法向的点云间隔（15个点显示一次法向）
	//参数4：法向长度
	viewer->addPointCloudNormals<PointXYZ, Normal>(cloud, cloud_normals, 15, 300, "normals");				//添加需要显示的法向

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		user_data++;
	}

	return 0;
}

//积分图像计算有组织点云的法线估计
//输入：无
//输出：正常结束0
int integralImageNormalEstimationFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	//读入数据
	PCDReader reader;
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);

	//估计法向
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	IntegralImageNormalEstimation<PointXYZ, Normal> ne;
	//COVARIANCE_MATRIX模式创建9个积分图像，以根据其局部邻域的协方差矩阵计算特定点的法线。 
	//AVERAGE_3D_GRADIENT模式创建6个积分图像以计算水平和垂直3D渐变的平滑版本，并使用这两个渐变之间的交叉积来计算法线。 
	//AVERAGE_DEPTH_CHANGE模式仅创建单个积分图像，并根据平均深度变化计算法线。
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);	//设置估计方法：平均3D梯度
	ne.setMaxDepthChangeFactor(0.02f);						//计算物体边缘时的深度变化阈值
	ne.setNormalSmoothingSize(10.0f);						//用于平滑法线区域的大小
	ne.setInputCloud(cloud);								//设置输入点云数据
	ne.compute(*normals);									//计算法向

	//显示结果
	visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<PointXYZ, Normal>(cloud, normals);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	return 0;
}

//PFH（点特征直方图）描述
int pfhEstimationFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

	//读入数据
	PCDReader reader;
	reader.read("table_scene_mug_stereo_textured.pcd", *cloud);

	//创建PFH估计类
	PFHEstimation<PointXYZ, Normal, PFHSignature125> pfh;
	pfh.setInputCloud(cloud);
	pfh.setInputNormals(normals);
	

	return 0;
}

int main()
{
	//estimatingTheNormalsFeatures();			//PCA估计法向并显示
	integralImageNormalEstimationFeatures();	//积分图像计算有组织点云的法线估计

	return 0;
}