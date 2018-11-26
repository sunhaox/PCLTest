/*
 * Copyright (c) 2018 HadenSun 
 * Contact: http://www.sunhx.cn
 * E-mail: admin@sunhx.cn
 *
 * Description:
 *	 主函数内分别调用不同函数，实现不同滤波效果。
 *   showPCD：读取PCD文件然后对比显示。
 *   pathThroughFilter：范围过滤。对指定范围外的点云数据删除。
 *	 voxelGridFilter：利用体素化网络方式降采样。
 *   statisticalOutlierRemovalFilter：基于距离统计去除异常噪声
 *   extractIndicedFilter：从点云数据中提取索引
 *	 radiusOutlierRemovalFilter：基于邻居数量去除异常噪声
 *	 conditionalRemovalFilter：基于指定条件去除异常噪声
 *
 */


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;

int user_data;

//读取PCD文件然后对比显示
//各滤波函数处理结果已经存入对应PCD文件，打开可对比查看结果
//输入：file1 第一个文件名
//输入：file2 第二个文件名
//输出： 无
void showPCD(string file1, string file2)
{
	PointCloud<PointXYZ>::Ptr cloud_1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_2(new PointCloud<PointXYZ>);

	//文件读取
	PCDReader reader;
	reader.read<PointXYZ>(file1, *cloud_1);
	reader.read<PointXYZ>(file2, *cloud_2);

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.setWindowName("结果显示");
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud(cloud_1, "before", v1);

	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addPointCloud(cloud_2, "after", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		user_data++;
	}
}


//范围过滤，在指定范围外的点会被剔除
//输入：无
//输出：正常结束0
int pathThroughFilter()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);

	//初始化点云数据
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	srand((int)time(0));		//初始化随机种子
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
	}

	//展示点云赋值结果
	cerr << "Cloud before filtering: " << endl;
	for (rsize_t i = 0; i < cloud->points.size(); ++i)
	{
		cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << endl;
	}

	//过滤对象
	PassThrough<PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");		//过滤字段设置为z坐标
	pass.setFilterLimits(0.0, 1.0);		//接收的间隔值设置为（0.0, 1.0）
	//pass.setFilterLimitsNegative(true);	//接收范围反转
	pass.filter(*cloud_filtered);

	//显示过滤结果
	cerr << "Cloud after filtering: " << endl;
	for (rsize_t i = 0; i < cloud_filtered->points.size(); i++)
	{
		cerr << " " << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << " " << endl;
	}

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);			//新建窗口，位置参数采用百分比
	viewer.setBackgroundColor(0, 0, 0, v1);				//设置窗口背景
	viewer.addPointCloud(cloud, "before", v1);			//向v1中添加点云数据

	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);		//新建窗口v2
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);		//设置v2背景
	viewer.addPointCloud(cloud_filtered, "after", v2);	//向v2中添加点云数据
	//viewer.spin();		//主进程暂停，3D界面不间断执行

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);		//3D界面运行100ms，返回主线程
		user_data++;
	}

	return 0;
}

//利用体素化网络方式降采样
//VoxelGrid类在输入点云数据上创建3D体素网格（将体素网格视为空间中的一组微小3D框）。
//然后，在每个体素（即3D框）中，所有存在的点将用它们的质心近似（即，下采样）。 
//这种方法比用体素的中心逼近它们要慢一些，但它更准确地代表了下面的表面。
//输入：无
//输出：正常结束0
int voxelGridFilter()
{
	PCLPointCloud2::Ptr cloud(new PCLPointCloud2());
	PCLPointCloud2::Ptr cloud_filtered(new PCLPointCloud2());
	PointCloud<PointXYZ>::Ptr cloud_xyz(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_xyz_filtered(new PointCloud<PointXYZ>);

	//填充数据
	PCDReader reader;
	reader.read("table_scene_lms400.pcd", *cloud);		//读取数据
	cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << getFieldsList(*cloud) << ")." << endl;

	//创建过滤对象
	VoxelGrid<PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);		//设置过滤器叶大小1cm 1cm 1cm
	sor.filter(*cloud_filtered);

	cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << getFieldsList(*cloud_filtered) << ")." << endl;

	//过滤结果写入pcd文件
	PCDWriter writer;
	writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	//点云格式转换
	fromPCLPointCloud2(*cloud, *cloud_xyz);
	fromPCLPointCloud2(*cloud_filtered, *cloud_xyz_filtered);

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.setWindowName("结果显示");
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud(cloud_xyz, "before",v1);
	
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addPointCloud(cloud_xyz_filtered, "after",v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		user_data++;
	}
	
	return 0;
}

//基于距离统计去除异常噪声
//我们的稀疏异常值去除基于输入数据集中点到邻居距离的分布的计算。对于每个点，我们计算从它到所有邻居的平均距离。
//通过假设所得到的分布是具有平均值和标准偏差的高斯分布。
//所有平均距离在由全局距离平均值和标准偏差定义的区间之外的点可以被认为是异常值并且从数据集中修剪。
//输入：无
//输出：正常结果0
int statisticalOutlierRemovalFilter()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);

	//填充数据
	PCDReader reader;
	reader.read<PointXYZ>("table_scene_lms400.pcd", *cloud);	

	cerr << "Cloud before filtering: " << std::endl;
	cerr << *cloud << endl;

	//创建滤波器对象
	StatisticalOutlierRemoval<PointXYZ> sor;
	//该算法迭代整个输入两次：在第一次迭代期间，它将计算每个点与其最近的k个邻居之间的平均距离。 可以使用setMeanK（）设置k的值。
	//接下来，计算所有这些距离的平均值和标准偏差，以便确定距离阈值。
	//距离阈值将等于：mean + stddev_mult * stddev。 可以使用setStddevMulThresh（）设置标准偏差的乘数。
	//在下一次迭代期间，如果它们的平均相邻距离分别低于或高于该阈值，则点将被分类为内部或异常值。
	//为每个查询点找到的邻居将在setInputCloud（）的所有点中找到，而不仅仅是由setIndices（）索引的那些点。 
	//setIndices（）方法仅索引将作为搜索查询点迭代的点。
	sor.setInputCloud(cloud);
	sor.setMeanK(50);					//设置与50个邻居计算平均距离
	sor.setStddevMulThresh(1.0);		//设置标准差放大系数
	sor.filter(*cloud_filtered);

	cerr << "Cloud after filtering: " << endl;
	cerr << *cloud_filtered << endl;

	//写入PCD文件
	PCDWriter writer;
	writer.write<PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);	

	//参数取反，提取噪声
	//sor.setNegative(true);
	//sor.filter(*cloud_filtered);
	//writer.write<PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.setWindowName("结果显示");
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud(cloud, "before", v1);

	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addPointCloud(cloud_filtered, "after", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		user_data++;
	}

	return 0;
}

//从点云数据中提取索引
//使用了体素化滤波后的降采样结果
//利用SACSegmentation分割点云，利用ExtractIndices索引提取分割的点云。
//输入：无
//输出：正常结束0
int extractIndicedFilter()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_p(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_f(new PointCloud<PointXYZ>);

	//读取点云数据
	PCDReader reader;
	PCDWriter writer;
	reader.read<PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud);

	ModelCoefficients::Ptr coefficients(new ModelCoefficients());		//结构体，存储拟合出的模型系数。这里拟合平面，数学定义ax+by+cz+d=0即一个平面，这里存储a,b,c,d
	PointIndices::Ptr inliers(new PointIndices());						//结构体，存储拟合出的符合模型的点索引
	//创建分割对象
	SACSegmentation<PointXYZ> seg;
	//可选
	seg.setOptimizeCoefficients(true);
	//必选
	seg.setModelType(SACMODEL_PLANE);		//设置模型类型，平面/线/圆2D3D/球/圆柱/椎体/平行线等可选
	seg.setMethodType(SAC_RANSAC);			//设置方法类型SAC_RANSAC（随机抽样一致）
	seg.setMaxIterations(1000);				//设置最大迭代次数
	seg.setDistanceThreshold(0.01);			//到模型距离的阈值

	//创建滤波对象
	ExtractIndices<PointXYZ> extract;

	int i = 0;
	int nr_points = (int)cloud->points.size();

	//30% 的原始点还在则继续提取
	while (cloud->points.size() > 0.3*nr_points)
	{
		//从剩余的云中分割出最大的平面组件
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);			//拟合出的平面的点索引和平面参数分别存入inliers和coefficients
		if (inliers->indices.size() == 0)
		{
			cerr << "Could not estimate a planar model for the given dataset. " << endl;
			break;
		}

		//提取索引
		extract.setInputCloud(cloud);		//设置原始点云数据
		extract.setIndices(inliers);		//设置点云索引
		extract.setNegative(false);
		extract.filter(*cloud_p);			//提取点云
		cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl;

		stringstream ss;
		ss << "table_scene_lms400_plane_" << i << ".pcd";
		writer.write<PointXYZ>(ss.str(), *cloud_p, false);		//存储分割后的点云数据

		//提取剩下的点
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud.swap(cloud_f);
		i++;
	}

	return 0;
}

//基于邻居数量过滤噪声
//输入：无
//输出：正常结果0
int radiusOutlierRemovalFilter()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);

	//填入数据
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->height * cloud->width);

	srand((int)time(0));		//初始化随机种子
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
	}

	//打印输入
	cerr << "Cloud before filtering: " << endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}

	//构建过滤器
	RadiusOutlierRemoval<PointXYZ> outrem;
	//迭代整个输入一次，并且对于每个点，检索特定半径内的邻居的数量。
	//如果邻居太少，则该点将被视为异常值，由setMinNeighborsInRadius（）确定。 
	//可以使用setRadiusSearch（）更改半径。
	//为每个查询点找到的邻居将在setInputCloud（）的所有点中找到，而不仅仅是由setIndices（）索引的那些点。
	//setIndices（）方法仅索引将作为搜索查询点迭代的点。
	outrem.setInputCloud(cloud);			//设置输入点云数据
	outrem.setRadiusSearch(0.8);			//设置搜索半径0.8
	outrem.setMinNeighborsInRadius(2);		//设置邻居阈值2
	outrem.filter(*cloud_filtered);

	//打印结果
	cerr << "Cloud after filtering: " << endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
	{
		cerr << " " << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << endl;
	}

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.setWindowName("结果显示");
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud(cloud, "before", v1);

	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addPointCloud(cloud_filtered, "after", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		user_data++;
	}

	return 0;
}

//基于给定的条件过滤噪声
//输入：无
//输出：正常结果0
int conditionalRemovalFilter()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);

	//填入数据
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->height * cloud->width);

	srand((int)time(0));		//初始化随机种子
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].y = rand() / (RAND_MAX + 1.0f) - 0.5;
		cloud->points[i].z = rand() / (RAND_MAX + 1.0f) - 0.5;
	}

	//打印输入
	cerr << "Cloud before filtering: " << endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cerr << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
	}

	//构建条件
	ConditionAnd<PointXYZ>::Ptr range_cond(new ConditionAnd<PointXYZ>());
	range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("z", ComparisonOps::GT, 0.0)));		//greater than(GT)大于0.0
	range_cond->addComparison(FieldComparison<PointXYZ>::ConstPtr(new FieldComparison<PointXYZ>("z", ComparisonOps::LT, 0.8)));		//less than(LT)小于0.8

	//构建过滤器
	ConditionalRemoval<PointXYZ> condrem;
	//ConditionalRemoval过滤满足特定条件的数据。
	//必须为ConditionalRemoval提供条件。 
	//有两种类型的条件：ConditionAnd和ConditionOr。 
	//条件需要一个或多个比较和 / 或其他条件。 比较具有名称，比较运算符和值。
	//根据派生的比较类型，名称可以对应于PointCloud字段名称，或rgb颜色空间或hsi颜色空间中的颜色分量。
	condrem.setCondition(range_cond);
	condrem.setInputCloud(cloud);
	condrem.setKeepOrganized(true);
	condrem.filter(*cloud_filtered);

	//打印结果
	cerr << "Cloud after filtering: " << endl;
	for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
	{
		cerr << " " << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << endl;
	}

	//显示结果对比
	int v1(0), v2(0);
	visualization::PCLVisualizer viewer;
	viewer.setWindowName("结果显示");
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addPointCloud(cloud, "before", v1);

	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer.addPointCloud(cloud_filtered, "after", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		user_data++;
	}

	return 0;
}

//主函数
//各方法效果的调用
int main(int argc, char** argv)
{
	//pathThroughFilter();					//指定范围外剔除
	//voxelGridFilter();					//下采样，减少点个数
	statisticalOutlierRemovalFilter();	//基于距离统计滤波，滤除噪声
	//radiusOutlierRemovalFilter();			//基于范围内邻居数滤波，滤除噪声
	//conditionalRemovalFilter();			//基于给定的条件过滤噪声
	//extractIndicedFilter();				//从点云数据中提取索引


	//各方法处理的结果都保存在各自的PCD文件，可以直接打开查看
	//showPCD("table_scene_lms400_plane_1.pcd", "table_scene_lms400_plane_0.pcd");

	system("pause");

	return 0;
}