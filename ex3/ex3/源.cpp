/*
* Copyright (c) 2018 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* Description:
*	 estimatingTheNormalsFeatures：PCA估计法向量
*	 integralImageNormalEstimationFeatures：积分图像计算有组织点云的法线估计
*	 pfhEstimationFeatures：PFH特征计算
*	 fpfhEstimationFeatures：FPFH特征计算
*
*/

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>

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
//注意该方法只适用于有序点云
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
//点特征直方图(Point Feature Histograms)
//正如点特征表示法所示，表面法线和曲率估计是某个点周围的几何特征基本表示法。
//虽然计算非常快速容易，但是无法获得太多信息，因为它们只使用很少的
//几个参数值来近似表示一个点的k邻域的几何特征。然而大部分场景中包含许多特征点，
//这些特征点有相同的或者非常相近的特征值，因此采用点特征表示法，
//其直接结果就减少了全局的特征信息。
//每一点对，原有12个参数，6个坐标值，6个坐标姿态（基于法线）
//PHF计算没一点对的 相对坐标角度差值三个值和 坐标点之间的欧氏距离 d
//从12个参数减少到4个参数
//默认PFH的实现使用5个区间分类（例如：四个特征值中的每个都使用5个区间来统计），
//其中不包括距离（在上文中已经解释过了——但是如果有需要的话，
//也可以通过用户调用computePairFeatures方法来获得距离值），
//这样就组成了一个125浮点数元素的特征向量（15），
//其保存在一个pcl::PFHSignature125的点类型中。
//注意：计算PFH特征时间较长
//输入：无
//输出：正常结束0
int pfhEstimationFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);

	//PFH计算时间太长，加入提示
	cout << "开始读入数据" << endl;

	//读入数据
	PCDReader reader;
	reader.read("table_scene_lms400_downsampled.pcd", *cloud);
	cout << "读入数据完成" << endl;
	
	//计算法线
	cout << "计算法线" << endl;
	NormalEstimationOMP<PointXYZ, Normal> ne;		//利用OpenMP实现多线程计算法线
	search::KdTree<PointXYZ>::Ptr tree1(new search::KdTree<PointXYZ>());
	ne.setInputCloud(cloud);			//设置计算数据
	ne.setSearchMethod(tree1);			//设置搜索算法KDtree搜索
	ne.setRadiusSearch(0.03);			//设置搜索半径
	ne.setNumberOfThreads(4);			//设置计算线程数
	ne.compute(*normals);
	cout << "法线计算完成" << endl;

	//创建PFH估计类
	cout << "计算PFH特征" << endl;
	PFHEstimation<PointXYZ, Normal, PFHSignature125> pfh;
	search::KdTree<PointXYZ>::Ptr tree2(new search::KdTree<PointXYZ>());
	PointCloud<PFHSignature125>::Ptr pfhs(new PointCloud<PFHSignature125>());		//输出数据集
	pfh.setInputCloud(cloud);			//设置原始点云数据
	pfh.setInputNormals(normals);		//传入法线数据
	pfh.setSearchMethod(tree2);			//创建一个空KD树，传入PFH估计对象
	pfh.setRadiusSearch(0.05);			//5cm内所有邻居参与计算，注意大于法线估计时的半径
	pfh.compute(*pfhs);					//计算PFH特征
	cout << "PFH特征计算完成" << endl;

	//直方图可视化
	cout << "结果展示" << endl;
	visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*pfhs, "pfh",300);	//显示第300个点FPH直方图
	plotter.plot();
	
	//直方图可视化方法2
	//1.8版本中将visualization\include\pcl\visualization\common\ren_win_interact_map.h中的RenWinInteract的空构造函数给删除了。
	//这个使得在使用PCLHistogramVisualizer时，出现缺少RenWinInteract函数的链接错误。
	//来源：http://www.pclcn.org/bbs/forum.php?mod=viewthread&tid=1300&extra=page%3D1
	//visualization::PCLHistogramVisualizer view;//直方图可视化
	//view.setBackgroundColor(255, 0, 0);//背景红色
	//view.addFeatureHistogram<pcl::PFHSignature125>(*pfhs, "pfh", 100);
	//view.spinOnce();  //循环的次数

	return 0;
}

//FPFH特征
//phf点特征直方图 计算复杂度还是太高
//计算法线-- - 计算临近点对角度差值---- - 直方图--
//因此存在一个O(nk ^ 2) 的计算复杂性。
//k个点之间相互的点对 k×k条连接线
//快速点特征直方图FPFH（Fast Point Feature Histograms）把算法的计算复杂度降低到了O(nk) ，
//但是任然保留了PFH大部分的识别特性。
//查询点和周围k个点的连线 的4参数特征
//也就是1×k = k个线
//默认的FPFH实现使用11个统计子区间（例如：四个特征值中的每个都将它的参数区间分割为11个），
//特征直方图被分别计算然后合并得出了浮点值的一个33元素的特征向量，
//这些保存在一个pcl::FPFHSignature33点类型中。
//输入：无
//输出：正常结束0
int fpfhEstimationFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	//读入数据
	PCDReader reader;
	reader.read("table_scene_lms400_downsampled.pcd", *cloud);

	//计算法线
	NormalEstimationOMP<PointXYZ, Normal> ne;
	PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
	search::KdTree<PointXYZ>::Ptr tree1(new search::KdTree<PointXYZ>());
	ne.setInputCloud(cloud);			//传入原始点云数据
	ne.setSearchMethod(tree1);			//设置搜索算法
	ne.setRadiusSearch(0.03);			//设置搜索半径
	ne.setNumberOfThreads(4);			//设置计算线程
	ne.compute(*normal);

	//计算FPFH特征
	FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh;
	PointCloud<FPFHSignature33>::Ptr fpfhs(new PointCloud<FPFHSignature33>());
	search::KdTree<PointXYZ>::Ptr tree2(new search::KdTree<PointXYZ>());
	fpfh.setInputCloud(cloud);				//设置原始点云数据
	fpfh.setInputNormals(normal);			//设置法线数据
	fpfh.setSearchMethod(tree2);			//设置近邻搜索算法
	fpfh.setRadiusSearch(0.05);				//设置搜索半径，注意大于法线估计时的半径
	fpfh.setNumberOfThreads(4);				//设置线程数
	fpfh.compute(*fpfhs);					

	cout << "fpfh feature size:" << fpfhs->points.size() << endl;	//应该与点数相同

	//直方图可视化
	visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*fpfhs, 300);	//显示第300个点FPH直方图
	plotter.plot();

	//直方图可视化方法2
	//1.8版本中将visualization\include\pcl\visualization\common\ren_win_interact_map.h中的RenWinInteract的空构造函数给删除了。
	//这个使得在使用PCLHistogramVisualizer时，出现缺少RenWinInteract函数的链接错误。
	//来源：http://www.pclcn.org/bbs/forum.php?mod=viewthread&tid=1300&extra=page%3D1
	//visualization::PCLHistogramVisualizer view;//直方图可视化
	//view.setBackgroundColor(255, 0, 0);//背景红色
	//view.addFeatureHistogram<pcl::FPFHSignature33>(*pfhs, "fpfh", 100);
	//view.spinOnce();  //循环的次数

	return 0;
}

//VFH特征
//视点特征直方图VFH(Viewpoint Feature Histogram)描述子，
//它是一种新的特征表示形式，应用在点云聚类识别和六自由度位姿估计问题。
//我们做了以下两种计算来构造特征，以应用于目标识别问题和位姿估计：
//1.扩展FPFH，使其利用整个点云对象来进行计算估计，
//在计算FPFH时以物体中心点与物体表面其他所有点之间的点对作为计算单元。
//2.添加视点方向与每个点估计法线之间额外的统计信息，为了达到这个目的，
//我们的关键想法是在FPFH计算中将视点方向变量直接融入到相对法线角计算当中
//对扩展的FPFH分量来说，默认的VFH的实现使用45个子区间进行统计，而对于视点分量要使用128个子区间进行统计，
//这样VFH就由一共308个浮点数组成阵列。
//在PCL中利用pcl::VFHSignature308的点类型来存储表示。
//对于一个已知的点云数据集，只一个单一的VFH描述子，而合成的PFH/FPFH特征的数目和点云中的点数目相同。
//输入：无
//输出：正常结束0
int vfhEstimationFeatures()
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	//读取点云数据
	PCDReader reader;
	reader.read("1258107463333_cluster_0_nxyz.pcd", *cloud);

	//计算法线
	NormalEstimationOMP<PointXYZ, Normal> ne;
	PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
	search::KdTree<PointXYZ>::Ptr tree1(new search::KdTree<PointXYZ>());
	ne.setInputCloud(cloud);		//设置输入点云数据
	ne.setSearchMethod(tree1);		//设置搜索算法KD树
	ne.setRadiusSearch(0.03);		//设置搜索半径0.03
	ne.setNumberOfThreads(4);		//设置计算线程4
	ne.compute(*normal);			//计算法线

	//计算VFH
	VFHEstimation<PointXYZ, Normal, VFHSignature308> vfh;
	PointCloud<VFHSignature308>::Ptr vfhs(new PointCloud<VFHSignature308>());
	search::KdTree<PointXYZ>::Ptr tree2(new search::KdTree<PointXYZ>());
	vfh.setInputCloud(cloud);		//设置输入点云数据
	vfh.setInputNormals(normal);	//设置输入法线
	vfh.setSearchMethod(tree2);		//设置搜索算法KD数
	vfh.compute(*vfhs);				//计算VFH特征

	cout << "vfh feature size: " << vfhs->points.size() << endl;		//应该等于1

	//直方图可视化
	visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*vfhs, 300);
	plotter.plot();

	return 0;
}

//NARF特征
//输入：angular_resolution 角度分辨率，默认0.5
//输入：coordinate_frame 坐标系，默认相机坐标系
//输入：setUnseenToMaxRange 是否将所有不可见点看作最大距离，默认false
//输入：support_size 感兴趣点的尺寸（球面直径），默认0.2
//输入：roation_invariant 特征旋转不变特性，默认开
//输出：正常结束0
int narfDescriptor(float angular_resolution = 0.5f,
	RangeImage::CoordinateFrame coordinate_frame = RangeImage::CAMERA_FRAME,
	bool setUnseenToMaxRange = false,
	float support_size = 0.2f,
	bool rotation_invariant = true)
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& point_cloud = *cloud;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity()); //仿射变换
	PointCloud<PointWithViewpoint> far_ranges;						//带视角的点云

	
	//读取点云数据
	PCDReader reader;
	reader.read("office_scene.pcd", *cloud);

	//设置传感器姿势
	scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
		cloud->sensor_origin_[1],
		cloud->sensor_origin_[2])) *
		Eigen::Affine3f(cloud->sensor_orientation_);

	//读取远距离文件
	//reader.read("frame_00000_far_ranges.pcd", far_ranges);

	/*
	setUnseenToMaxRange = true;//将所有不可见的点 看作 最大距离
	cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
	for (float x = -0.5f; x <= 0.5f; x += 0.01f)
	{
		for (float y = -0.5f; y <= 0.5f; y += 0.01f)
		{
			PointXYZ point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
			point_cloud.points.push_back(point);//设置点云中点的坐标
		}
	}
	point_cloud.width = (int)point_cloud.points.size();
	point_cloud.height = 1;
	*/
	

	//从点云数据，创建深度图像
	//直接把三维点云投射成二维图像
	float noise_level = 0.0;	//容差率，因为1°X1°空间内可能不止一点，0表示去最近点的距离作为像素值，0.05表示最近点后5cm求平均
	float min_range = 0.0f;		//深度最小值，0表示取1°X1°空间内最远点
	int border_size = 1;		//图像周边点
	boost::shared_ptr<RangeImage> range_image_ptr(new RangeImage);
	RangeImage& range_image = *range_image_ptr;
	
	range_image.createFromPointCloud(point_cloud, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	//range_image.integrateFarRanges(far_ranges);		//整合远距离点云
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	//3D点云显示
	visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1);			//背景白色
	visualization::PointCloudColorHandlerCustom<PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range image");
	viewer.initCameraParameters();

	//显示深度图像（平面图）
	visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);

	//提取NARF关键点
	RangeImageBorderExtractor range_image_border_extractor;				//创建深度图像的边界提取器，用于提取NARF关键点
	NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);	//创建NARF对象
	narf_keypoint_detector.setRangeImage(&range_image);					//设置点云对应深度图
	narf_keypoint_detector.getParameters().support_size = support_size; //感兴趣点尺寸（球面的直径）

	PointCloud<int> keypoint_indices;	//用于存储关键点的索引 PointCloud<int>
	narf_keypoint_detector.compute(keypoint_indices);	//计算NARF
	cout << "找到关键点：" << keypoint_indices.points.size() << " key points." << endl;

	//3D显示关键点
	PointCloud<PointXYZ>::Ptr keypoints_ptr(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>& keypoints = *keypoints_ptr;
	keypoints.points.resize(keypoint_indices.points.size());		//初始化大小
	for (size_t i = 0; i < keypoint_indices.points.size(); i++)
	{
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
	}

	visualization::PointCloudColorHandlerCustom<PointXYZ> keypoints_color_handler(keypoints_ptr, 255, 0, 0);
	viewer.addPointCloud<PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");		//添加显示关键点
	viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	//提取NARF特征
	vector<int> keypoints_indices2;
	keypoints_indices2.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.size(); i++)
		keypoints_indices2[i] = keypoint_indices.points[i];				//narf关键点索引
	NarfDescriptor narf_descriptor(&range_image, &keypoints_indices2);	//narf特征描述子
	narf_descriptor.getParameters().support_size = support_size;
	narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
	PointCloud<Narf36> narf_descriptors;
	narf_descriptor.compute(narf_descriptors);
	cout << "Extracted " << narf_descriptors.size() << " descriptors for " << keypoint_indices.points.size() << " keypoints." << endl;


	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	return 0;

}

int main()
{
	//estimatingTheNormalsFeatures();			//PCA估计法向并显示
	//integralImageNormalEstimationFeatures();	//积分图像计算有组织点云的法线估计
	//关于PFH和FPFH特征，参考ex_etc里的pfh_demo和fpfh_demo更好理解
	//pfhEstimationFeatures();					//PFH特征
	//fpfhEstimationFeatures();					//FPFH特征
	//vfhEstimationFeatures();					//VFH特征
	narfDescriptor();							//NARF特征

	system("pause");

	return 0;
}