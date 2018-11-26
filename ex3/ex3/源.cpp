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

int main()
{
	//estimatingTheNormalsFeatures();			//PCA估计法向并显示
	//integralImageNormalEstimationFeatures();	//积分图像计算有组织点云的法线估计
	//pfhEstimationFeatures();					//PFH特征
	fpfhEstimationFeatures();					//FPFH特征

	system("pause");

	return 0;
}