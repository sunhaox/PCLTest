#include <iostream> //标准输入/输出
#include <boost/thread/thread.hpp> //多线程
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h> //深度图有关头文件
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/visualization/range_image_visualizer.h> //深度图可视化
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h> //命令行参数解析

typedef pcl::PointXYZ PointType;


//参数 全局
float angular_resolution_x = 0.5f, //角分辨率（单位弧度）
angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame1 = pcl::RangeImage::CAMERA_FRAME; //坐标帧（相机帧）
bool live_update = true; //是否根据选择的视角更新深度图像

// 打印帮助信息
void printUsage1(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-rx <float>  angular resolution in degrees (default " << angular_resolution_x << ")\n"
		<< "-ry <float>  angular resolution in degrees (default " << angular_resolution_y << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame1 << ")\n"
		<< "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
		<< "-h           this help\n"
		<< "\n\n";
}

/*
void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
look_at_vector[0], look_at_vector[1], look_at_vector[2],
up_vector[0], up_vector[1], up_vector[2]);
}
*/


// 主函数
int main1(int argc, char** argv)
{
	//解析命令行参数
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage1(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;
		std::cout << "Live update is on.\n";
	}
	if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
		std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
	if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
		std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
	int tmp_coordinate_frame1;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame1) >= 0)
	{
		coordinate_frame1 = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame1);
		std::cout << "Using coordinate frame " << (int)coordinate_frame1 << ".\n";
	}
	angular_resolution_x = pcl::deg2rad(angular_resolution_x);
	angular_resolution_y = pcl::deg2rad(angular_resolution_y);

	//读取pcd文件。如果没有指定文件，则创建样本云点
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
	if (!pcd_filename_indices.empty())
	{
		std::string filename = argv[pcd_filename_indices[0]];
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
		{
			std::cout << "Was not able to open file \"" << filename << "\".\n";
			printUsage1(argv[0]);
			return 0;
		}
		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
			point_cloud.sensor_origin_[1],
			point_cloud.sensor_origin_[2])) *
			Eigen::Affine3f(point_cloud.sensor_orientation_);
	}
	else
	{
		std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f)
		{
			for (float y = -0.5f; y <= 0.5f; y += 0.01f)
			{
				PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
				point_cloud.points.push_back(point);
			}
		}
		point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
	}

	//从点云创建出深度图
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage); //深度图指针
	pcl::RangeImage& range_image = *range_image_ptr;   //引用
	range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
		pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame1, noise_level, min_range, border_size); //从点云创建出深度图

	//打开一个3D图形窗口，并添加点云数据
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(1, 1, 1); //背景
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
	viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters();
	//setViewerPose(viewer, range_image.getTransformationToWorldSystem ()); //PCL 1.6 出错

	//以图像的形式显示深度图像，深度值作为颜色显示
	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image);


	//主循环
	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);

		if (live_update) //根据3D显示，实时更新2D图像
		{
			scene_sensor_pose = viewer.getViewerPose(); //获取观测姿势
			range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
				pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
				scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size); //重新生成新的深度图
			range_image_widget.showRangeImage(range_image); //重新显示
		}
	}
}
