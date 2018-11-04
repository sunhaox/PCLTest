
#include <pcl\visualization\cloud_viewer.h>
#include <iostream>
#include <pcl\io\io.h>
#include <pcl\io\pcd_io.h>
#include <opencv2\opencv.hpp>
#include <fstream>

//内参参数
#define FX 210.88
#define FY 211.12
#define CX 168.340
#define CY 138.013
//畸变矫正参数
#define K1 -0.369
#define K2 0.134

using namespace cv;
using namespace std;
using namespace pcl;

int user_data;

//PCL窗体启动时调用的函数
//输入： 窗体类
//输出： 无
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	//viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;
}

//每帧循环调用函数
//输入： 窗体类
//输出： 无
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}



//读取CSV文件为Mat格式
//适用与箱体测量，320列，241行，第一行数据为0,1,...,319
//参数： 无
//返回： Mat格式图像
Mat csvToMat(void)
{
	fstream p_file;
	p_file.open("123.csv", ios::in);
	if (!p_file.is_open())
	{
		printf("文件不存在\r\n");
		exit(-1);
	}

	int pixelDep = 0;		//像素值
	int colCounter = 0;		//行计数
	int rowCounter = 0;		//列计数

	string line;

	getline(p_file, line);			//忽略第一行
	Mat img(240, 320, CV_16U);

	while (getline(p_file, line))
	{
		rowCounter = 0;
		char* tmp;
		tmp = strtok((char*)line.c_str(), ",");		//以逗号分割字符串
		while (tmp)
		{
			img.at<ushort>(colCounter, rowCounter) = (unsigned short)atoi(tmp);		//char转int
			rowCounter++;
			tmp = strtok(NULL, ",");
		}
		colCounter++;
	}

	return img.clone();
}

//畸变矫正
//输入： 待矫正的图片
//输出： 校正后的图片
Mat imageUndist(Mat src)
{
	//畸变矫正
	Mat img;

	//内参矩阵
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = FX;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = CX;
	cameraMatrix.at<double>(1, 1) = FY;
	cameraMatrix.at<double>(1, 2) = CY;
	cameraMatrix.at<double>(2, 2) = 1;
	//畸变参数
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = K1;
	distCoeffs.at<double>(1, 0) = K2;
	distCoeffs.at<double>(2, 0) = 0;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;

	Size imageSize = src.size();
	Mat map1, map2;

	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//
	remap(src, img, map1, map2, INTER_LINEAR);

	return img.clone();
}



//CSV文件转Pcd
//输入： 无
//输出： 无
void imageCsvToPcd()
{
	Mat mImageDepth = csvToMat();		//读取CSV文件转Mat
	Mat img;

	mImageDepth = imageUndist(mImageDepth);		//畸变矫正
	

	//16位压缩至8位
	Mat mImageZip;
	mImageDepth.convertTo(mImageZip, CV_8U, 1/255, 0);

	//生成伪彩色图像
	Mat mImageColor;
	applyColorMap(mImageZip, mImageColor, COLORMAP_HSV);

	//相机极坐标距离转世界坐标距离
	int imgWidth = mImageDepth.size().width;
	int imgHeight = mImageDepth.size().height;
	PointCloud<PointXYZRGB> pointCloud;

	for (int i = 0; i < imgHeight; i++)
	{
		for (int j = 0; j < imgWidth; j++)
		{
			float picDist = sqrt((i - imgHeight / 2.0)*(i - imgHeight / 2.0) + (j - imgWidth / 2.0)*(j - imgWidth / 2.0));	//图像上点到中心的像素点个数
			float picAngle = atan2(i - imgHeight / 2.0, j - imgWidth / 2.0);												//图像上x,y和中心点角度关系
			float angle = atan(sqrt((j - imgWidth / 2.0)*(j - imgWidth / 2.0) / FX / FX + (i - imgHeight / 2.0)*(i - imgHeight / 2.0) / FY / FY));
			float dist = mImageDepth.at<ushort>(i, j);				//原始图像深度

			PointXYZRGB p;
			p.z = dist*cos(angle);									//坐标变换后的深度
			p.x = dist*sin(angle)*cos(picAngle);
			p.y = dist*sin(angle)*sin(picAngle);

			p.r = mImageColor.at<Vec3b>(i, j)[0];
			p.g = mImageColor.at<Vec3b>(i, j)[1];
			p.b = mImageColor.at<Vec3b>(i, j)[2];
			pointCloud.points.push_back(p);
		}
	}

	//保存点云
	io::savePCDFileBinary("pcl.pcd", pointCloud);

}

int main()
{
	//读取CSV文件，转Pcd点云文件
	imageCsvToPcd();

	//点云类
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//从pcd读入点云数据
	pcl::io::loadPCDFile("pcl.pcd", *cloud);

	//初始化点云窗口
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//显示点云数据
	viewer.showCloud(cloud);

	//设置点云线程启动函数
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//设置点云线程每帧调用函数
	viewer.runOnVisualizationThread(viewerPsycho);

	while (!viewer.wasStopped())
	{
		//you can also do cool processing here
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}

	return 0;
}
