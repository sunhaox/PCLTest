/*
* Copyright (c) 2018 HadenSun
* Contact: http://www.sunhx.cn
* E-mail: admin@sunhx.cn
*
* 注意：
*	 Allinone安装方式下，FLANN库不完整，需要修改FLANN/include/flann/io/hdf5.h中
*	 #include <hdf5.h> 改为
*	 #include <vtkhdf5\hdf5.h>
*
* 用法：
*	 CMD窗口进入程序文件夹 ./build_tree pcd文件目录
*
* 解释：
*	 我们假设训练对象已经作为单独的簇分离（参见Euclidean Cluster Extraction）
*	 一组已收集的数据集在data文件夹下
*	 抓取了一个目录结构，查看了我们找到的所有.PCD文件，测试了它们是否是VFH签名并将它们加载到内存中;
*	 将数据转换为FLANN格式并将其转储到磁盘;
*	 构建了一个kd-tree结构并将其转储到磁盘上。
*	 
*/


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

typedef std::pair<std::string, std::vector<float>> vfh_model;

/** \brief Loads an n-D histogram file as a VFH signature
* \param path the input file name
* \param vfh the resultant VFH model
*/
bool
loadHist(const boost::filesystem::path &path, vfh_model &vfh)
{
	int vfh_idx;
	// Load the file as a PCD
	try
	{
		pcl::PCLPointCloud2 cloud;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type; unsigned int idx;
		r.readHeader(path.string(), cloud, origin, orientation, version, type, idx);

		//判断是否读到vfh字段
		vfh_idx = pcl::getFieldIndex(cloud, "vfh");
		if (vfh_idx == -1)
			return (false);
		if ((int)cloud.width * cloud.height != 1)
			return (false);
	}
	catch (const pcl::InvalidConversionException&)
	{
		return (false);
	}

	// Treat the VFH signature as a single Point Cloud
	pcl::PointCloud<pcl::VFHSignature308> point;
	pcl::io::loadPCDFile(path.string(), point);
	vfh.second.resize(308);

	std::vector<pcl::PCLPointField> fields;
	pcl::getFieldIndex(point, "vfh", fields);

	for (size_t i = 0; i < fields[vfh_idx].count; ++i)
	{
		vfh.second[i] = point.points[0].histogram[i];
	}
	vfh.first = path.string();
	return (true);
}

/** \brief Load a set of VFH features that will act as the model (training data)
* \param base_dir the path of files/directory
* \param extension the file extension containing the VFH features
* \param models the resultant vector of histogram models
*/
void
loadFeatureModels(const boost::filesystem::path &base_dir, const std::string &extension,
std::vector<vfh_model> &models)
{
	if (!boost::filesystem::exists(base_dir) && !boost::filesystem::is_directory(base_dir))
		return;

	for (boost::filesystem::directory_iterator it(base_dir); it != boost::filesystem::directory_iterator(); ++it)
	{
		//如果是文件夹，递归调用自己
		if (boost::filesystem::is_directory(it->status()))
		{
			std::stringstream ss;
			ss << it->path();
			pcl::console::print_highlight("Loading %s (%lu models loaded so far).\n", ss.str().c_str(), (unsigned long)models.size());
			loadFeatureModels(it->path(), extension, models);
		}
		//如果是文件，读取直方图信息
		if (boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == extension)
		{
			vfh_model m;
			if (loadHist(base_dir / it->path().filename(), m))
				models.push_back(m);
		}
	}
}

int
main(int argc, char** argv)
{
	if (argc < 2)
	{
		PCL_ERROR("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
		return (-1);
	}

	std::string extension(".pcd");
	transform(extension.begin(), extension.end(), extension.begin(), (int(*)(int))tolower);

	std::string kdtree_idx_file_name = "kdtree.idx";
	std::string training_data_h5_file_name = "training_data.h5";
	std::string training_data_list_file_name = "training_data.list";

	std::vector<vfh_model> models;

	// Load the model histograms
	loadFeatureModels(argv[1], extension, models);
	pcl::console::print_highlight("Loaded %d VFH models. Creating training data %s/%s.\n",
		(int)models.size(), training_data_h5_file_name.c_str(), training_data_list_file_name.c_str());

	// Convert data into FLANN format
	flann::Matrix<float> data(new float[models.size() * models[0].second.size()], models.size(), models[0].second.size());

	for (size_t i = 0; i < data.rows; ++i)
	for (size_t j = 0; j < data.cols; ++j)
		data[i][j] = models[i].second[j];

	// Save data to disk (list of models)
	flann::save_to_file(data, training_data_h5_file_name, "training_data");
	std::ofstream fs;
	fs.open(training_data_list_file_name.c_str());
	for (size_t i = 0; i < models.size(); ++i)
		fs << models[i].first << "\n";
	fs.close();

	// Build the tree index and save it to disk
	pcl::console::print_error("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str(), (int)data.rows);
	// 使用LinearIndexParams卡方距离进行搜索
	flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams());
	// 使用KDtree进行搜索
	//flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
	// KDtree比LinearIndexParams更快，但是是近似的近邻结果
	index.buildIndex();
	index.save(kdtree_idx_file_name);
	delete[] data.ptr();

	return (0);
}