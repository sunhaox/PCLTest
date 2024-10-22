# PCLTest

## Description
PCL点云库的学习练习内容

## Software
Windows 10  
Visual Studio 2013  
PCL 1.8.0  
OpenCV 2.4.13

## Contents
* pcd:  
供参考的PCD文件。

* ex1:  
读取CSV文件，根据参数畸变矫正，转世界坐标系后PCL显示结果。

* ex2:  
不同滤波方法的测试例子。  
showPCD：读取PCD文件然后对比显示。  
pathThroughFilter：范围过滤。对指定范围外的点云数据删除。  
voxelGridFilter：利用体素化网络方式降采样。  
statisticalOutlierRemovalFilter：基于距离统计去除异常噪声  
extractIndicedFilter：从点云数据中提取索引  
radiusOutlierRemovalFilter：基于邻居数量去除异常噪声  
conditionalRemovalFilter：基于指定条件去除异常噪声  

* ex3:  
不同特征提取测试例子。  
estimatingTheNormalsFeatures：PCA估计法向量  
integralImageNormalEstimationFeatures：积分图像计算有组织点云的法线估计  
pfhEstimationFeatures：PFH特征计算  
fpfhEstimationFeatures：FPFH特征计算  
narfEstimationFeatures: NARF特征计算  

* ex4:
配准算法测试例子  
ICPAlgorithms：利用ICP（迭代最近点）算法求取状态转移矩阵。  
NDTAlgorithms：利用NDT（正太分布变换）算法配准。  
SACIAAlgorithms：根据提取的FPFH特征，SAC-IA匹配  

* ex_etc:  
一些综合演示程序。  
pfh_demo：PFH特征提取例子。运行后按SHIFT+左键选点显示PFH直方图。  
fpfh_demo：FPFH特征提取例子。运行后按SHIFT+左键选点显示FPFH直方图。  
6DOFPoseEstimation：利用VFH特征，根据训练样本，输出和测试样本一类的物体。  
narfFeatureExtraction：NARF特征提取例子。  
ICPRegisterPairsOfClouds：ICP配准例子。动态展示每次迭代内容，理解ICP迭代过程。  
  
* test：
还在测试中的程序。