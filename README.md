# PCLTest

## Description
PCL点云库的学习练习内容

## Software
Windows 10  
Visual Studio 2013  
PCL 1.8.0  
OpenCV 2.4.13

## Contents
* ex1:  
读取CSV文件，根据参数畸变矫正，转世界坐标系后PCL显示结果。

* ex2:  
不同滤波方法的测试例子。  
showPCD：读取PCD文件然后对比显示。  
pathThroughFilter：范围过滤。对指定范围外的点云数据删除。  
voxelGridFilter：利用体素化网络方式降采样。  
statisticalOutlierRemovalFilter：基于距离统计去除异常噪声  
radiusOutlierRemovalFilter：基于邻居数量去除异常噪声  
conditionalRemovalFilter：基于指定条件去除异常噪声  
