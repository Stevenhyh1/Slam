/****************************
 * 题目：给定一个稠密的点云，结合前面的练习，对其进行如下操作：
 * 下采样和滤波、重采样平滑、法线计算，贪心投影网格化（请提供结果的截图）。
 *
* 本程序学习目标：
 * 熟悉PCL网格化流程。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.01
****************************/
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{

	// Load input file
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile("./data/fusedCloud.pcd", *cloud) == -1)
    {
        cout << "点云数据读取失败！" << endl;
    }

    std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

   	// ----------------------开始你的代码--------------------------//
	// 请参考之前文章中点云下采样，滤波、平滑等内容，以及PCL官网实现以下功能。代码不难。
	
	// 下采样
	pcl::VoxelGrid<PointT> downsampler;
	downsampler.setInputCloud(cloud);
	downsampler.setLeafSize(0.01f, 0.01f, 0.01f);
	downsampler.setDownsampleAllData(false);
	downsampler.filter(*cloud_downSampled);
	pcl::io::savePCDFileBinary("./cloud_downsampled.pcd", *cloud_downSampled);
	
	// 统计滤波
	pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;
	statisOutlierRemoval.setInputCloud(cloud_downSampled);
	statisOutlierRemoval.setMeanK(50);
	statisOutlierRemoval.setStddevMulThresh(1.0);
	statisOutlierRemoval.filter(*cloud_filtered);
	pcl::io::savePCDFileBinary("./cloud_filtered.pcd", *cloud_filtered);
	
	// 对点云重采样  
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::MovingLeastSquares<PointT, PointT> mls_method;
	mls_method.setInputCloud(cloud_filtered);
	mls_method.setPolynomialOrder(2);
	mls_method.setSearchRadius(0.1);
	mls_method.setPolynomialFit(false);
	mls_method.setComputeNormals(false);
	mls_method.setSearchMethod(tree);
	mls_method.process(*cloud_smoothed);
	pcl::io::savePCDFileBinary("./cloud_smoothed.pcd", *cloud_smoothed);


	// 法线估计
	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
	normalEstimation.setInputCloud(cloud_smoothed);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setKSearch(10);
	normalEstimation.compute(*cloud_normals);
	
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr point_with_normal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*cloud_smoothed, *cloud_normals, *point_with_normal);
	
	// 贪心投影三角化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(point_with_normal);
	pcl::PolygonMesh mesh;
	gp3.setInputCloud(point_with_normal);
	gp3.setSearchMethod(tree2);
	gp3.setSearchRadius(0.05);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);

	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setNormalConsistency(true);

	gp3.reconstruct(mesh);
	

	// ----------------------结束你的代码--------------------------//

    // 显示网格化结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  //
    viewer->addPolygonMesh(mesh, "mesh");  //
    while (!viewer->wasStopped())
    {
    	viewer->spinOnce(100);
    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 1;
}

