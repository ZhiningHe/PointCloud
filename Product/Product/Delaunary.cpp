//#include <stdio.h>
//using namespace cv;
//using namespace std;
//
//cv::Rect rect(0, 0, 600, 600); // Our outer bounding box 矩形框
//cv::Subdiv2D subdiv(rect); // Create the initial subdivision 虚拟三角形
//cv::Point2f fp; //This is our point holder
//float point_list[1000] = { 0 };
//for (int i = 0; i < 1000; i++) {//在框中插入点
//	// However you want to set points
//	fp = point_list[i];
//	subdiv.insert(fp);
//}
//vector<cv::Vec6f> triangles;//计算三角形
//subdiv.getTriangleList(triangles);
//vector<vector<cv::Point2f> > facets;//获得voronoi图
//vector<cv::Point2f> centers;
//subdiv.getVoronoiFacetList(vector<int>(), facets, centers);
//
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	// 确定文件格式
	char tmpStr[100];
	//strcpy(tmpStr, argv[1]);
	strcpy(tmpStr, "a.jpg");
	char* pext = strrchr(tmpStr, '.');
	//std::string extply("ply");
	std::string extply("jpg");
	std::string extpcd("pcd");
	if (pext) {
		*pext = '\0';
		pext++;
	}
	std::string ext(pext);
	//如果不支持文件格式，退出程序
	if (!((ext == extply) || (ext == extpcd))) {
		std::cout << "文件格式不支持!" << std::endl;
		std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
		return(-1);
	}

	//根据文件格式选择输入方式
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
	if (ext == extply) {
		if (pcl::io::loadPLYFile(argv[1], *cloud) == -1) {
			PCL_ERROR("Could not read ply file!\n");
			return -1;
		}
	}
	else {
		if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
			PCL_ERROR("Could not read pcd file!\n");
			return -1;
		}
	}

	// 估计法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //计算法线，结果存储在normals中
	//* normals 不能同时包含点的法向量和表面的曲率

	//将点云和法线放到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//初始化GreedyProjectionTriangulation对象，并设置参数
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//创建多变形网格，用于存储结果
	pcl::PolygonMesh triangles;

	//设置GreedyProjectionTriangulation对象的参数
	  //第一个参数影响很大
	gp3.setSearchRadius(1.5f); //设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径【默认值 0】
	gp3.setMu(2.5f); //设置最近邻距离的乘子，以得到每个点的最终搜索半径【默认值 0】
	gp3.setMaximumNearestNeighbors(100); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees（pi）最大平面角
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees 每个三角的最小角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees 每个三角的最大角度
	gp3.setNormalConsistency(false); //如果法向量一致，设置为true

	//设置搜索方法和输入点云
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	//执行重构，结果保存在triangles中
	gp3.reconstruct(triangles);

	//保存网格图
	pcl::io::savePLYFile("result.ply", triangles);

	// Additional vertex information
	//std::vector<int> parts = gp3.getPartIDs();
	//std::vector<int> states = gp3.getPointStates();

	  // 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景
	viewer->addPolygonMesh(triangles, "my"); //设置显示的网格
	viewer->addCoordinateSystem(1.0); //设置坐标系
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	system("pause");
	return (0);
}