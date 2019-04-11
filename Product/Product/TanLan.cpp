#include "MyPCL.h"
#include <pcl/io/vtk_io.h>
#include <pcl/surface/gp3.h>      //贪婪投影三角化算法
using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
	// 将一个XYZ点类型的PCD文件打开并存储到对象中
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	if (pcl::io::loadPCDFile("yuanshi.pcd", cloud_blob) == -1)
	{
		PCL_ERROR("Could not read pcd file!\n");
		return 0;
	}
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
	tree->setInputCloud(cloud);   //用cloud构建tree对象
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);	//设置搜索方法
	n.setKSearch(20);
	n.compute(*normals);       //估计法线存储到其中

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
	gp3.setSearchRadius(0.75);  //设置连接点之间的最大距离，（即是三角形最大边长）
	gp3.setMu(2.5f); //设置最近邻距离的乘子，以得到每个点的最终搜索半径
	gp3.setMaximumNearestNeighbors(100); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees（pi）最大平面角
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees 每个三角的最小角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees 每个三角的最大角度
	gp3.setNormalConsistency(false); //如果法向量一致，设置为true

	//设置搜索方法和输入点云
	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化，结果保存在triangles中

	// 附加顶点信息
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	//输出并保存
	fstream fs;
	fs.open("statesID.txt", ios::out);
	if (!fs) return 0;
	fs << "点云数量为：" << states.size() << "\n";
	for (int i = 0; i < states.size(); i++)
	{
		if (states[i] != 0)
		{
			fs << states[i] << "\n";
			std::cout << states[i] << endl;
		}
	}


	//保存网格图
//	pcl::io::savePLYFile("result.ply", triangles);
	pcl::io::saveVTKFile("mesh.vtk", triangles);
	 
	  // 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 150); //设置背景
	viewer->addPolygonMesh(triangles, "my"); //设置显示的网格
	viewer->addCoordinateSystem(1.0); //设置坐标系
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return (0);
}


//int main(int argc, char** argv) {
//	string infile = "result.ply";
//
//	pcl::PolygonMesh mesh;
//	pcl::io::loadPolygonFile(infile, mesh);
//
//	std::cout << "Polygon: " << mesh.polygons.size() << std::endl;
//	std::cout << "cloud: " << mesh.cloud.width << std::endl;
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr
//		cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	fromPCLPointCloud2(mesh.cloud, *cloud);
//
//	std::cout << cloud->width << " " << cloud->height << std::endl;
//	std::cout << cloud->points.size() << endl;
//
//	int index = rand() % 10000;
//	std::cout << cloud->points[index].x << " " <<
//		cloud->points[index].y << " " << cloud->points[index].z << std::endl;
//	return 0;
//}