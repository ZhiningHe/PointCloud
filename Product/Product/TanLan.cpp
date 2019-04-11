#include "MyPCL.h"
#include <pcl/io/vtk_io.h>
#include <pcl/surface/gp3.h>      //̰��ͶӰ���ǻ��㷨
using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
	// ��һ��XYZ�����͵�PCD�ļ��򿪲��洢��������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	if (pcl::io::loadPCDFile("yuanshi.pcd", cloud_blob) == -1)
	{
		PCL_ERROR("Could not read pcd file!\n");
		return 0;
	}
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);


	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //�洢���Ƶķ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //����kd��ָ��
	tree->setInputCloud(cloud);   //��cloud����tree����
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);	//������������
	n.setKSearch(20);
	n.compute(*normals);       //���Ʒ��ߴ洢������

	//�����ƺͷ��߷ŵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//��ʼ��GreedyProjectionTriangulation���󣬲����ò���
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//����������������ڴ洢���
	pcl::PolygonMesh triangles;

	//����GreedyProjectionTriangulation����Ĳ���
	  //��һ������Ӱ��ܴ�
	gp3.setSearchRadius(0.75);  //�������ӵ�֮��������룬���������������߳���
	gp3.setMu(2.5f); //��������ھ���ĳ��ӣ��Եõ�ÿ��������������뾶
	gp3.setMaximumNearestNeighbors(100); //��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees��pi�����ƽ���
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees ÿ�����ǵ���С�Ƕ�
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees ÿ�����ǵ����Ƕ�
	gp3.setNormalConsistency(false); //���������һ�£�����Ϊtrue

	//���������������������
	gp3.setInputCloud(cloud_with_normals);     //�����������Ϊ�������
	gp3.setSearchMethod(tree2);   //����������ʽ
	gp3.reconstruct(triangles);  //�ؽ���ȡ���ǻ������������triangles��

	// ���Ӷ�����Ϣ
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	//���������
	fstream fs;
	fs.open("statesID.txt", ios::out);
	if (!fs) return 0;
	fs << "��������Ϊ��" << states.size() << "\n";
	for (int i = 0; i < states.size(); i++)
	{
		if (states[i] != 0)
		{
			fs << states[i] << "\n";
			std::cout << states[i] << endl;
		}
	}


	//��������ͼ
//	pcl::io::savePLYFile("result.ply", triangles);
	pcl::io::saveVTKFile("mesh.vtk", triangles);
	 
	  // ��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 150); //���ñ���
	viewer->addPolygonMesh(triangles, "my"); //������ʾ������
	viewer->addCoordinateSystem(1.0); //��������ϵ
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