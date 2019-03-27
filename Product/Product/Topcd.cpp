//#include <pcl/io/openni2_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/time.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/common/transforms.h>
//#include <string>
//#include <iostream>
//#include <vector>
//
//using namespace pcl;
//using namespace std;
//
//boost::mutex cloud_mutex;
//pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_global;
//
//class SimpleOpenNIViewer
//{
//public:
//	SimpleOpenNIViewer() {}
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud)//实时打印
//	{
//		static unsigned count = 0;
//		static double last = pcl::getTime();
//		int n = 0;
//		if (++count == 30)
//		{
//			double now = pcl::getTime();
//			cout << "time is" << now << endl;
//			char filename[100];
//			sprintf(filename, "D:\\%d.pcd",n++);
//			pcl::io::savePCDFileASCII(filename, *cloud);
//			//隔一段时间存储一张pcd文件
//			count = 0;
//		}
//	}
//	//回调函数，回传收到的点云到run()函数里
//	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out, bool* new_cloud_available_flag)
//	{
//		//锁住，不允许此段时间内，点云数据被修改
//		cloud_mutex.lock();
//		*cloud_out = *cloud;
//		*new_cloud_available_flag = true;
//		cloud_mutex.unlock();
//	}
//	//kinect读到的点云是在viewer.showCloud里是颠倒的，这个函数就是把它再颠倒过
//	pcl::PointCloud<pcl::PointXYZI>::Ptr upsideDown(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in) {
//		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
//		Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
//		transform(1, 1) = -1;
//		transform(2, 2) = -1;
//		pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);
//		return cloud_transformed;
//	}
//	//突出中间的部分的点云
//	pcl::PointCloud<pcl::PointXYZI>::Ptr highlightMiddleArea(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in) {
//		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
//		pcl::PassThrough<pcl::PointXYZI> pt;
//		pcl::IndicesPtr reserve_indices(new std::vector <int>);
//
//		pcl::copyPointCloud(*cloud_in, *cloud_out);
//		pt.setInputCloud(cloud_in);
//		pt.setFilterFieldName("x");
//		//突出x方向上（-0.3,0.3）这个范围内的点云
//		pt.setFilterLimits(-0.3, 0.3);
//		pt.filter(*reserve_indices);
//		for (int i = 0; i < (*reserve_indices).size(); i++) {
//			cloud_out->points[(*reserve_indices)[i]].intensity = 10;
//		}
//		return cloud_out;
//	}
//	//代码的核心部分，整个流程都在这里
//	void run(boost::shared_ptr<pcl::visualization::CloudViewer> viewer)
//	{
//		bool new_cloud_available_flag = false;
//		pcl::Grabber* interface = new OpenNI2Grabber();//创建openni2采集对象(接口)
//		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
//		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
//		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_toshow(new pcl::PointCloud<pcl::PointXYZI>);
//
//		//定义回调函数
//		boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> f =
//			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1, cloud_out, &new_cloud_available_flag);
//		boost::signals2::connection = interface->registerCallback(f);//注册回调函数（开始数据流）
//		//start()之后，点云数据源源不断的传到cloud_out
//		interface->start();
//
//		while (!viewer->wasStopped())
//		{
//			if (new_cloud_available_flag)
//			{//如果有新的点云获取
//				cloud_mutex.lock();       //锁住，不允许此段时间内，点云数据被修改
//				cloud_tmp = upsideDown(cloud_out);//颠倒点云，为了可视化
//				cloud_mutex.unlock();
//				cloud_toshow = highlightMiddleArea(cloud_tmp);
//				viewer->showCloud(cloud_toshow);//可视化点云
//				cloud_global = cloud_tmp;       //传点云到keyboardEventOccured()里
//				new_cloud_available_flag = false;
//			}
//		}
//		interface->stop();
//	}
//};
//
////回调函数，当键盘有输入时，被调用
//void keyboardEventOccured(const pcl::visualization::KeyboardEvent &event, void *nothing) {
//	if (event.getKeySym() == "space" && event.keyDown()) {//当按下空格键时
//		cout << "Space is pressed => pointcloud saved as output.pcd" << endl;
//		pcl::io::savePCDFile("D://output.pcd", *cloud_global);
//	}
//}
//
//int main()
//{
//	boost::shared_ptr<pcl::visualization::CloudViewer> viewer2(new pcl::visualization::CloudViewer("a viewer"));
//	//绑定可视化窗口和键盘事件的函数
//	viewer2->registerKeyboardCallback(keyboardEventOccured, (void*)NULL);
//	SimpleOpenNIViewer v;
//	v.run(viewer2);
//	return 0;
//}
//
