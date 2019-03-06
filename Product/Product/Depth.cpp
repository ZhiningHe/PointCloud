#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#include<Windows.h>
#include<Kinect.h>
#include<iostream>
#include<stdio.h>
#include <highgui.h>
#include <stdio.h>
#include <OpenNI.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
	using namespace std;
	using namespace pcl;
	using namespace openni;



//打开kinect设备
int Depth_Kinect() {
	//打开设备
		Status rc = OpenNI::initialize();
		Device device;
		rc = device.open(ANY_DEVICE);
		if (rc != STATUS_OK)
	{
			printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
			return 2;
	}
		//打开深度相机
		VideoStream depth;
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		//成功则创建深度指针
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}
	//开始获取深度数据流
	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	//每2000ms获取一次
	VideoFrameRef frame;
	while (1)
	{
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}
		//读取获取的深度数据并保存到frame
		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}
		//如果获取的数据像素不合格
		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected frame format\n");
			continue;
		}

		DepthPixel* pDepth = (DepthPixel*)frame.getData();
		//输出中间的像素
		int middleIndex = (frame.getHeight() + 1)*frame.getWidth() / 2;
		//输出获取时间和像素
		printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);


		pcl::PointCloud<pcl::PointXYZI> cloud;
		//设置点云的参数
		cloud.width = frame.getWidth();
		cloud.height = frame.getHeight();
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
			cloud.points[i].intensity = pDepth[i];
		}
		//保存为pcd
		pcl::io::savePCDFileASCII("depth_pcd.pcd", cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
		//点云显示前5个
		for (size_t i = 0; i < 5; ++i)
			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	}

	//停止获取
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}

