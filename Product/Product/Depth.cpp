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



//��kinect�豸
int Depth_Kinect() {
	//���豸
		Status rc = OpenNI::initialize();
		Device device;
		rc = device.open(ANY_DEVICE);
		if (rc != STATUS_OK)
	{
			printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
			return 2;
	}
		//��������
		VideoStream depth;
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		//�ɹ��򴴽����ָ��
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}
	//��ʼ��ȡ���������
	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	//ÿ2000ms��ȡһ��
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
		//��ȡ��ȡ��������ݲ����浽frame
		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}
		//�����ȡ���������ز��ϸ�
		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected frame format\n");
			continue;
		}

		DepthPixel* pDepth = (DepthPixel*)frame.getData();
		//����м������
		int middleIndex = (frame.getHeight() + 1)*frame.getWidth() / 2;
		//�����ȡʱ�������
		printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);


		pcl::PointCloud<pcl::PointXYZI> cloud;
		//���õ��ƵĲ���
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
		//����Ϊpcd
		pcl::io::savePCDFileASCII("depth_pcd.pcd", cloud);
		std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
		//������ʾǰ5��
		for (size_t i = 0; i < 5; ++i)
			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	}

	//ֹͣ��ȡ
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}

