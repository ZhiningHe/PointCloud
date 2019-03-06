///*
//Depth min: 500  max: 4500
//Frame: 424  512
//Buffer size: 217088
//*/
//#include<Windows.h>
//#include<Kinect.h>
//#include<iostream>
//#include<stdio.h>
//#include <highgui.h>
//
//int _tmain(int argc, _TCHAR* argv[])
//{
//	printf("Hello, Wellcome to kinect world!\n");
//	IKinectSensor* pKinect = nullptr;
//	GetDefaultKinectSensor(&pKinect);
//	if ( !pKinect)
//	{
//		printf("Get Kinect failed!\n");
//		goto endstop;
//	}
//	pKinect->Open();
//	BOOLEAN bOpen = false;
//	// 一直等待直到Kinect打开完成
//	while (!bOpen)
//	{
//		pKinect->get_IsOpen(&bOpen);
//		Sleep(200);
//	}
//	IDepthFrameSource* depths = nullptr;
//	pKinect->get_DepthFrameSource(&depths); // 获取深度数据源
//	IDepthFrameReader* depthr = nullptr;
//	depths->OpenReader(&depthr); // 打开深度解析器
//	while (true)
//	{
//		IDepthFrame* depthf = nullptr;
//		depthr->AcquireLatestFrame(&depthf); // 获取最近的深度数据
//		if ( !depthf )
//		{
//			Sleep(200);
//			continue;
//		}
//		USHORT minds, maxds;
//		depthf->get_DepthMinReliableDistance(&minds); // 获取最近的有效距离，500
//		depthf->get_DepthMaxReliableDistance(&maxds); // 获取最远的有效距离，4500
//		printf("Depth min: %d  max: %d\n", minds, maxds);
//		IFrameDescription* frameDs = nullptr;
//		depthf->get_FrameDescription(&frameDs); // 获取深度信息的属性
//		int height, width;
//		frameDs->get_Height(&height);
//		frameDs->get_Width(&width);
//		printf("Frame: %d  %d\n", height, width);
//		UINT ncaps = 0;
//		UINT16* buff = nullptr;
//		depthf->AccessUnderlyingBuffer(&ncaps, &buff); // 获取深度数据的指针和大小
//		//depthf->CopyFrameDataToArray(...); // 讲深度数据Copy到制定的buffer中
//		
//		printf("Buffer size: %d\n", ncaps);
// 
//		depthf->Release();
//		frameDs->Release();
//		Sleep(200);
//	}
//	
// 
//	pKinect->Close();
//endstop:
//	system("pause");
//	return 0;
//}
//-----------------------------------------------------------------------------------------------------
//#include <stdio.h>
//#include <iostream>
//#include <Kinect.h>
//#include <windows.h>
//#include <highgui.h>
//#include <opencv/cv.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//using namespace cv;
//using namespace std;
//
//
//// 转换depth图像到cv::Mat
//Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight)
//{
//	Mat img(nHeight, nWidth, CV_8UC1);
//	uchar* p_mat = img.data;//指向头指针
//
//	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);//指向最后一个元素的指针
//
//	while (pBuffer < pBufferEnd)//16位最大值为65536
//	{
//		*p_mat++ = *pBuffer++ / 65536.0 * 256;
//	}
//	//free(p_mat);
//	return img;
//}
//
//int main()
//{
//	IKinectSensor*          m_pKinectSensor;
//	IDepthFrameReader*      m_pDepthFrameReader;
//	IFrameDescription* pFrameDescription = NULL;
//	IDepthFrameSource* pDepthFrameSource = NULL;
//
//	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);//获取默认kinect传感器
//	assert(hr >= 0);
//	printf("打开kinect传感器成功\n");
//
//	hr = m_pKinectSensor->Open();//打开传感器
//	assert(hr >= 0);
//	hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);//获得深度信息传感器
//	assert(hr >= 0);
//	hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);//打开深度信息帧读取器
//	assert(hr >= 0);
//	int i = 0;
//	while (1)
//	{
//		//循环直到获取到最近的一帧
//		IDepthFrame* pDepthFrame = NULL;
//		while (hr < 0 || pDepthFrame == NULL)
//			hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//由于有时候获取不到，因此循环获取最近的帧
//		assert(hr >= 0);
//		hr = pDepthFrame->get_FrameDescription(&pFrameDescription);//获取帧的像素信息（宽和高）
//		int depth_width, depth_height;
//		pFrameDescription->get_Width(&depth_width);
//		pFrameDescription->get_Height(&depth_height);
//		printf("width=%d height=%d\n", depth_width, depth_height);
//
//		USHORT nDepthMinReliableDistance = 0;//获取最大、最小深度距离信息
//		USHORT nDepthMaxReliableDistance = 0;
//		assert(hr >= 0);
//		hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
//		assert(hr >= 0);
//		hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
//
//		printf("%d  nDepthMinD=%d nDepthMaxD=%d\n", i++, nDepthMinReliableDistance, nDepthMaxReliableDistance);
//
//		UINT nBufferSize_depth = 0;
//		UINT16 *pBuffer_depth = NULL;
//		pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);//获取图像像素个数和指向图像的指针
//
//		//转换为MAT格式
//		Mat depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height);//转换为8位的mat
//		pDepthFrame->Release();
//		//打印数据
//		//cv::Mat Img;
//
//		IplImage *src;
//		src = &IplImage(depthImg_show);
//		for (int i = 0; i < depthImg_show.rows; i++)
//		{
//			for (int j = 0; j < depthImg_show.cols; j++)
//			{
//				double ImgPixelVal = cvGetReal2D(src, i, j);
//				//输出像素值
//				cout << ImgPixelVal << "  ";
//			}
//			cout << "\n" << endl;
//		}
//
//		equalizeHist(depthImg_show, depthImg_show);//均衡化，为了提高显示效果
//
//		imwrite("MyFirstKinectImg.jpg", depthImg_show);//保存图片
//		//用opencv显示
//
//		namedWindow("display");
//		imshow("display", depthImg_show);
//
//		if (27 == waitKey(3) || 'Q' == waitKey(3) || 'q' == waitKey(3))
//			return 0;
//
//	}
//}
//---------------------------------------------------------------------------------------------------------
//
//#include <stdio.h>
//#include <OpenNI.h>
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//
//#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
//
//using namespace openni;
//
//int main()
//{
//	Status rc = OpenNI::initialize();
//	if (rc != STATUS_OK)
//	{
//		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
//		return 1;
//	}
//	//打开设备
//	Device device;
//	rc = device.open(ANY_DEVICE);
//	if (rc != STATUS_OK)
//	{
//		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
//		return 2;
//	}
//	//获取深度相机
//	VideoStream depth;
//
//	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
//	{
//		rc = depth.create(device, SENSOR_DEPTH);
//		if (rc != STATUS_OK)
//		{
//			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
//			return 3;
//		}
//	}
//	//深度图像
//	rc = depth.start();
//	if (rc != STATUS_OK)
//	{
//		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
//		return 4;
//	}
//
//	VideoFrameRef frame;
//	while (1)
//	{
//		int changedStreamDummy;
//		VideoStream* pStream = &depth;
//		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
//		if (rc != STATUS_OK)
//		{
//			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
//			continue;
//		}
//
//		rc = depth.readFrame(&frame);
//		if (rc != STATUS_OK)
//		{
//			printf("Read failed!\n%s\n", OpenNI::getExtendedError());
//			continue;
//		}
//
//		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
//		{
//			printf("Unexpected frame format\n");
//			continue;
//		}
//
//		DepthPixel* pDepth = (DepthPixel*)frame.getData();
//
//		int middleIndex = (frame.getHeight() + 1)*frame.getWidth() / 2;
//
//		printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);
//	}
//
//		pcl::PointCloud<pcl::PointXYZI> cloud;
//
//		 //Fill in the cloud data
//		cloud.width = 5;
//		cloud.height = 1;
//		cloud.is_dense = false;
//		cloud.points.resize(cloud.width * cloud.height);
//
//		for (size_t i = 0; i < cloud.points.size(); ++i)
//		{
//			cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//			cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//			cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//		}
//		//保存为pcd
//		pcl::io::savePCDFileASCII("depth_pcd.pcd", cloud);
//		std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;
//		//点云显示
//		for (size_t i = 0; i < cloud.points.size(); ++i)
//			std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
//	}
//
//	//停止获取
//	depth.stop();
//	depth.destroy();
//	device.close();
//	OpenNI::shutdown();
//
//	return 0;
//}
