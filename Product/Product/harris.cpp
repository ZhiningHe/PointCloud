//
//Harris算法提取角点

//#include "opencv2/imgproc/imgproc.hpp"  
//#include "opencv2/highgui/highgui.hpp"  
//#include <iostream>  
//#include <cmath>
//using namespace cv;
//using namespace std;
//
//void ConvertRGB2GRAY(const Mat &image, Mat &imageGray);
//void SobelGradDirction(Mat &imageSource, Mat &imageSobelX, Mat &imageSobelY);
//void SobelXX(const Mat imageGradX, Mat_<float> &SobelAmpXX);
//void SobelYY(const Mat imageGradY, Mat_<float> &SobelAmpYY);
//void SobelXY(const Mat imageGradX, const Mat imageGradY, Mat_<float> &SobelAmpXY);
//double *getOneGuassionArray(int size, double sigma);
//void MyGaussianBlur(Mat_<float> &srcImage, Mat_<float> &dst, int size);
//void harrisResponse(Mat_<float> &GaussXX, Mat_<float> &GaussYY, Mat_<float> &GaussXY, Mat_<float> &resultData, float k);
//void LocalMaxValue(Mat_<float> &resultData, Mat &srcGray, Mat &ResultImage, int kSize);
//
//int main()
//{
//	const Mat srcImage = imread("a.jpg");
//	if (!srcImage.data)
//	{
//		printf("could not load image...\n");
//		return -1;
//	}
//	imshow("srcImage", srcImage);	//显示原图像
//	Mat srcGray;
//	ConvertRGB2GRAY(srcImage, srcGray);//把图像转化为灰度图像
//
//	Mat imageSobelX;
//	Mat imageSobelY;
//	Mat resultImage;
//	Mat_<float> imageSobelXX;
//	Mat_<float> imageSobelYY;
//	Mat_<float> imageSobelXY;
//	Mat_<float> GaussianXX;
//	Mat_<float> GaussianYY;
//	Mat_<float> GaussianXY;
//	Mat_<float> HarrisRespond;
//
//	SobelGradDirction(srcGray, imageSobelX, imageSobelY);//计算灰度图像的Soble的XY梯度
//	SobelXX(imageSobelX, imageSobelXX);//计算X方向的梯度的平方
//	SobelYY(imageSobelY, imageSobelYY);//计算Y方向的梯度的平方
//	SobelXY(imageSobelX, imageSobelY, imageSobelXY);//计算对角方向的梯度的平方
//	//计算高斯模糊XX YY XY
//	MyGaussianBlur(imageSobelXX, GaussianXX, 3);
//	MyGaussianBlur(imageSobelYY, GaussianYY, 3);
//	MyGaussianBlur(imageSobelXY, GaussianXY, 3);
//	harrisResponse(GaussianXX, GaussianYY, GaussianXY, HarrisRespond, 0.05);	//响应函数
//	LocalMaxValue(HarrisRespond, srcGray, resultImage, 3);//非极大值抑制
//	//显示Soble的XY梯度和点提取结果
//	imshow("imageSobelX", imageSobelX);
//	imshow("imageSobelY", imageSobelY);
//	imshow("resultImage", resultImage);
//	waitKey(0);
//	return 0;
//}
//
///*
//RGB转换成灰度图像的一个常用公式是：
//Gray = R*0.299 + G*0.587 + B*0.114
//*/
////******************灰度转换函数*************************  
//void ConvertRGB2GRAY(const Mat &image, Mat &imageGray)
//{
//	if (!image.data || image.channels() != 3)
//	{
//		return;
//	}
//	//创建一张单通道的灰度图像
//	imageGray = Mat::zeros(image.size(), CV_8UC1);
//	//取出存储图像像素的数组的指针，用于写入和读取
//	uchar *pointImage = image.data;
//	uchar *pointImageGray = imageGray.data;
//	//取出图像每行所占的字节数，一维数组
//	size_t stepImage = image.step;
//	size_t stepImageGray = imageGray.step;
//	for (int i = 0; i < imageGray.rows; i++)
//	{
//		for (int j = 0; j < imageGray.cols; j++)
//		{
//			//Gray = 0.2989*R + 0.5870*G + 0.1140*B
//			pointImageGray[i*stepImageGray + j] = (uchar)(0.114*pointImage[i*stepImage + 3 * j] + 0.587*pointImage[i*stepImage + 3 * j + 1] + 0.299*pointImage[i*stepImage + 3 * j + 2]);
//		}
//	}
//}
//
//void SobelGradDirction(Mat &imageSource, Mat &imageSobelX, Mat &imageSobelY)
//{
//	//转换图片大小
//	imageSobelX = Mat::zeros(imageSource.size(), CV_32SC1);
//	imageSobelY = Mat::zeros(imageSource.size(), CV_32SC1);
//	//取出原图和X和Y梯度图的数组的首地址，用于遍历图片
//	uchar *P = imageSource.data;
//	uchar *PX = imageSobelX.data;
//	uchar *PY = imageSobelY.data;
//
//	//取出每行所占据的字节数，因为Mat中存放的是一维数组。
//	int step = imageSource.step;
//	int stepXY = imageSobelX.step;
//
//	int index = 0;//梯度方向角的索引
//	for (int i = 1; i < imageSource.rows - 1; ++i)
//	{
//		for (int j = 1; j < imageSource.cols - 1; ++j)
//		{
//			//通过指针遍历图像上每一个像素 ，计算每一个像素的梯度  
//			//-1 0 1
//			//-2 0 2
//			//-1 0 1
//			double gradX = P[(i + 1)*step + j - 1] + P[(i + 1)*step + j] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[(i - 1)*step + j] * 2 - P[(i - 1)*step + j + 1];
//			//写入Mat图像数组
//			PX[i*stepXY + j * (stepXY / step)] = abs(gradX);
//			//1 2 1
//			//0 0 0
//			//-1 -2 -1
//			double gradY = P[(i - 1)*step + j + 1] + P[i*step + j + 1] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[i*step + j - 1] * 2 - P[(i + 1)*step + j - 1];
//			PY[i*stepXY + j * (stepXY / step)] = abs(gradY);
//		}
//	}
//	//将梯度数组转换成8位无符号整型，opencv中函数
//	convertScaleAbs(imageSobelX, imageSobelX);
//	convertScaleAbs(imageSobelY, imageSobelY);
//}
//
//void SobelXX(const Mat imageGradX, Mat_<float> &SobelAmpXX)
//{
//	//给X方向的梯度图像转化大小方便显示
//	SobelAmpXX = Mat_<float>(imageGradX.size(), CV_32FC1);
//	//遍历X梯度图片
//	for (int i = 0; i < SobelAmpXX.rows; i++)
//	{
//		for (int j = 0; j < SobelAmpXX.cols; j++)
//		{
//			//得到平方后的图
//			SobelAmpXX.at<float>(i, j) = imageGradX.at<uchar>(i, j)*imageGradX.at<uchar>(i, j);
//		}
//	}
//}
//void SobelYY(const Mat imageGradY, Mat_<float> &SobelAmpYY)
//{
//	//给Y方向的梯度图像转化大小方便显示
//	SobelAmpYY = Mat_<float>(imageGradY.size(), CV_32FC1);
//	for (int i = 0; i < SobelAmpYY.rows; i++)
//	{
//		for (int j = 0; j < SobelAmpYY.cols; j++)
//		{
//			SobelAmpYY.at<float>(i, j) = imageGradY.at<uchar>(i, j)*imageGradY.at<uchar>(i, j);
//		}
//	}
//}
//void SobelXY(const Mat imageGradX, const Mat imageGradY, Mat_<float> &SobelAmpXY)
//{
//	SobelAmpXY = Mat_<float>(imageGradX.size(), CV_32FC1);
//	for (int i = 0; i < SobelAmpXY.rows; i++)
//	{
//		for (int j = 0; j < SobelAmpXY.cols; j++)
//		{
//			SobelAmpXY.at<float>(i, j) = imageGradX.at<uchar>(i, j)*imageGradY.at<uchar>(i, j);
//		}
//	}
//}
//
//
////****************计算一维高斯的权值数组*****************
////size是代表的卷积核的边长的大小
//double *getOneGuassionArray(int size, double sigma)
//{
//	double sum = 0.0;
//	//定义高斯核半径
//	int kerR = size / 2;
//	//建立一个size大小的动态一维数组
//	double *arr = new double[size];
//	cout << "高斯权值数组：" << endl;
//	for (int i = 0; i < size; i++)
//	{
//
//		// 高斯函数前的常数可以不用计算，会在归一化的过程中给消去
//		//高斯核函数公式
//		arr[i] = exp(-((i - kerR)*(i - kerR)) / (2 * sigma*sigma));
//		sum += arr[i];//将所有的值进行相加
//	}
//
//	//进行归一化	
//	for (int i = 0; i < size; i++)
//	{
//		arr[i] /= sum;
//		cout << arr[i] << endl;
//	}
//	return arr;
//}
//
////****************高斯滤波函数的实现*****************
////srcImage是代表的输入的灰度原图
////dst表示的是输出的图
////size表示的是窗口的边长的大小	
//void MyGaussianBlur(Mat_<float> &srcImage, Mat_<float> &dst, int size)
//{
//	//断言assert
//	CV_Assert(srcImage.channels() == 1 || srcImage.channels() == 3); // 只处理单通道或者三通道图像
//	int kerR = size / 2;
//	//拷贝一个灰度图，保证大小一致
//	dst = srcImage.clone();
//	double* arr;
//	arr = getOneGuassionArray(size, 1);//先求出高斯数组
//
//		//遍历图像 水平方向的卷积
//	for (int i = kerR; i < dst.rows - kerR; i++)
//	{
//		for (int j = kerR; j < dst.cols - kerR; j++)
//		{
//			float GuassionSum[3] = { 0 };
//			//滑窗搜索完成高斯核平滑
//			for (int k = -kerR; k <= kerR; k++)
//			{
//				GuassionSum[0] += arr[kerR + k] * dst.at<float>(i, j + k);//行不变，列变换，先做水平方向的卷积
//			}
//			for (int k = 0; k < 1; k++)
//			{
//				if (GuassionSum[k] < 0)
//					GuassionSum[k] = 0;
//				else if (GuassionSum[k] > 255)
//					GuassionSum[k] = 255;
//			}
//			dst.at<float>(i, j) = static_cast<float>(GuassionSum[0]);
//
//		}
//	}
//
//	//竖直方向
//	for (int i = kerR; i < dst.rows - kerR; i++)
//	{
//		for (int j = kerR; j < dst.cols - kerR; j++)
//		{
//			float GuassionSum[3] = { 0 };
//			//滑窗搜索完成高斯核平滑
//			for (int k = -kerR; k <= kerR; k++)
//				GuassionSum[0] += arr[kerR + k] * dst.at<float>(i + k, j);//行变，列不换，再做竖直方向的卷积
//			for (int k = 0; k < 1; k++)
//			{
//				if (GuassionSum[k] < 0)
//					GuassionSum[k] = 0;
//				else if (GuassionSum[k] > 255)
//					GuassionSum[k] = 255;
//			}
//			dst.at<float>(i, j) = static_cast<float>(GuassionSum[0]);
//		}
//	}
//	delete[] arr;
//}
//
////****计算自相关矩阵M的特征值和响应函数R = (A*B - C) - k*(A+B)^2******
//void harrisResponse(Mat_<float> &GaussXX, Mat_<float> &GaussYY, Mat_<float> &GaussXY, Mat_<float> &resultData, float k)
//{
//	//创建一张响应函数输出的矩阵
//	resultData = Mat_<float>(GaussXX.size(), CV_32FC1);
//	for (int i = 0; i < resultData.rows; i++)
//	{
//		for (int j = 0; j < resultData.cols; j++)
//		{
//			float a = GaussXX.at<float>(i, j);
//			float b = GaussYY.at<float>(i, j);
//			float c = GaussXY.at<float>(i, j);
//			//R 角点响应函数 R = (A*B - C) - k*(A+B)*（A+B）
//			resultData.at<float>(i, j) = a * b - c * c - k * (a + b)*(a + b);
//		}
//	}
//}
//
////***********非极大值抑制和满足阈值及某邻域内的局部极大值为角点***
//void LocalMaxValue(Mat_<float> &resultData, Mat &srcGray, Mat &ResultImage, int kSize)
//{
//	int Thr = 20000;
//	//r是窗口的一半
//	int r = kSize / 2;
//	//拷贝了一个新的原始图像
//	ResultImage = srcGray.clone();
//	cout << "本次非极大值抑制的阈值为" << Thr << endl;
//	for (int i = r; i < ResultImage.rows - r; i++)
//	{
//		for (int j = r; j < ResultImage.cols - r; j++)
//		{
//			//保证是窗口内极大值
//			if (resultData.at<float>(i, j) > resultData.at<float>(i - 1, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i - 1, j) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i - 1, j + 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i, j + 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j + 1))
//			{
//				//阈值
//				if ((int)resultData.at<float>(i, j) > Thr)
//				{
//					//在结果图上标注提取角点
//					circle(ResultImage, Point(i, j), 5, Scalar(0, 0, 255), 2, 8, 0);
//				}
//			}
//
//		}
//	}
//}
