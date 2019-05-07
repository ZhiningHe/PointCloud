//
//Harris�㷨��ȡ�ǵ�

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
//	imshow("srcImage", srcImage);	//��ʾԭͼ��
//	Mat srcGray;
//	ConvertRGB2GRAY(srcImage, srcGray);//��ͼ��ת��Ϊ�Ҷ�ͼ��
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
//	SobelGradDirction(srcGray, imageSobelX, imageSobelY);//����Ҷ�ͼ���Soble��XY�ݶ�
//	SobelXX(imageSobelX, imageSobelXX);//����X������ݶȵ�ƽ��
//	SobelYY(imageSobelY, imageSobelYY);//����Y������ݶȵ�ƽ��
//	SobelXY(imageSobelX, imageSobelY, imageSobelXY);//����ԽǷ�����ݶȵ�ƽ��
//	//�����˹ģ��XX YY XY
//	MyGaussianBlur(imageSobelXX, GaussianXX, 3);
//	MyGaussianBlur(imageSobelYY, GaussianYY, 3);
//	MyGaussianBlur(imageSobelXY, GaussianXY, 3);
//	harrisResponse(GaussianXX, GaussianYY, GaussianXY, HarrisRespond, 0.05);	//��Ӧ����
//	LocalMaxValue(HarrisRespond, srcGray, resultImage, 3);//�Ǽ���ֵ����
//	//��ʾSoble��XY�ݶȺ͵���ȡ���
//	imshow("imageSobelX", imageSobelX);
//	imshow("imageSobelY", imageSobelY);
//	imshow("resultImage", resultImage);
//	waitKey(0);
//	return 0;
//}
//
///*
//RGBת���ɻҶ�ͼ���һ�����ù�ʽ�ǣ�
//Gray = R*0.299 + G*0.587 + B*0.114
//*/
////******************�Ҷ�ת������*************************  
//void ConvertRGB2GRAY(const Mat &image, Mat &imageGray)
//{
//	if (!image.data || image.channels() != 3)
//	{
//		return;
//	}
//	//����һ�ŵ�ͨ���ĻҶ�ͼ��
//	imageGray = Mat::zeros(image.size(), CV_8UC1);
//	//ȡ���洢ͼ�����ص������ָ�룬����д��Ͷ�ȡ
//	uchar *pointImage = image.data;
//	uchar *pointImageGray = imageGray.data;
//	//ȡ��ͼ��ÿ����ռ���ֽ�����һά����
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
//	//ת��ͼƬ��С
//	imageSobelX = Mat::zeros(imageSource.size(), CV_32SC1);
//	imageSobelY = Mat::zeros(imageSource.size(), CV_32SC1);
//	//ȡ��ԭͼ��X��Y�ݶ�ͼ��������׵�ַ�����ڱ���ͼƬ
//	uchar *P = imageSource.data;
//	uchar *PX = imageSobelX.data;
//	uchar *PY = imageSobelY.data;
//
//	//ȡ��ÿ����ռ�ݵ��ֽ�������ΪMat�д�ŵ���һά���顣
//	int step = imageSource.step;
//	int stepXY = imageSobelX.step;
//
//	int index = 0;//�ݶȷ���ǵ�����
//	for (int i = 1; i < imageSource.rows - 1; ++i)
//	{
//		for (int j = 1; j < imageSource.cols - 1; ++j)
//		{
//			//ͨ��ָ�����ͼ����ÿһ������ ������ÿһ�����ص��ݶ�  
//			//-1 0 1
//			//-2 0 2
//			//-1 0 1
//			double gradX = P[(i + 1)*step + j - 1] + P[(i + 1)*step + j] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[(i - 1)*step + j] * 2 - P[(i - 1)*step + j + 1];
//			//д��Matͼ������
//			PX[i*stepXY + j * (stepXY / step)] = abs(gradX);
//			//1 2 1
//			//0 0 0
//			//-1 -2 -1
//			double gradY = P[(i - 1)*step + j + 1] + P[i*step + j + 1] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[i*step + j - 1] * 2 - P[(i + 1)*step + j - 1];
//			PY[i*stepXY + j * (stepXY / step)] = abs(gradY);
//		}
//	}
//	//���ݶ�����ת����8λ�޷������ͣ�opencv�к���
//	convertScaleAbs(imageSobelX, imageSobelX);
//	convertScaleAbs(imageSobelY, imageSobelY);
//}
//
//void SobelXX(const Mat imageGradX, Mat_<float> &SobelAmpXX)
//{
//	//��X������ݶ�ͼ��ת����С������ʾ
//	SobelAmpXX = Mat_<float>(imageGradX.size(), CV_32FC1);
//	//����X�ݶ�ͼƬ
//	for (int i = 0; i < SobelAmpXX.rows; i++)
//	{
//		for (int j = 0; j < SobelAmpXX.cols; j++)
//		{
//			//�õ�ƽ�����ͼ
//			SobelAmpXX.at<float>(i, j) = imageGradX.at<uchar>(i, j)*imageGradX.at<uchar>(i, j);
//		}
//	}
//}
//void SobelYY(const Mat imageGradY, Mat_<float> &SobelAmpYY)
//{
//	//��Y������ݶ�ͼ��ת����С������ʾ
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
////****************����һά��˹��Ȩֵ����*****************
////size�Ǵ���ľ���˵ı߳��Ĵ�С
//double *getOneGuassionArray(int size, double sigma)
//{
//	double sum = 0.0;
//	//�����˹�˰뾶
//	int kerR = size / 2;
//	//����һ��size��С�Ķ�̬һά����
//	double *arr = new double[size];
//	cout << "��˹Ȩֵ���飺" << endl;
//	for (int i = 0; i < size; i++)
//	{
//
//		// ��˹����ǰ�ĳ������Բ��ü��㣬���ڹ�һ���Ĺ����и���ȥ
//		//��˹�˺�����ʽ
//		arr[i] = exp(-((i - kerR)*(i - kerR)) / (2 * sigma*sigma));
//		sum += arr[i];//�����е�ֵ�������
//	}
//
//	//���й�һ��	
//	for (int i = 0; i < size; i++)
//	{
//		arr[i] /= sum;
//		cout << arr[i] << endl;
//	}
//	return arr;
//}
//
////****************��˹�˲�������ʵ��*****************
////srcImage�Ǵ��������ĻҶ�ԭͼ
////dst��ʾ���������ͼ
////size��ʾ���Ǵ��ڵı߳��Ĵ�С	
//void MyGaussianBlur(Mat_<float> &srcImage, Mat_<float> &dst, int size)
//{
//	//����assert
//	CV_Assert(srcImage.channels() == 1 || srcImage.channels() == 3); // ֻ����ͨ��������ͨ��ͼ��
//	int kerR = size / 2;
//	//����һ���Ҷ�ͼ����֤��Сһ��
//	dst = srcImage.clone();
//	double* arr;
//	arr = getOneGuassionArray(size, 1);//�������˹����
//
//		//����ͼ�� ˮƽ����ľ��
//	for (int i = kerR; i < dst.rows - kerR; i++)
//	{
//		for (int j = kerR; j < dst.cols - kerR; j++)
//		{
//			float GuassionSum[3] = { 0 };
//			//����������ɸ�˹��ƽ��
//			for (int k = -kerR; k <= kerR; k++)
//			{
//				GuassionSum[0] += arr[kerR + k] * dst.at<float>(i, j + k);//�в��䣬�б任������ˮƽ����ľ��
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
//	//��ֱ����
//	for (int i = kerR; i < dst.rows - kerR; i++)
//	{
//		for (int j = kerR; j < dst.cols - kerR; j++)
//		{
//			float GuassionSum[3] = { 0 };
//			//����������ɸ�˹��ƽ��
//			for (int k = -kerR; k <= kerR; k++)
//				GuassionSum[0] += arr[kerR + k] * dst.at<float>(i + k, j);//�б䣬�в�����������ֱ����ľ��
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
////****��������ؾ���M������ֵ����Ӧ����R = (A*B - C) - k*(A+B)^2******
//void harrisResponse(Mat_<float> &GaussXX, Mat_<float> &GaussYY, Mat_<float> &GaussXY, Mat_<float> &resultData, float k)
//{
//	//����һ����Ӧ��������ľ���
//	resultData = Mat_<float>(GaussXX.size(), CV_32FC1);
//	for (int i = 0; i < resultData.rows; i++)
//	{
//		for (int j = 0; j < resultData.cols; j++)
//		{
//			float a = GaussXX.at<float>(i, j);
//			float b = GaussYY.at<float>(i, j);
//			float c = GaussXY.at<float>(i, j);
//			//R �ǵ���Ӧ���� R = (A*B - C) - k*(A+B)*��A+B��
//			resultData.at<float>(i, j) = a * b - c * c - k * (a + b)*(a + b);
//		}
//	}
//}
//
////***********�Ǽ���ֵ���ƺ�������ֵ��ĳ�����ڵľֲ�����ֵΪ�ǵ�***
//void LocalMaxValue(Mat_<float> &resultData, Mat &srcGray, Mat &ResultImage, int kSize)
//{
//	int Thr = 20000;
//	//r�Ǵ��ڵ�һ��
//	int r = kSize / 2;
//	//������һ���µ�ԭʼͼ��
//	ResultImage = srcGray.clone();
//	cout << "���ηǼ���ֵ���Ƶ���ֵΪ" << Thr << endl;
//	for (int i = r; i < ResultImage.rows - r; i++)
//	{
//		for (int j = r; j < ResultImage.cols - r; j++)
//		{
//			//��֤�Ǵ����ڼ���ֵ
//			if (resultData.at<float>(i, j) > resultData.at<float>(i - 1, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i - 1, j) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i - 1, j + 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i, j + 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j - 1) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j) &&
//				resultData.at<float>(i, j) > resultData.at<float>(i + 1, j + 1))
//			{
//				//��ֵ
//				if ((int)resultData.at<float>(i, j) > Thr)
//				{
//					//�ڽ��ͼ�ϱ�ע��ȡ�ǵ�
//					circle(ResultImage, Point(i, j), 5, Scalar(0, 0, 255), 2, 8, 0);
//				}
//			}
//
//		}
//	}
//}
