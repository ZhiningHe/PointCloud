#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include <ctime>

using namespace std;
using namespace cv;

const double w = 0.95;
const int r = 7;

//计算暗通道影像
Mat getDarkChannelImg(const Mat src, const int r)
{
	int height = src.rows;
	int width = src.cols;
	Mat darkChannelImg(src.size(), CV_8UC1);
	Mat darkTemp(darkChannelImg.size(), darkChannelImg.type());

	//求取src中每个像素点三个通道中的最小值，将其赋值给暗通道图像中对应的像素点
	for (int i = 0; i < height; i++)
	{
		const uchar* srcPtr = src.ptr<uchar>(i);
		uchar* dstPtr = darkTemp.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
		{
			int b = srcPtr[3 * j];
			int g = srcPtr[3 * j + 1];
			int r = srcPtr[3 * j + 2];
			dstPtr[j] = min(min(b, g), r);
		}
	}

	//把图像分成patch,求patch框内的最小值,得到dark_channel image
	//r is the patch radius, patchSize=2*r+1 
	//这一步实际上是最小值滤波的过程
	cv::Mat rectImg;
	int patchSize = 2 * r + 1;
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			cv::getRectSubPix(darkTemp, cv::Size(patchSize, patchSize), cv::Point(i, j), rectImg);
			double minValue = 0;
			cv::minMaxLoc(rectImg, &minValue, 0, 0, 0); //get min pix value
			darkChannelImg.at<uchar>(j, i) = cv::saturate_cast<uchar>(minValue);//using saturate_cast to set pixel value to [0,255]  
		}
	}
	return darkChannelImg;
}

//计算A
double getGlobelAtmosphericLight(const Mat darkChannelImg)
{
	//这里是简化的处理方式,A的最大值限定为220
	double minAtomsLight = 220;//经验值
	double maxValue = 0;
	cv::Point maxLoc;
	minMaxLoc(darkChannelImg, NULL, &maxValue, NULL, &maxLoc);
	double A = min(minAtomsLight, maxValue);
	return A;
}

//tx
Mat getTransimissionImg(const Mat darkChannelImg, const double A)
{
	cv::Mat transmissionImg(darkChannelImg.size(), CV_8UC1);
	cv::Mat look_up(1, 256, CV_8UC1);

	uchar* look_up_ptr = look_up.data;
	for (int k = 0; k < 256; k++)
	{
		look_up_ptr[k] = cv::saturate_cast<uchar>(255 * (1 - w * k / A));
	}

	cv::LUT(darkChannelImg, look_up, transmissionImg);

	return transmissionImg;
}

//导向滤波
Mat fastGuidedFilter(cv::Mat I_org, cv::Mat p_org, int r, double eps, int s)
{
	/*
	% GUIDEDFILTER   O(N) time implementation of guided filter.
	%
	%   - guidance image: I (should be a gray-scale/single channel image)
	%   - filtering input image: p (should be a gray-scale/single channel image)
	%   - local window radius: r
	%   - regularization parameter: eps
	*/

	cv::Mat I, _I;
	I_org.convertTo(_I, CV_64FC1, 1.0 / 255);
	resize(_I, I, Size(), 1.0 / s, 1.0 / s, 1);

	cv::Mat p, _p;
	p_org.convertTo(_p, CV_64FC1, 1.0 / 255);
	//p = _p;
	resize(_p, p, Size(), 1.0 / s, 1.0 / s, 1);

	//[hei, wid] = size(I);    
	int hei = I.rows;
	int wid = I.cols;

	r = (2 * r + 1) / s + 1;//因为opencv自带的boxFilter（）中的Size,比如9x9,我们说半径为4   

	//mean_I = boxfilter(I, r) ./ N;    
	cv::Mat mean_I;
	cv::boxFilter(I, mean_I, CV_64FC1, cv::Size(r, r));

	//mean_p = boxfilter(p, r) ./ N;    
	cv::Mat mean_p;
	cv::boxFilter(p, mean_p, CV_64FC1, cv::Size(r, r));

	//mean_Ip = boxfilter(I.*p, r) ./ N;    
	cv::Mat mean_Ip;
	cv::boxFilter(I.mul(p), mean_Ip, CV_64FC1, cv::Size(r, r));

	//cov_Ip = mean_Ip - mean_I .* mean_p; % this is the covariance of (I, p) in each local patch.    
	cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);

	//mean_II = boxfilter(I.*I, r) ./ N;    
	cv::Mat mean_II;
	cv::boxFilter(I.mul(I), mean_II, CV_64FC1, cv::Size(r, r));

	//var_I = mean_II - mean_I .* mean_I;    
	cv::Mat var_I = mean_II - mean_I.mul(mean_I);

	//a = cov_Ip ./ (var_I + eps); % Eqn. (5) in the paper;       
	cv::Mat a = cov_Ip / (var_I + eps);

	//b = mean_p - a .* mean_I; % Eqn. (6) in the paper;    
	cv::Mat b = mean_p - a.mul(mean_I);

	//mean_a = boxfilter(a, r) ./ N;    
	cv::Mat mean_a;
	cv::boxFilter(a, mean_a, CV_64FC1, cv::Size(r, r));
	Mat rmean_a;
	resize(mean_a, rmean_a, Size(I_org.cols, I_org.rows), 1);

	//mean_b = boxfilter(b, r) ./ N;    
	cv::Mat mean_b;
	cv::boxFilter(b, mean_b, CV_64FC1, cv::Size(r, r));
	Mat rmean_b;
	resize(mean_b, rmean_b, Size(I_org.cols, I_org.rows), 1);

	//q = mean_a .* I + mean_b; % Eqn. (8) in the paper;    
	cv::Mat q = rmean_a.mul(_I) + rmean_b;
	Mat q1;
	q.convertTo(q1, CV_8UC1, 255, 0);

	return q1;
}

//去雾操作
Mat getDehazedChannel(cv::Mat srcChannel, cv::Mat transmissionChannel, double A)
{
	double tmin = 0.1;
	double tmax;

	cv::Mat dehazedChannel(srcChannel.size(), CV_8UC1);
	for (int i = 0; i < srcChannel.rows; i++)
	{
		for (int j = 0; j < srcChannel.cols; j++)
		{
			double transmission = transmissionChannel.at<uchar>(i, j);

			tmax = (transmission / 255) < tmin ? tmin : (transmission / 255);
			//(I-A)/t +A  
			dehazedChannel.at<uchar>(i, j) = cv::saturate_cast<uchar>(abs((srcChannel.at<uchar>(i, j) - A) / tmax + A));
		}
	}
	return dehazedChannel;
}

//去雾
Mat getDehazedImg_guidedFilter(Mat src, Mat darkChannelImg)
{
	cv::Mat dehazedImg = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

	cv::Mat transmissionImg(src.rows, src.cols, CV_8UC3);
	cv::Mat fineTransmissionImg(src.rows, src.cols, CV_8UC3);
	std::vector < cv::Mat> srcChannel, dehazedChannel, transmissionChannel, fineTransmissionChannel;
	//A
	cv::split(src, srcChannel);
	double A0 = getGlobelAtmosphericLight(darkChannelImg);
	double A1 = getGlobelAtmosphericLight(darkChannelImg);
	double A2 = getGlobelAtmosphericLight(darkChannelImg);
	//tx
	cv::split(transmissionImg, transmissionChannel);
	transmissionChannel[0] = getTransimissionImg(darkChannelImg, A0);
	transmissionChannel[1] = getTransimissionImg(darkChannelImg, A1);
	transmissionChannel[2] = getTransimissionImg(darkChannelImg, A2);
	//滤波
	cv::split(fineTransmissionImg, fineTransmissionChannel);
	fineTransmissionChannel[0] = fastGuidedFilter(srcChannel[0], transmissionChannel[0], 64, 0.01, 8);
	fineTransmissionChannel[1] = fastGuidedFilter(srcChannel[1], transmissionChannel[1], 64, 0.01, 8);
	fineTransmissionChannel[2] = fastGuidedFilter(srcChannel[2], transmissionChannel[2], 64, 0.01, 8);
	//合成
	merge(fineTransmissionChannel, fineTransmissionImg);
	imshow("fineTransmissionChannel", fineTransmissionImg);
	
	cv::split(dehazedImg, dehazedChannel);
	dehazedChannel[0] = getDehazedChannel(srcChannel[0], fineTransmissionChannel[0], A0);
	dehazedChannel[1] = getDehazedChannel(srcChannel[1], fineTransmissionChannel[1], A1);
	dehazedChannel[2] = getDehazedChannel(srcChannel[2], fineTransmissionChannel[2], A2);
	//合成
	cv::merge(dehazedChannel, dehazedImg);

	return dehazedImg;
}

//去雾
Mat getDehazedImg(const Mat src, const Mat transmissionImage, const int A)
{
	double tmin = 0.1;
	double tmax = 0;

	Vec3b srcData;
	Mat dehazedImg = Mat::zeros(src.size(), CV_8UC3);

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			double transmission = transmissionImage.at<uchar>(i, j);
			srcData = src.at<Vec3b>(i, j);
			//归一化tx
			tmax = max(transmission / 255, tmin);
			//(I-A)/t +A  
			for (int c = 0; c < 3; c++)
			{
				dehazedImg.at<cv::Vec3b>(i, j)[c] = cv::saturate_cast<uchar>(abs((srcData.val[c] - A) / tmax + A));
			}
		}
	}
	return dehazedImg;
}




int main(void)
{
	cv::Mat src, darkChanelImg;
	src = imread("sea.jpg");
	if (src.empty())
		std::cout << "Load Image Error!";

									double t1 = (double)getTickCount();	//开始计时
									cout << "start:" << endl;
	src.convertTo(src, CV_8U, 1, 0);
	//暗通道图像
	darkChanelImg = getDarkChannelImg(src, r);
	cv::imshow("darkChanelImg", darkChanelImg);
	Mat  dehazedImg_guideFilter;
	//导通滤波计算t（x）
	dehazedImg_guideFilter = getDehazedImg_guidedFilter(src, darkChanelImg);

									//结束时间
									t1 = (double)getTickCount() - t1;
									std::cout <<"去雾用时："<< 1000 * t1 / (getTickFrequency()) << "ms" << std::endl;

	imshow("src", src);
	imshow("dehazedImg_guideFilter", dehazedImg_guideFilter);
	cvWaitKey(0);
	return 0;
}
