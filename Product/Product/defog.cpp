#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include<algorithm>
#include<set>
#include<map>
#include <ctime>

using namespace std;
using namespace cv;


//导向滤波，用来优化t(x)，针对单通道
Mat guidedfilter(Mat &srcImage, Mat &srcClone, int r, double eps)
{
	//转换源图像信息
	srcImage.convertTo(srcImage, CV_32FC1, 1 / 255.0);
	srcClone.convertTo(srcClone, CV_32FC1);
	int nRows = srcImage.rows;
	int nCols = srcImage.cols;

	Mat boxResult;
	//步骤一：计算均值
	boxFilter(Mat::ones(nRows, nCols, srcImage.type()),boxResult, CV_32FC1, Size(r, r));

	//生成导向均值mean_I
	Mat mean_I;
	boxFilter(srcImage, mean_I, CV_32FC1, Size(r, r));

	//生成原始均值mean_p
	Mat mean_p;
	boxFilter(srcClone, mean_p, CV_32FC1, Size(r, r));

	//生成互相关均值mean_Ip
	Mat mean_Ip;
	boxFilter(srcImage.mul(srcClone), mean_Ip,CV_32FC1, Size(r, r));
	Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);
	//生成自相关均值mean_II
	Mat mean_II;
	//应用盒滤波器计算相关的值
	boxFilter(srcImage.mul(srcImage), mean_II,
		CV_32FC1, Size(r, r));
	//步骤二：计算相关系数
	Mat var_I = mean_II - mean_I.mul(mean_I);
	Mat var_Ip = mean_Ip - mean_I.mul(mean_p);
	//步骤三：计算参数系数a,b
	Mat a = cov_Ip / (var_I + eps);
	Mat b = mean_p - a.mul(mean_I);
	//步骤四：计算系数a\b的均值
	Mat mean_a;
	boxFilter(a, mean_a, CV_32FC1, Size(r, r));
	mean_a = mean_a / boxResult;

	Mat mean_b;
	boxFilter(b, mean_b, CV_32FC1, Size(r, r));
	mean_b = mean_b / boxResult;
	//步骤五：生成输出矩阵
	Mat resultMat = mean_a.mul(srcImage) + mean_b;
	return resultMat;
}

//计算暗通道图像矩阵，针对三通道彩色图像
Mat dark_channel(Mat src)
{
	//窗口为15*15
	int border = 7;
	std::vector<cv::Mat> rgbChannels(3);
	Mat min_mat(src.size(), CV_8UC1, Scalar(0)), min_mat_expansion;
	Mat dark_channel_mat(src.size(), CV_8UC1, Scalar(0));
	split(src, rgbChannels);
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			int min_val = 0;
			int val_1, val_2, val_3;
			val_1 = rgbChannels[0].at<uchar>(i, j);
			val_2 = rgbChannels[1].at<uchar>(i, j);
			val_3 = rgbChannels[2].at<uchar>(i, j);

			min_val = std::min(val_1, val_2);
			min_val = std::min(min_val, val_3);
			//选择最小的赋值
			min_mat.at<uchar>(i, j) = min_val;

		}
	}
	copyMakeBorder(min_mat, min_mat_expansion, border, border, border, border, BORDER_REPLICATE);

	for (int m = border; m < min_mat_expansion.rows - border; m++)
	{
		for (int n = border; n < min_mat_expansion.cols - border; n++)
		{
			Mat imageROI;
			int min_num = 256;
			imageROI = min_mat_expansion(Rect(n - border, m - border, 2 * border + 1, 2 * border + 1));
			for (int i = 0; i < imageROI.rows; i++)
			{
				for (int j = 0; j < imageROI.cols; j++)
				{
					int val_roi = imageROI.at<uchar>(i, j);
					min_num = std::min(min_num, val_roi);
				}
			}
			dark_channel_mat.at<uchar>(m - border, n - border) = min_num;
		}
	}
	return dark_channel_mat;
}

//计算A
int calculate_A(Mat src, Mat dark_channel_mat)
{
	std::vector<cv::Mat> rgbChannels(3);
	//分离三通道
	split(src, rgbChannels);
	map<int, Point> pair_data;
	map<int, Point>::iterator iter;
	vector<Point> cord;
	int max_val = 0;
	//cout << dark_channel_mat.rows << " " << dark_channel_mat.cols << endl;
	for (int i = 0; i < dark_channel_mat.rows; i++)
	{
		for (int j = 0; j < dark_channel_mat.cols; j++)
		{
			int val = dark_channel_mat.at<uchar>(i, j);
			Point pt;
			pt.x = j;
			pt.y = i;
			pair_data.insert(make_pair(val, pt));
		}
	}

	for (iter = pair_data.begin(); iter != pair_data.end(); iter++)
	{
		//cout << iter->first << endl;
		cord.push_back(iter->second);
	}
	for (int m = 0; m < cord.size(); m++)
	{
		Point tmp = cord[m];
		int val_1, val_2, val_3;
		val_1 = rgbChannels[0].at<uchar>(tmp.y, tmp.x);
		val_2 = rgbChannels[1].at<uchar>(tmp.y, tmp.x);
		val_3 = rgbChannels[2].at<uchar>(tmp.y, tmp.x);
		max_val = std::max(val_1, val_2);
		max_val = std::max(max_val, val_3);

	}
	return max_val;

}
  
Mat calculate_tx( int A, Mat &dark_channel_mat)
{
	Mat dst;//是用来计算t(x)
	Mat tx;
	float dark_channel_num;
	dark_channel_num = A / 255.0;

	dark_channel_mat.convertTo(dst, CV_32FC3, 1 / 255.0);
	dst = dst / dark_channel_num;
	tx = 1 - 0.95*dst;//最终的tx图
	float t0 = 0.01;
	for (unsigned int r = 0; r < tx.rows; r++)
	{
		for (unsigned int c = 0; c < tx.cols; c++)
		{
			//std::cout<< tx.at<float>(r, c) << endl;
			if (tx.at<float>(r, c) < t0) {
				tx.at<float>(r, c) = (t0 - tx.at<float>(r, c)) / t0;
				}
			}
		}
	return tx;
	}


Mat haze_removal_img(Mat &src, int A, Mat &tx)
{
	Mat result_img(src.rows, src.cols, CV_8UC3);
	vector<Mat> srcChannels(3), resChannels(3);
	split(src, srcChannels);
	split(result_img, resChannels);

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			for (int m = 0; m < 3; m++)
			{
				int value_num = srcChannels[m].at<uchar>(i, j);
				float max_t = tx.at<float>(i, j);
				//t0为0.1；太小对比会很强烈
				if (max_t < 0.1)
				{
					max_t = 0.1;
				}
				//核心公式
				resChannels[m].at<uchar>(i, j) = (value_num - A) / max_t + A;
				//排除异常值
				resChannels[m].at<uchar>(i,j)>255 ? 255 : resChannels[m].at<uchar>(i, j);
				resChannels[m].at<uchar>(i, j)< 0 ?0 : resChannels[m].at<uchar>(i, j);
			}
		}
	}
	merge(resChannels, result_img);
	return result_img;
}


int main()
{
	time_t start = time(0);
	cout << start << endl;
	Mat src = imread("4.jpg");
	Mat dst;	//灰度图像
	cvtColor(src, dst, CV_BGR2GRAY);

	Mat dark_channel_mat = dark_channel(src);//输出的是暗通道图像
	//A
	int A = calculate_A(src, dark_channel_mat);
	//tx
	Mat tx = calculate_tx( A, dark_channel_mat);
	Mat tx_ = guidedfilter(dst, tx, 40, 0.001);	//导向滤波后的tx，dst为引导图像
	imshow("darkchannel", tx_);
	//去雾
	Mat haze_removal_image;
	haze_removal_image = haze_removal_img(src, A, tx_);
	imwrite("defog.jpg", haze_removal_image);

	double alpha = 1.5; //对比度控制 [1~3]
	int beta = 2; //明亮度控制 [0~100]
	Mat light_img = Mat::zeros(haze_removal_image.size(), haze_removal_image.type());
	haze_removal_image.convertTo(light_img, -1, alpha, beta);
	imshow("变换后图像", light_img);

	time_t end = time(0);
	cout << end << endl;
	cout <<"用时：" <<end - start<<"s" << endl;
	//显示
	namedWindow("去雾后的图像", 0);
	namedWindow("原始图像", 0);
	imshow("原始图像", src);
	imshow("去雾后的图像", haze_removal_image);
	waitKey(0);
	return 0;
}




