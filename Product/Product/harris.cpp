//#include "Harris.h"
//
//
//Harris::Harris(char*const image_name)
//{
//	initImage = cvLoadImage(image_name);
//	if (initImage == NULL) {
//		cout << "inint error";
//		return;
//	}
//
//	cvNamedWindow("init");
//    cvShowImage("init", initImage);
//  
//	gauss_weight = new double[3 * 3];
//	for (int i = 0; i < 3; i++) {
//		for (int j = 0; j < 3; j++) {
//			gauss_weight[i * 3 + j] = (double)(3 - (abs(i - 1) + abs(j - 1)));
//		}
//	}
//	grayImage_8U = cvCreateImage(cvGetSize(initImage), IPL_DEPTH_8U, 1);
//	grayImage_64F = cvCreateImage(cvGetSize(initImage), IPL_DEPTH_64F, 1);
//	cvCvtColor(initImage, grayImage_8U, CV_BGR2GRAY);
//	cvConvertScale(grayImage_8U, grayImage_64F);
//
//	¼ÆËãÌÝ¶È
//	IplImage* Ix = cvCreateImage(cvGetSize(grayImage_64F), IPL_DEPTH_64F, grayImage_64F->nChannels);
//	IplImage* Iy = cvCreateImage(cvGetSize(grayImage_64F), IPL_DEPTH_64F, grayImage_64F->nChannels);
//	for (int i = 1; i < grayImage_64F->height - 1; i++) {
//		for (int j = 1; j < grayImage_64F->width - 1; j++) {
//			double x_1 = cvGet2D(grayImage_64F, i, j - 1).val[0];
//			double x_2 = cvGet2D(grayImage_64F, i, j + 1).val[0];
//			double y_1 = cvGet2D(grayImage_64F, i - 1, j).val[0];
//			double y_2 = cvGet2D(grayImage_64F, i + 1, j).val[0];
//			CvScalar s;
//			s.val[0] = x_2 - x_1;
//			cvSet2D(Ix, i, j, s);
//			s.val[0] = y_2 - y_1;
//			cvSet2D(Iy, i, j, s);
//		}
//	}
//
//	Ixx = Mul(Ix, Ix);
//	Iyy = Mul(Iy, Iy);
//	Ixy = Mul(Ix, Iy);
//
//	A = MW(Ixx);
//	C = MW(Iyy);
//	B = MW(Ixy);
//
//	Mat harriss(1080, 1920, CV_8UC4);
//	cout << "das" << endl;
//	harris_image = harris_conner();
//	nonMaxValueSuppression();
//	cvNamedWindow("harris");
//	cvShowImage("harris", harris_image);
//	cvWaitKey();
//}
//
//IplImage* Harris::Mul(IplImage* M1, IplImage*M2)
//{
//	IplImage * M = cvCreateImage(cvGetSize(M1), IPL_DEPTH_64F, M1->nChannels);
//	for (int i = 0; i < M1->height; i++) {
//		for (int j = 0; j < M2->width; j++) {
//			double x = cvGet2D(M1, i, j).val[0];
//			double y = cvGet2D(M2, i, j).val[0];
//			CvScalar s;
//			s.val[0] = x * y;
//			cvSet2D(M, i, j, s);
//		}
//	}
//	return M;
//}
//
//IplImage* Harris::MW(IplImage*M)
//{
//	IplImage* MW = cvCreateImage(cvGetSize(M), IPL_DEPTH_64F, M->nChannels);
//	for (int i = 1; i < M->height - 1; i++) {
//		for (int j = 1; j < M->width - 1; j++) {
//			double sum = 0;
//			for (int k = 0; k < 3; k++) {
//				for (int m = 0; m < 3; m++) {
//					double x = cvGet2D(M, i + (k - 1), j + (m - 1)).val[0];
//					sum = sum + x * gauss_weight[k * 3 + m];
//				}
//			}
//			CvScalar s;
//			s.val[0] = sum;
//			cvSet2D(MW, i, j, s);
//		}
//	}
//	return MW;
//}
//
//IplImage*Harris::harris_conner()
//{
//	double alpha = 0.06;
//	IplImage* conner_image = cvCreateImage(cvGetSize(initImage),
//		IPL_DEPTH_64F, initImage->nChannels);
//	for (int i = 0; i < initImage->height; i++) {
//		for (int j = 0; j < initImage->width; j++) {
//			double det = cvGet2D(A, i, j).val[0] * cvGet2D(B, i, j).val[0]
//				- cvGet2D(C, i, j).val[0] * cvGet2D(C, i, j).val[0];
//			double trace = cvGet2D(A, i, j).val[0] + cvGet2D(B, i, j).val[0];
//			CvScalar s;
//			s.val[0] = det - alpha * trace*trace;
//			cvSet2D(conner_image, i, j, s);
//		}
//	}
//	return conner_image;
//}
//
//IplImage* Harris::nonMaxValueSuppression()
//{
//	double t = 100000000;
//	for (int i = 1; i < harris_image->height - 1; i++) {
//		for (int j = 1; j < harris_image->width - 1; j++) {
//			if (cvGet2D(harris_image, i, j).val[0] < t) {
//				CvScalar s;
//				s.val[0] = 0;
//				cvSet2D(harris_image, i, j, s);
//				continue;
//			}
//			double max = cvGet2D(harris_image, i, j).val[0], min = cvGet2D(harris_image, i, j).val[0];
//			int max_x = i, max_y = j;
//			int min_x = i, min_y = j;
//			for (int k = -1; k < 2; k++) {
//				for (int m = -1; m < 2; m++) {
//					if (k == 0 && m == 0) {
//						continue;
//					}
//					if (max < cvGet2D(harris_image, i + k, j + m).val[0]) {
//						max_x = i + k;
//						max_y = j + m;
//					}
//					if (min > cvGet2D(harris_image, i + k, j + m).val[0]) {
//						min_x = i + k;
//						min_y = j + m;
//					}
//				}
//			}
//			if (max_x == i && max_y == j || min_x == i && min_y == j) {
//				CvScalar s;
//				s.val[0] = 255;
//				cvSet2D(harris_image, i, j, s);
//			}
//			else {
//				CvScalar s;
//				s.val[0] = 0;
//				cvSet2D(harris_image, i, j, s);
//			}
//		}
//	}
//	return harris_image;
//}
//
//Harris::~Harris()
//{
//	dtor
//}
//
//int main() {
//	char* p;
//	p = (char*)"haha.jpg";
//	Harris::Harris(p);
//	return 0;
//}
