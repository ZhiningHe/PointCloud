////
////#include <pcl/io/io.h>
////#include <pcl/io/pcd_io.h>
////#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
////#include <pcl/visualization/cloud_viewer.h>
////#include <pcl/point_types.h>
////#include <pcl/features/normal_3d.h>
////
////int main()
////{
////	//打开点云代码
////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
////	pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);
////	//创建法线估计估计向量
////	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
////	ne.setInputCloud(cloud);
////	//创建一个空的KdTree对象，并把它传递给法线估计向量
////	//基于给出的输入数据集，KdTree将被建立
////	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
////	ne.setSearchMethod(tree);
////	//存储输出数据
////	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
////	//使用半径在查询点周围3厘米范围内的所有临近元素
////	ne.setRadiusSearch(0.03);
////	//计算特征值
////	ne.compute(*cloud_normals);
////	// cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同的尺寸
////	//可视化
////	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
////	viewer.setBackgroundColor(0.0, 0.0, 0.0);
////	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);
////	//视点默认坐标是（0，0，0）可使用 setViewPoint(float vpx,float vpy,float vpz)来更换
////	while (!viewer.wasStopped())
////	{
////		viewer.spinOnce();
////	}
////	return 0;
////}
//
////#include <opencv2/imgproc/imgproc_c.h>  
////#include <opencv2/legacy/legacy.hpp>  
////#include "opencv2/highgui/highgui.hpp"  
////#include<opencv2\opencv.hpp>  
////#include<iostream>  
////#include <stdio.h>  
////using namespace std;
////using namespace cv;
////static void help(void)
////{
////	printf("\nThis program demostrates iterative construction of\n"//这个程序阐述了delaunay剖分和voronoi细分的迭代构造  
////		"delaunay triangulation and voronoi tesselation.\n"
////		"It draws a random set of points in an image and then delaunay triangulates them.\n"//在图像上画出一些随机点，然后进行delaunay三角剖分  
////		"Usage: \n"
////		"./delaunay \n"
////		"\nThis program builds the traingulation interactively, you may stop this process by\n"
////		"hitting any key.\n");//迭代构造三角剖分，如果像停止，则按任意键  
////}
////
////static CvSubdiv2D* init_delaunay(CvMemStorage* storage,//初始化三角剖分结构，为其分配单元  
////	CvRect rect)
////{
////	CvSubdiv2D* subdiv;//三角剖分的数据单元  
////
////	subdiv = cvCreateSubdiv2D(CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
////		sizeof(CvSubdiv2DPoint),
////		sizeof(CvQuadEdge2D),
////		storage);
////	cvInitSubdivDelaunay2D(subdiv, rect);
////
////	return subdiv;
////}
////
////
////static void draw_subdiv_point(IplImage* img, CvPoint2D32f fp, CvScalar color)//画出三角剖分的顶点  
////{
////	cvCircle(img, cvPoint(cvRound(fp.x), cvRound(fp.y)), 5, color, CV_FILLED, 8, 0);
////}
////
////
////static void draw_subdiv_edge(IplImage* img, CvSubdiv2DEdge edge, CvScalar color)//画出三角剖分的边  
////{
////	CvSubdiv2DPoint* org_pt;//源顶点  
////	CvSubdiv2DPoint* dst_pt;//目地顶点  
////	CvPoint2D32f org;
////	CvPoint2D32f dst;
////	CvPoint iorg, idst;
////
////	org_pt = cvSubdiv2DEdgeOrg(edge);//通过边获取顶点  
////	dst_pt = cvSubdiv2DEdgeDst(edge);
////
////	if (org_pt && dst_pt)//如果两个端点不为空  
////	{
////		org = org_pt->pt;
////		dst = dst_pt->pt;
////
////		iorg = cvPoint(cvRound(org.x), cvRound(org.y));
////		idst = cvPoint(cvRound(dst.x), cvRound(dst.y));
////
////		cvLine(img, iorg, idst, color, 1, CV_AA, 0);
////	}
////}
////
////
////static void draw_subdiv(IplImage* img, CvSubdiv2D* subdiv,
////	CvScalar delaunay_color, CvScalar voronoi_color)//画出剖分和细分  
////{
////	CvSeqReader  reader;
////	int i, total = subdiv->edges->total;//边的数量  
////	int elem_size = subdiv->edges->elem_size;//边的大小  
////	cout << typeid(subdiv->edges).name() << endl;
////
////	cvStartReadSeq((CvSeq*)(subdiv->edges), &reader, 0);//使用CvSeqReader遍历Delaunay或者Voronoi边  
////
////	for (i = 0; i < total; i++)
////	{
////		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);
////
////		if (CV_IS_SET_ELEM(edge))
////		{
////			// draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );  
////			draw_subdiv_edge(img, (CvSubdiv2DEdge)edge, delaunay_color);
////		}
////
////		CV_NEXT_SEQ_ELEM(elem_size, reader);
////	}
////
////}
////
////
////static void locate_point(CvSubdiv2D* subdiv, CvPoint2D32f fp, IplImage* img,//遍历三角剖分的边  
////	CvScalar active_color)
////{
////	CvSubdiv2DEdge e;
////	CvSubdiv2DEdge e0 = 0;
////	CvSubdiv2DPoint* p = 0;
////
////	cvSubdiv2DLocate(subdiv, fp, &e0, &p);
////
////	if (e0)
////	{
////		e = e0;
////		do
////		{
////			draw_subdiv_edge(img, e, active_color);
////			e = cvSubdiv2DGetEdge(e, CV_NEXT_AROUND_LEFT);
////		} while (e != e0);
////	}
////
////	draw_subdiv_point(img, fp, active_color);
////}
////
//////@author andme-单目视觉  
////void dashLine(Mat &img, Point2d& pt1, Point2d& pt2, int n)//n为虚线段数  
////{
////	Point sub = pt2 - pt1;
////	for (int i = 0; i < 2 * n; i += 2)
////	{
////		line(img, Point(pt1.x + sub.x * i / (2 * n - 1), pt1.y + sub.y * i / (2 * n - 1)), Point(pt1.x + sub.x * (i + 1) / (2 * n - 1), pt1.y + sub.y * (i + 1) / (2 * n - 1)), Scalar(0, 255, 0), 2);
////	}
////}
////
////
////
//////调用形式draw_subdiv_facet( img, cvSubdiv2DRotateEdge( e, 1 ));  
////static void draw_subdiv_facet(IplImage* img, CvSubdiv2DEdge edge)//画出voronoi面   
////{
////	//cout<<edge<<endl;//edge低两位表示表示索引，高位表示四方边缘指针。  
////	//cout<<(edge&3)<<endl;  
////	CvSubdiv2DEdge t = edge;//当我们按上面的调用形式时，edge为eRot。  
////	int i, count = 0;
////	CvPoint* buf = 0;
////	Point2d *buf1 = 0;
////
////	// count number of edges in facet //面内边的计数  
////	do
////	{
////		count++;
////		t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);
////	} while (t != edge);//我们绕着一个voronoi单元一周，遍历该vornonoi边缘所拥有的边缘数。  
////
////	buf = (CvPoint*)malloc(count * sizeof(buf[0]));
////	buf1 = (Point2d*)malloc(count * sizeof(buf1[0]));
////
////	// gather points  
////	t = edge;
////	for (i = 0; i < count; i++)
////	{
////		CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg(t);//第一次获取eRot边缘的起始点  
////		if (!pt) break;//如果得不到该源点，则退出循环  
////		buf[i] = cvPoint(cvRound(pt->pt.x), cvRound(pt->pt.y));//将该点转换为cvPoint类型点，存储在buf中  
////		t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);//然后绕着vornonoi单元，左旋转。  
////	}
////
////	if (i == count)//如果所有的点都存储起来了。  
////	{
////		CvSubdiv2DPoint* pt = cvSubdiv2DEdgeDst(cvSubdiv2DRotateEdge(edge, 1));//这里eRot的旋转边缘应该是reversed e,那么目的点，就是e的源点。  
////		// cvFillConvexPoly( img, buf, count, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );//填充凸多边形  
////		for (i = 0; i < count; i++)
////		{
////			buf1[i].x = buf[i].x;
////			buf1[i].y = buf[i].y;
////		}
////		Mat mat_img(img);
////
////		cvPolyLine(img, &buf, &count, 1, 1, CV_RGB(0, 200, 0), 1, CV_AA, 0);//画出线。  
////		//for(int i=0;i<count-1;i++)  
////		//{  
////		//dashLine(mat_img,buf1[i],buf1[i+1],100);  
////		//}  
////		//dashLine(mat_img,buf1[i],buf1[0],100);  
////		draw_subdiv_point(img, pt->pt, CV_RGB(255, 0, 0));//用黑色画出画出剖分顶点。  
////	}
////	free(buf);
////}
////
////static void paint_voronoi(CvSubdiv2D* subdiv, IplImage* img)//画出voronoi面  
////{
////	CvSeqReader  reader;
////	int i, total = subdiv->edges->total;//边缘总数  
////	int elem_size = subdiv->edges->elem_size;//边缘的大小  
////
////	cvCalcSubdivVoronoi2D(subdiv);
////
////	cvStartReadSeq((CvSeq*)(subdiv->edges), &reader, 0);
////
////	for (i = 0; i < total; i++)
////	{
////		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);//获取四方边缘  
////
////		if (CV_IS_SET_ELEM(edge))//判断边缘是否在边缘集中  
////		{
////			CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;//edge是四方边缘的指针，而CvSubdiv2DEdge高位表示四方边缘的指针。  
////			//cout<<(e&3)<<endl;//通过测试e低2位即索引值应该设置为0了，即输入边缘  
////			// left  
////			draw_subdiv_facet(img, cvSubdiv2DRotateEdge(e, 1));//e为Delaunay边，获得Delaunay边对应的voronoi边，即e的旋转边缘  
////
////			// right  
////			draw_subdiv_facet(img, cvSubdiv2DRotateEdge(e, 3));//反向的旋转边缘  
////		}
////
////		CV_NEXT_SEQ_ELEM(elem_size, reader);//移动到下一个位置  
////	}
////}
////
////
////static void run(void)
////{
////	char win[] = "source";
////	int i;
////	CvRect rect = { 0, 0, 600, 600 };
////	CvMemStorage* storage;
////	CvSubdiv2D* subdiv;
////	IplImage* img;
////	CvScalar active_facet_color, delaunay_color, voronoi_color, bkgnd_color;
////
////	active_facet_color = CV_RGB(255, 0, 0);//红色  
////	delaunay_color = CV_RGB(0, 0, 0);//黑色  
////	voronoi_color = CV_RGB(0, 180, 0);//绿色  
////	bkgnd_color = CV_RGB(255, 255, 255);//白色  
////
////	img = cvCreateImage(cvSize(rect.width, rect.height), 8, 3);
////	cvSet(img, bkgnd_color, 0);
////
////	cvNamedWindow(win, 1);
////
////	storage = cvCreateMemStorage(0);
////	subdiv = init_delaunay(storage, rect);
////
////	printf("Delaunay triangulation will be build now interactively.\n"
////		"To stop the process, press any key\n\n");
////
////	vector<CvPoint2D32f> points;
////	for (i = 0; i < 5; i++)
////	{
////		CvPoint2D32f fp = cvPoint2D32f((float)(rand() % (rect.width - 10)),//使点约束在距离边框10像素之内。  
////			(float)(rand() % (rect.height - 10)));
////		points.push_back(fp);
////
////		locate_point(subdiv, fp, img, active_facet_color);//定位点的位置，并画出点所在voronoi面的边。  
////		cvShowImage(win, img);//刷新显示  
////
////		if (cvWaitKey(100) >= 0)
////			break;
////
////		cvSubdivDelaunay2DInsert(subdiv, fp);//向三角剖分中插入该点，即对该点进行三角剖分  
////		cvCalcSubdivVoronoi2D(subdiv);//计算Voronoi细分，有时候我们不需要  
////		cvSet(img, bkgnd_color, 0);//设置图像的背景颜色为白色  
////		draw_subdiv(img, subdiv, delaunay_color, voronoi_color);
////		cvShowImage(win, img);
////
////		//cvWaitKey();  
////		if (cvWaitKey(100) >= 0)
////			break;
////	}
////	for (int i = 0; i < points.size(); i++)
////		draw_subdiv_point(img, points[i], active_facet_color);
////	cvShowImage(win, img);
////	cvWaitKey();
////
////	//  cvSet( img, bkgnd_color, 0 );//重新刷新画布，即设置背景颜色为白色  
////	paint_voronoi(subdiv, img);//画出细分  
////	cvShowImage(win, img);//  
////
////	cvWaitKey(0);
////
////	cvReleaseMemStorage(&storage);
////	cvReleaseImage(&img);
////	cvDestroyWindow(win);
////}
////
////int main(int argc, char** argv)
////{
////	(void)argc; (void)argv;
////	help();
////	run();
////	return 0;
////}
////
////#ifdef _EiC  
////main(1, "delaunay.c");
////#endif  
//
//
//
//#include <opencv2/legacy/legacy.hpp>  
//#include <opencv2/opencv.hpp>    
//#include <opencv2/nonfree/nonfree.hpp>    
//#include <opencv2/nonfree/features2d.hpp>    
//#include <atlstr.h> // use STL string instead, although not as convenient...  
//#include <atltrace.h>  
//#include <iostream>  
//#include <fstream>  
//#include <string>  
//#include<time.h>  
//
//using namespace std;
//using namespace cv;
///*
//pts，要剖分的散点集,in
//img,剖分的画布,in
//tri,存储三个表示顶点变换的正数,out
//*/
//
//// used for doing delaunay trianglation with opencv function  
////该函数用来防止多次重画并消去虚拟三角形的 顶点  
//bool isGoodTri(Vec3i &v, vector<Vec3i> & tri)
//{
//	int a = v[0], b = v[1], c = v[2];
//	v[0] = min(a, min(b, c));//v[0]找到点插入的先后顺序（0....N-1，N为点的个数）的最小值  
//	v[2] = max(a, max(b, c));//v[2]存储最大值.  
//	v[1] = a + b + c - v[0] - v[2];//v[1]为中间值  
//	if (v[0] == -1) return false;
//
//	vector<Vec3i>::iterator iter = tri.begin();//开始时为空  
//	for (; iter != tri.end(); iter++)
//	{
//		Vec3i &check = *iter;//如果当前待压入的和存储的重复了，则停止返回false。  
//		if (check[0] == v[0] &&
//			check[1] == v[1] &&
//			check[2] == v[2])
//		{
//			break;
//		}
//	}
//	if (iter == tri.end())
//	{
//		tri.push_back(v);
//		return true;
//	}
//	return false;
//}
///*
//pts，要剖分的散点集,in
//img,剖分的画布,in
//tri,存储三个表示顶点变换的正数,out
//*/
//void TriSubDiv(vector<Point2f> &pts, Mat &img, vector<Vec3i> &tri)
//{
//	CvSubdiv2D* subdiv;
//	CvMemStorage* storage = cvCreateMemStorage(0); //创建存储器  
//	Rect rc = Rect(0, 0, img.cols, img.rows);//矩形是图像的大小  
//
//	subdiv = cvCreateSubdiv2D(CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
//		sizeof(CvSubdiv2DPoint),
//		sizeof(CvQuadEdge2D),
//		storage);//为剖分数据分配空间  
//	cvInitSubdivDelaunay2D(subdiv, rc);
//	for (size_t i = 0; i < pts.size(); i++)
//	{
//		CvSubdiv2DPoint *pt = cvSubdivDelaunay2DInsert(subdiv, pts[i]);//利用插入法进行剖分  
//		pt->id = i;//为每一个顶点分配一个id  
//
//	}
//
//	CvSeqReader reader;//利用CvSeqReader遍历  
//	int total = subdiv->edges->total;//边的总数  
//	int elem_size = subdiv->edges->elem_size;//边的大小  
//
//	cvStartReadSeq((CvSeq*)(subdiv->edges), &reader, 0);
//	Point buf[3];
//	const Point *pBuf = buf;
//	Vec3i verticesIdx;
//	Mat imgShow = img.clone();
//
//	srand((unsigned)time(NULL));
//	for (int i = 0; i < total; i++)
//	{
//		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);
//
//		if (CV_IS_SET_ELEM(edge))
//		{
//			CvSubdiv2DEdge t = (CvSubdiv2DEdge)edge;
//			int iPointNum = 3;
//			Scalar color = CV_RGB(rand() & 255, rand() & 255, rand() & 255);
//			//Scalar color=CV_RGB(255,0,0);  
//			//bool isNeg = false;  
//			int j;
//			for (j = 0; j < iPointNum; j++)
//			{
//				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg(t);//获取t边的源点  
//				if (!pt) break;
//				buf[j] = pt->pt;//将点存储起来  
//				//if (pt->id == -1) isNeg = true;  
//				verticesIdx[j] = pt->id;//获取顶点的Id号，将三个点的id存储到verticesIdx中  
//				t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);//获取下一条边  
//			}
//			if (j != iPointNum) continue;
//			if (isGoodTri(verticesIdx, tri))
//			{
//				//tri.push_back(verticesIdx);  
//				polylines(imgShow, &pBuf, &iPointNum,
//					1, true, color,
//					1, CV_AA, 0);//画出三条边  
//				//printf("(%d, %d)-(%d, %d)-(%d, %d)\n", buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);  
//				//printf("%d\t%d\t%d\n", verticesIdx[0], verticesIdx[1], verticesIdx[2]);  
//				//imshow("Delaunay", imgShow);  
//				//waitKey();  
//			}
//
//			t = (CvSubdiv2DEdge)edge + 2;//相反边缘 reversed e  
//
//			for (j = 0; j < iPointNum; j++)
//			{
//				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg(t);
//				if (!pt) break;
//				buf[j] = pt->pt;
//				verticesIdx[j] = pt->id;
//				t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);
//			}
//			if (j != iPointNum) continue;
//			if (isGoodTri(verticesIdx, tri))
//			{
//				//tri.push_back(verticesIdx);  
//				polylines(imgShow, &pBuf, &iPointNum,
//					1, true, color,
//					1, CV_AA, 0);
//				//printf("(%d, %d)-(%d, %d)-(%d, %d)\n", buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);  
//				//printf("%d\t%d\t%d\n", verticesIdx[0], verticesIdx[1], verticesIdx[2]);  
//				//imshow("Delaunay", imgShow);  
//				//waitKey();  
//			}
//
//		}
//
//		CV_NEXT_SEQ_ELEM(elem_size, reader);
//
//	}
//
//	//RemoveDuplicate(tri);  
//	char title[100];
//	sprintf_s(title, 100, "Delaunay: %d Triangles", tri.size());//tri存储的为3个顶点为一个vec3i，故tri.size()表示三角形的个数。  
//	imshow(title, imgShow);
//	waitKey();
//}
//
//void main(int argc, char* argv[])
//
//{
//	Mat imgL(600, 600, CV_8UC3);
//	/************************************************************************/
//	/* Delaunay triangulation                                               */
//	/************************************************************************/
//	cout << "doing triangulation..." << endl;
//	vector<Vec3i> tri;
//
//	vector<Point2f> vec_points;
//	for (int i = 0; i < 60; i++)
//	{
//		Point2f fp = cvPoint2D32f((float)(rand() % (imgL.cols - 10)),//使点约束在距离边框10像素之内。    
//			(float)(rand() % (imgL.rows - 10)));
//		vec_points.push_back(fp);
//	}
//
//	TriSubDiv(vec_points, imgL, tri);
//
//
//}




