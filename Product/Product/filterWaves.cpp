//#include "MyPCL.h"
//
//int main(int argc, char** argv)
//{
//	/*创建点云数据集。*/
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
//	cloud1->width = 500;
//	cloud1->height = 1;
//	cloud1->points.resize(cloud1->width*cloud1->height);
//	std::cout << "创建原始点云数据" << std::endl;
//	for (size_t i = 0; i < cloud1->points.size(); ++i)
//	{
//		cloud1->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud1->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
//		cloud1->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
//	}
//	for (size_t i = 0; i < cloud1->points.size(); i++)
//	{
//		std::cerr << " " << cloud1->points[i].x << " "
//			<< cloud1->points[i].y << " "
//			<< cloud1->points[i].z << std::endl;
//	}
//	std::cout << "原始点云数据点数：" << cloud1->points.size() << std::endl ;
//	pcl::io::savePCDFile("yuanshi.pcd", *cloud1);
//	cout << "Point cloud saved." << endl;
//
//	//读取点云数据
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云（指针）
//	if (pcl::io::loadPCDFile<pcl::PointXYZI>("yuanshi.pcd", *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//	{
//		std::cout<<"Couldn't read file test_pcd.pcd \n"<<std::endl; //文件不存在时，返回错误，终止程序。
//		system("pause");
//		return (-1);
//	}
//	std::cout << "Loaded "<< cloud->width * cloud->height	//总点数
//		<< " data points from test_file.pcd with the following fields."<< std::endl;
//	//试显示前5个的点
//	for (size_t i = 0; i < 5; ++i){
//		std::cerr << " " << cloud->points[i].x
//			<< " " << cloud->points[i].y
//			<< " " << cloud->points[i].z << std::endl;
//	}
//	pcl::visualization::CloudViewer viewer("pcd_before viewer");//显示点云
//	viewer.showCloud(cloud);
//	system("pause");
//
//
//
//	/*方法：直通滤波器对点云进行处理。*/
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::PassThrough<pcl::PointXYZI> passthrough;
//	passthrough.setInputCloud(cloud);//输入点云
//	passthrough.setFilterFieldName("I");//对z轴进行操作
//	passthrough.setFilterLimits(0.0, 400.0);//设置直通滤波器操作范围
//	passthrough.setFilterLimitsNegative(true);//true表示保留范围内，false表示保留范围外
//	passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
//	std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
//	//显示点云数据
//	pcl::visualization::CloudViewer viewer_limits("pcd viewer");//给显示窗口命名，CloudViewer
//	viewer_limits.showCloud(cloud_after_PassThrough);//定义要显示的对象,showCloud
//	system("pause");//此处防止显示闪退
//	//保存点云数据
//	pcl::io::savePCDFile("after.pcd", *cloud_after_PassThrough);
//	cout << "Point cloud saved." << endl;
//	
//	/*s
//	//方法： 双边滤波
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_bilaterfilter(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::FastBilateralFilter<pcl::PointXYZI> filter;
//	filter.setInputCloud(could);
//	filter.setSigmaS(0.5);
//	filter.setSigmaR(0.004);
//	filter.applyFilter(*cloud_after_bilaterfilter);
//	std::cout << "双边滤波方法后点云数据点数：" << cloud_after_bilaterfilter->points.size() << std::endl;
//
//
//	方法：体素滤波器实现下采样
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZI>);//
//	pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
//	voxelgrid.setInputCloud(cloud);//输入点云数据
//	voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);//AABB长宽高
//	voxelgrid.filter(*cloud_after_voxelgrid);
//	std::cout << "体素化网格方法后点云数据点数：" << cloud_after_voxelgrid->points.size() << std::endl;
//
//	方法：统计滤波器滤波
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZI>);//
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> Statistical;
//	Statistical.setInputCloud(cloud);
//	Statistical.setMeanK(20);//取平均值的临近点数
//	Statistical.setStddevMulThresh(5);//临近点数数目少于多少时会被舍弃
//	Statistical.filter(*cloud_after_StatisticalRemoval);
//	std::cout << "统计分析滤波后点云数据点数：" << cloud_after_StatisticalRemoval->points.size() << std::endl;
//
//	方法：条件滤波器
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZI>());
//	range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
//		pcl::FieldComparison<pcl::PointXYZI>("I", pcl::ComparisonOps::GT, 0.0)));  //GT表示大于等于
//	range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
//		pcl::FieldComparison<pcl::PointXYZI>("I", pcl::ComparisonOps::LT, 0.8)));  //LT表示小于等于
//	pcl::ConditionalRemoval<pcl::PointXYZI> condition;
//	condition.setCondition(range_condition);
//	condition.setInputCloud(cloud);                   //输入点云
//	condition.setKeepOrganized(true);
//	condition.filter(*cloud_after_Condition);
//	std::cout << "条件滤波后点云数据点数：" << cloud_after_Condition->points.size() << std::endl;
//
//	方法：半径滤波器
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusoutlier;  //创建滤波器
//	radiusoutlier.setInputCloud(cloud);    //设置输入点云
//	radiusoutlier.setRadiusSearch(100);     //设置半径为100的范围内找临近点
//	radiusoutlier.setMinNeighborsInRadius(2); //设置查询点的邻域点集数小于2的删除
//	radiusoutlier.filter(*cloud_after_Radius);
//	std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
//	*/
//
//
//	int a;
//	std::cin >> a;
//	return 0;
//
//}