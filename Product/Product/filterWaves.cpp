//#include "MyPCL.h"
//
//int main(int argc, char** argv)
//{
//	/*�����������ݼ���*/
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
//	cloud1->width = 500;
//	cloud1->height = 1;
//	cloud1->points.resize(cloud1->width*cloud1->height);
//	std::cout << "����ԭʼ��������" << std::endl;
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
//	std::cout << "ԭʼ�������ݵ�����" << cloud1->points.size() << std::endl ;
//	pcl::io::savePCDFile("yuanshi.pcd", *cloud1);
//	cout << "Point cloud saved." << endl;
//
//	//��ȡ��������
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // �������ƣ�ָ�룩
//	if (pcl::io::loadPCDFile<pcl::PointXYZI>("yuanshi.pcd", *cloud) == -1) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
//	{
//		std::cout<<"Couldn't read file test_pcd.pcd \n"<<std::endl; //�ļ�������ʱ�����ش�����ֹ����
//		system("pause");
//		return (-1);
//	}
//	std::cout << "Loaded "<< cloud->width * cloud->height	//�ܵ���
//		<< " data points from test_file.pcd with the following fields."<< std::endl;
//	//����ʾǰ5���ĵ�
//	for (size_t i = 0; i < 5; ++i){
//		std::cerr << " " << cloud->points[i].x
//			<< " " << cloud->points[i].y
//			<< " " << cloud->points[i].z << std::endl;
//	}
//	pcl::visualization::CloudViewer viewer("pcd_before viewer");//��ʾ����
//	viewer.showCloud(cloud);
//	system("pause");
//
//
//
//	/*������ֱͨ�˲����Ե��ƽ��д���*/
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::PassThrough<pcl::PointXYZI> passthrough;
//	passthrough.setInputCloud(cloud);//�������
//	passthrough.setFilterFieldName("I");//��z����в���
//	passthrough.setFilterLimits(0.0, 400.0);//����ֱͨ�˲���������Χ
//	passthrough.setFilterLimitsNegative(true);//true��ʾ������Χ�ڣ�false��ʾ������Χ��
//	passthrough.filter(*cloud_after_PassThrough);//ִ���˲������˽�������� cloud_after_PassThrough
//	std::cout << "ֱͨ�˲���������ݵ�����" << cloud_after_PassThrough->points.size() << std::endl;
//	//��ʾ��������
//	pcl::visualization::CloudViewer viewer_limits("pcd viewer");//����ʾ����������CloudViewer
//	viewer_limits.showCloud(cloud_after_PassThrough);//����Ҫ��ʾ�Ķ���,showCloud
//	system("pause");//�˴���ֹ��ʾ����
//	//�����������
//	pcl::io::savePCDFile("after.pcd", *cloud_after_PassThrough);
//	cout << "Point cloud saved." << endl;
//	
//	/*s
//	//������ ˫���˲�
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_bilaterfilter(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::FastBilateralFilter<pcl::PointXYZI> filter;
//	filter.setInputCloud(could);
//	filter.setSigmaS(0.5);
//	filter.setSigmaR(0.004);
//	filter.applyFilter(*cloud_after_bilaterfilter);
//	std::cout << "˫���˲�������������ݵ�����" << cloud_after_bilaterfilter->points.size() << std::endl;
//
//
//	�����������˲���ʵ���²���
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZI>);//
//	pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
//	voxelgrid.setInputCloud(cloud);//�����������
//	voxelgrid.setLeafSize(10.0f, 10.0f, 10.0f);//AABB�����
//	voxelgrid.filter(*cloud_after_voxelgrid);
//	std::cout << "���ػ����񷽷���������ݵ�����" << cloud_after_voxelgrid->points.size() << std::endl;
//
//	������ͳ���˲����˲�
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_StatisticalRemoval(new pcl::PointCloud<pcl::PointXYZI>);//
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> Statistical;
//	Statistical.setInputCloud(cloud);
//	Statistical.setMeanK(20);//ȡƽ��ֵ���ٽ�����
//	Statistical.setStddevMulThresh(5);//�ٽ�������Ŀ���ڶ���ʱ�ᱻ����
//	Statistical.filter(*cloud_after_StatisticalRemoval);
//	std::cout << "ͳ�Ʒ����˲���������ݵ�����" << cloud_after_StatisticalRemoval->points.size() << std::endl;
//
//	�����������˲���
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZI>());
//	range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
//		pcl::FieldComparison<pcl::PointXYZI>("I", pcl::ComparisonOps::GT, 0.0)));  //GT��ʾ���ڵ���
//	range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new
//		pcl::FieldComparison<pcl::PointXYZI>("I", pcl::ComparisonOps::LT, 0.8)));  //LT��ʾС�ڵ���
//	pcl::ConditionalRemoval<pcl::PointXYZI> condition;
//	condition.setCondition(range_condition);
//	condition.setInputCloud(cloud);                   //�������
//	condition.setKeepOrganized(true);
//	condition.filter(*cloud_after_Condition);
//	std::cout << "�����˲���������ݵ�����" << cloud_after_Condition->points.size() << std::endl;
//
//	�������뾶�˲���
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusoutlier;  //�����˲���
//	radiusoutlier.setInputCloud(cloud);    //�����������
//	radiusoutlier.setRadiusSearch(100);     //���ð뾶Ϊ100�ķ�Χ�����ٽ���
//	radiusoutlier.setMinNeighborsInRadius(2); //���ò�ѯ�������㼯��С��2��ɾ��
//	radiusoutlier.filter(*cloud_after_Radius);
//	std::cout << "�뾶�˲���������ݵ�����" << cloud_after_Radius->points.size() << std::endl;
//	*/
//
//
//	int a;
//	std::cin >> a;
//	return 0;
//
//}