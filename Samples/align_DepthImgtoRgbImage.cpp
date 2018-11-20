#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

struct cameraIntrisc {
	double fx = 0.0;
	double fy = 0.0;
	double cx = 0.0;
	double cy = 0.0;
};


int main(int argc, char** argv)
{

	cameraIntrisc ir_camera, rgb_camera;

	ir_camera.fx = 361.56703;
	ir_camera.fy = 362.39493;
	ir_camera.cx = 251.25639;
	ir_camera.cy = 204.98329;

	rgb_camera.fx = 1057.39764;
	rgb_camera.fy = 1056.94703;
	rgb_camera.cx = 941.19365;
	rgb_camera.cy = 533.31857;

	Mat Img_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat Img_depth(424, 512, CV_16UC1);

	Img_rgb = imread(argv[1]);
	Img_depth = imread(argv[2], -1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ir(new pcl::PointCloud<pcl::PointXYZ>);;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


	int rowNumber = Img_depth.rows;
	int colNumber = Img_depth.cols;


	int point_num = 0;

	for (unsigned int u = 0; u < rowNumber; ++u)
	{
		for (unsigned int v = 0; v < colNumber; ++v)
		{
			pcl::PointXYZ ir_point;
			if (Img_depth.ptr<ushort>(u)[v] == 0)
				continue;

			double X_ir = 0, Y_ir = 0, Z_ir = 0;

			Z_ir = ((double)Img_depth.at<uchar>(u, v)) / 1000.0;
			X_ir = (u - ir_camera.cx) * Z_ir / ir_camera.fx;
			Y_ir = (v - ir_camera.cy) * Z_ir / ir_camera.fy;

			ir_point.x = X_ir;
			ir_point.y = Y_ir;
			ir_point.z = Z_ir;

			cloud_ir->push_back(ir_point);
			point_num++;
		}
	}

	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

	transform_matrix << 0.9998, 0.0095, 0.0152, 56.15800 / 1000,
		-0.0094, 0.9999, -0.0055, -0.52648 / 1000,
		-0.0152, 0.0054, 0.9999 - 3.59107 / 1000,
		0, 0, 0, 1;
	//Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
	//Eigen::Matrix3f rotation_matrix = Eigen::Matrix3d::Identity();
	//rotation_matrix << 0.9998, 0.0095, 0.0152,
	//					-0.0094, 0.9999, -0.0055, 
	//					-0.0152, 0.0054, 0.9999;
	//transform_matrix.translation() << 56.15800/1000, - 0.52648/1000, - 3.59107/1000;
	//transform_matrix.rotate(Eigen::Quaterniond(rotation_matrix));


	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_ir_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud_ir, *transformed_ir_cloud, transform_matrix);


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgba_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

	for (int i = 0; i < transformed_ir_cloud->size(); i++) {

		int u = (rgb_camera.fx* (*transformed_ir_cloud)[i].x +
			rgb_camera.cx * (*transformed_ir_cloud)[i].z) / (*transformed_ir_cloud)[i].z;
		int v = (rgb_camera.fy*(*transformed_ir_cloud)[i].y +
			rgb_camera.cy * (*transformed_ir_cloud)[i].z) / (*transformed_ir_cloud)[i].z;

		Vec4b uv_rgba = Img_rgb.at<Vec4b>(u, v);

		pcl::PointXYZRGBA point_xyzrgba;

		point_xyzrgba.x = (*transformed_ir_cloud)[i].x;
		point_xyzrgba.y = (*transformed_ir_cloud)[i].y;
		point_xyzrgba.z = (*transformed_ir_cloud)[i].z;
		point_xyzrgba.r = uv_rgba[0];
		point_xyzrgba.g = uv_rgba[1];
		point_xyzrgba.b = uv_rgba[2];
		point_xyzrgba.a = uv_rgba[3];

		rgba_cloud->push_back(point_xyzrgba);
	}


	pcl::io::savePCDFile("test_pcd.pcd", *rgba_cloud);
	cout << transform_matrix << endl;

	return 0;
}