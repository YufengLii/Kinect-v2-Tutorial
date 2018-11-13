#include <kinect.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

using namespace cv;
using namespace std;


int main() {

	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_16UC1);

	i_rgb = imread(".\\calibration_images\\rgb_1.jpg");
	i_depth = imread(".\\calibration_images\\depth_1.png",-1);

	imshow("depth",i_depth);
	cvWaitKey(10);

	double ir_fx = 367.796;
	double ir_fy = 367.796;

	double ir_cx = 252.624;
	double ir_cy = 208.879;

	Point3d xyzpoint;
	vector<Point3d> pointCloud;

	// ����ͼת����
	for ( int u = 0; u < i_depth.rows; u++) {
		for ( int v = 0; v < i_depth.cols; v++){

			ushort d = i_depth.ptr<ushort>(u)[v];
			if (d == 0)
			continue;
			xyzpoint.x = (v - ir_cx) * d / (ir_fx * 1000);
			xyzpoint.y = (u - ir_cy) * d / ir_fy / 1000;
			xyzpoint.z = (d / 1000.0);

			pointCloud.push_back(xyzpoint);
		}
	}
	// �洢
	ofstream outfile("pointcloud.ply");
	outfile << "ply\n" << "format ascii 1.0\n" << "comment VTK generated PLY File\n";
	outfile << "obj_info vtkPolyData points and polygons : vtk4.0\n" << "element vertex " << pointCloud.size() << "\n";
	outfile << "property float x\n" << "property float y\n" << "property float z\n" << "element face 0\n";
	outfile << "property list uchar int vertex_indices\n" << "end_header\n";
	for (int i = 0; i < pointCloud.size(); i++)
	{
		Point3d point = pointCloud.at(i);
		outfile << point.x << " ";
		outfile << point.y << " ";
		outfile << point.z << " ";
		outfile << "\n";
	}
	outfile.close();
	return 0;
}
