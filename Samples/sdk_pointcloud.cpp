#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// release resource
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
ColorSpacePoint depth2rgb[512*424];     // Maps depth pixels to rgb pixels
CameraSpacePoint depth2xyz[512*424];    // Maps depth pixels to 3d coordinates

PointCloud::Ptr cloud(new PointCloud);

int main()
{

	IKinectSensor* myKinectSensor;
	ICoordinateMapper* mapper;
	HRESULT hr;

	hr = GetDefaultKinectSensor(&myKinectSensor);
	if (FAILED(hr))
	{
		std::cout << "Get Sensor Filed." << endl;
		return hr;
	}	
	
	IMultiSourceFrameReader* myMultiFrameReader = nullptr;
	IMultiSourceFrame* myMultiFrame = nullptr;
	
		
	if (myKinectSensor)
	{
		myKinectSensor->get_CoordinateMapper(&mapper);
		hr = myKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			hr = myKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&myMultiFrameReader);
		}
	}
		
	if (!myKinectSensor || FAILED(hr))
	{
		std::cout << "Open Sensor Failed." << endl;
		return E_FAIL;
	}
	
	IColorFrameReference* myColorFrameReference = nullptr;
	IDepthFrameReference* myDepthFrameReference = nullptr;
	IColorFrame* myColorFrame = nullptr;
	IDepthFrame* myDepthFrame = nullptr;

	UINT nColorBufferSize = 1920 * 1080 * 4;
	Mat i_rgb(1080, 1920, CV_8UC4);
	Mat i_depth(424, 512, CV_16UC1);

	unsigned int sz;
	unsigned short* buf;


	while (true)
	{
		hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);
		//
		if (FAILED(hr) || !myMultiFrame) {
			std::cout << "trying capture data..." << endl;
			continue;
		}
				
		else
		{
			std::cout << "kinect sensor ready ..." << endl;
			if (SUCCEEDED(hr))
				hr = myMultiFrame->get_ColorFrameReference(&myColorFrameReference);
			if (SUCCEEDED(hr))
				hr = myColorFrameReference->AcquireFrame(&myColorFrame);
			if (SUCCEEDED(hr))
				hr = myMultiFrame->get_DepthFrameReference(&myDepthFrameReference);
			if (SUCCEEDED(hr))
				hr = myDepthFrameReference->AcquireFrame(&myDepthFrame);

			//
			if (SUCCEEDED(hr)) {
				hr = myColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

			}
			if (SUCCEEDED(hr)) {
				//hr = myDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
				myDepthFrame->AccessUnderlyingBuffer(&sz, &buf);
			}

			break;
		}
	}
	
	mapper->MapDepthFrameToCameraSpace(512 * 424, buf, 512 * 424, depth2xyz);
	mapper->MapDepthFrameToColorSpace(512 * 424, buf, 512 * 424, depth2rgb);

	for (int i = 0; i < 512 * 424; i++) {
		ColorSpacePoint p = depth2rgb[i];
		CameraSpacePoint p_xyz = depth2xyz[i];
		PointT p_xyzrgb;
		
		// Check if color pixel coordinates are in bounds
		if (p.X < 0 || p.Y < 0 || p.X > 1920 || p.Y > 1080) {
			continue;
		}
		else {
			//int idx = (int)p.X + 1080 * (int)p.Y;
			Vec4b p_rgba =  i_rgb.at<Vec4b>(p.Y, p.X);

			
			p_xyzrgb.b = p_rgba[0];
			p_xyzrgb.g = p_rgba[1];
			p_xyzrgb.r = p_rgba[2];

			p_xyzrgb.x = p_xyz.X;
			p_xyzrgb.y = p_xyz.Y;
			p_xyzrgb.z = p_xyz.Z;
		}
		cloud->push_back(p_xyzrgb);
	}
		
		
	cloud->height = 1;
	cloud->width = cloud->points.size();
	std::cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePLYFile("pointcloud_sdk.ply", *cloud);
	cloud->points.clear();
	std::cout << "Point cloud saved." << endl;
			
	SafeRelease(myColorFrame);
	SafeRelease(myDepthFrame);
	SafeRelease(myColorFrameReference);
	SafeRelease(myDepthFrameReference);
	SafeRelease(myMultiFrame);
		
		
	cv::destroyAllWindows();
	myKinectSensor->Close();
	std::system("pause");

	return 0;
}


