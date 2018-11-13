#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

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

struct kienctframedata
{
	Mat depth;
	Mat color;
	Mat ir;
};


int main()
{
	CreateDirectory(".\\captured_images", NULL);

	// 
	IKinectSensor* myKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&myKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* myMultiFrameReader = nullptr;
	IMultiSourceFrame* myMultiFrame = nullptr;


	if (myKinectSensor)
	{
		hr = myKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// 
			hr = myKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared | 
				FrameSourceTypes::FrameSourceTypes_Depth,
				&myMultiFrameReader);
		}
	}

	if (!myKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	while (true)
	{
		hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);
		// 
		if (FAILED(hr) || !myMultiFrame)
			continue;
		else
		{
			cout << "kinect sensor ready ..." << endl << endl;
			break;
		}
	}

	// 
	IColorFrameReference* myColorFrameReference = nullptr;
	IInfraredFrameReference* myInfraredFrameReference = nullptr;
	IDepthFrameReference* myDepthFrameReference = nullptr;
	IInfraredFrame* myInfraredFrame = nullptr;
	IColorFrame* myColorFrame = nullptr;
	IDepthFrame* myDepthFrame = nullptr;

	// 
	UINT nColorBufferSize = 1920 * 1080 * 4;
	Mat i_rgb(1080, 1920, CV_8UC4);      //
	Mat i_ir(424, 512, CV_16UC1);
	Mat i_depth(424, 512, CV_16UC1);

	int frame_count = 1;

	kienctframedata kienct100frame[100];
	while (true)
	{
		if (frame_count == 1)
			;
		else {
			hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);
			// 
			if (FAILED(hr) || !myMultiFrame)
			{
				cout << "trying to capture sensor data..." << endl;
				continue;
			}
		}
			
		// 
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_ColorFrameReference(&myColorFrameReference);
		if (SUCCEEDED(hr))
			hr = myColorFrameReference->AcquireFrame(&myColorFrame);
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_InfraredFrameReference(&myInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = myInfraredFrameReference->AcquireFrame(&myInfraredFrame);
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_DepthFrameReference(&myDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = myDepthFrameReference->AcquireFrame(&myDepthFrame);

		// 
		if (SUCCEEDED(hr))
			hr = myColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		if (SUCCEEDED(hr))
			hr = myInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		if (SUCCEEDED(hr))
			hr = myInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));

		
		kienct100frame[frame_count - 1].color = i_rgb;
		kienct100frame[frame_count - 1].depth = i_depth;
		kienct100frame[frame_count - 1].ir = i_ir;
		frame_count++;

		if (frame_count == 100)
			break;

		// 显示
		cvNamedWindow("color image", 0);
		cvResizeWindow("color image", i_rgb.cols / 2, i_rgb.rows / 2);
		cvMoveWindow("color image", 0, 200);

		cvNamedWindow("infrare image");
		cvMoveWindow("infrare image", i_rgb.cols / 2, 200);

		cvNamedWindow("depth image");
		cvMoveWindow("depth image", i_rgb.cols/2 +i_ir.cols, 200);

		imshow("color image", i_rgb);
		imshow("infrare image", i_ir);
		imshow("depth image", i_depth);
		cvWaitKey(10);

		// 释放资源
		SafeRelease(myColorFrame);
		SafeRelease(myInfraredFrame);
		SafeRelease(myDepthFrame);
		SafeRelease(myColorFrameReference);
		SafeRelease(myInfraredFrameReference);
		SafeRelease(myDepthFrameReference);

		SafeRelease(myMultiFrame);

	}



	// 关闭窗口，传感器
	cv::destroyAllWindows();
	myKinectSensor->Close();

	for (int frame_num = 1; frame_num <= 100; frame_num++) {
		

		// 
		char infrared_image_name[200] = { '\0' };
		char rgb_image_name[200] = { '\0' };
		char depth_image_name[200] = { '\0' };

		sprintf(infrared_image_name, "%s%d%s", ".\\captured_images\\infrared_", frame_num, ".tif");
		sprintf(rgb_image_name, "%s%d%s", ".\\captured_images\\rgb_", frame_num, ".jpg");
		sprintf(depth_image_name, "%s%d%s", ".\\captured_images\\depth_", frame_num, ".png");

		imwrite(rgb_image_name, kienct100frame[frame_num - 1].color);
		imwrite(infrared_image_name, kienct100frame[frame_num - 1].ir);
		imwrite(depth_image_name, kienct100frame[frame_num - 1].depth);
	
	}

	// 命令行窗口保持， 防止程序结束， 命令行窗口闪退。
	std::system("pause");
	return 0;
}
