#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int main()
{
	CreateDirectory(".\\calibration_images", NULL);

	// 获取Kinect设备
	IKinectSensor* myKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&myKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}
	ICoordinateMapper* myCoordinatemapper = NULL;
	CameraIntrinsics* myCameraIntrinsics = new CameraIntrinsics();
	IMultiSourceFrameReader* myMultiFrameReader = nullptr;
	IMultiSourceFrame* myMultiFrame = nullptr;


	if (myKinectSensor)
	{
		hr = myKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// 获取多数据源到读取器
			hr = myKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared,
				&myMultiFrameReader);
		}
	}

	if (!myKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}


	cout << "初始化中..." << endl;
	while (true)
	{
		hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);
		// 持续尝试从Reader获取最新帧，直到数据流正常
		if (FAILED(hr) || !myMultiFrame)
			continue;
		else
		{
			cout << "初始化完成..." << endl << endl;
			break;
		}
	}

	// 从SDK获取深度相机内参
	if (SUCCEEDED(hr))
		hr = myKinectSensor->get_CoordinateMapper(&myCoordinatemapper);

	if (SUCCEEDED(hr))
		hr = myCoordinatemapper->GetDepthCameraIntrinsics(myCameraIntrinsics);

	if (SUCCEEDED(hr))
	{
		cout << endl << "深度相机内参为：" << endl;
		cout << "FocalLengthX : " << myCameraIntrinsics->FocalLengthX << endl;
		cout << "FocalLengthY : " << myCameraIntrinsics->FocalLengthY << endl;
		cout << "PrincipalPointX : " << myCameraIntrinsics->PrincipalPointX << endl;
		cout << "PrincipalPointY : " << myCameraIntrinsics->PrincipalPointY << endl;
		cout << "RadialDistortionFourthOrder : " << myCameraIntrinsics->RadialDistortionFourthOrder << endl;
		cout << "RadialDistortionSecondOrder : " << myCameraIntrinsics->RadialDistortionSecondOrder << endl;
		cout << "RadialDistortionSixthOrder : " << myCameraIntrinsics->RadialDistortionSixthOrder << endl << endl << endl;
	}


	// 三个数据帧及引用
	IColorFrameReference* myColorFrameReference = nullptr;
	IInfraredFrameReference* myInfraredFrameReference = nullptr;
	IInfraredFrame* myInfraredFrame = nullptr;
	IColorFrame* myColorFrame = nullptr;

	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_ir(424, 512, CV_16UC1);
	int frame_count = 1;


	while (true)
	{
		// 设置存储文件名
		char infrared_image_name[200] = { '\0' };
		char rgb_image_name[200] = { '\0' };

		sprintf(infrared_image_name, "%s%d%s", ".\\calibration_images\\depth_", frame_count, ".tif");
		sprintf(rgb_image_name, "%s%d%s", ".\\calibration_images\\rgb_", frame_count, ".jpg");


		hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);

		// 尝试从Reader获取最新帧
		if (FAILED(hr) || !myMultiFrame)
		{
			cout << "获取失败，持续尝试中..." << endl;
			continue;
		}

		// 从多源数据帧中分离出彩色数据和红外数据
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_ColorFrameReference(&myColorFrameReference);
		if (SUCCEEDED(hr))
			hr = myColorFrameReference->AcquireFrame(&myColorFrame);
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_InfraredFrameReference(&myInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = myInfraredFrameReference->AcquireFrame(&myInfraredFrame);

		// color拷贝到图片中
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = myColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// infrared拷贝到图片中
		if (SUCCEEDED(hr))
			hr = myInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));

		// 显示 该帧 彩色图 红外图
		cvNamedWindow("color image!", 0);
		cvResizeWindow("color image!", i_rgb.cols / 2, i_rgb.rows / 2);
		cvMoveWindow("color image!", 0, 200);

		cvNamedWindow("infrare image!");
		cvMoveWindow("infrare image!", i_rgb.cols / 2, 200);

		imshow("color image!", i_rgb);
		imshow("infrare image!", i_ir);
		 

		//cout << "是否保存当前帧(y n )" << endl;
		//cout << "按ESC结束采集并退出" << endl;
		
		

		char keycommand = '0';
		keycommand = waitKey();
		if (keycommand == 'y') {
			imwrite(rgb_image_name, i_rgb);
			imwrite(infrared_image_name, i_ir);
			frame_count++;
		}
		else if (keycommand == 'n')
			continue;
		else if (keycommand == VK_ESCAPE)
			break;


		// 释放资源
		SafeRelease(myColorFrame);
		SafeRelease(myInfraredFrame);
		SafeRelease(myColorFrameReference);
		SafeRelease(myInfraredFrameReference);
		SafeRelease(myMultiFrame);
	}
	// 关闭窗口，设备
	cv::destroyAllWindows();
	myKinectSensor->Close();
	// 命令行窗口保持， 防止程序结束， 命令行窗口闪退。
	std::system("pause");
	return 0;
}
