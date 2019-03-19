#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <string>
#include <fstream>
#include <conio.h>

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


ColorSpacePoint depth2rgb[512 * 424];     // Maps depth pixels to rgb pixels


int main(int argc, char** argv)
{


	CreateDirectory(".\\rgb", NULL);
	CreateDirectory(".\\depth", NULL);

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


	IDepthFrameReference* myDepthFrameReference = nullptr;
	IColorFrameReference* myColorFrameReference = nullptr;
	IColorFrame* myColorFrame = nullptr;
	IDepthFrame* myDepthFrame = nullptr;



	UINT nColorBufferSize = 1920 * 1080 * 4;
	Mat i_rgb = Mat::zeros(1080, 1920, CV_8UC4);
	Mat i_depth = Mat::zeros(1080, 1920, CV_16UC1);

	unsigned int sz;
	unsigned short* buf;


	while (true)
	{
		hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);
		// 
		if (FAILED(hr) || !myMultiFrame)
		{
			cout << "trying to capture sensor data..." << endl;
			continue;
		}
		else
		{
			cout << "kinect sensor ready ..." << endl << endl;
			break;
		}
	}

	char exitflag = '\0';
	int frame_count = 0;

	ofstream rgb_txt;
	rgb_txt.open("rgb.txt");

	ofstream depth_txt;
	depth_txt.open("depth.txt");
	string added = "123";


	cv::namedWindow("color image", 0);

	while (true)
	{

		if (frame_count == 0)
		{
			frame_count++;
			continue;
		}

		else {
			hr = myMultiFrameReader->AcquireLatestFrame(&myMultiFrame);

			if (FAILED(hr) || !myMultiFrame)
			{
				cout << "trying to capture sensor data..." << endl;
				continue;
			}
		}


		std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
			std::chrono::system_clock::now().time_since_epoch()
			);

		string current_timestamp = std::to_string(ms.count());

		string  filename_rgb = ".\\rgb\\" + added + current_timestamp.substr(0, 7) +
			"." + current_timestamp.substr(7, current_timestamp.length()) + ".jpg";

		string  filename_depth = ".\\depth\\" + added + current_timestamp.substr(0, 7) +
			"." + current_timestamp.substr(7, current_timestamp.length()) + ".PNG";

		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_ColorFrameReference(&myColorFrameReference);
		if (SUCCEEDED(hr))
			hr = myColorFrameReference->AcquireFrame(&myColorFrame);
		if (SUCCEEDED(hr))
			hr = myMultiFrame->get_DepthFrameReference(&myDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = myDepthFrameReference->AcquireFrame(&myDepthFrame);
		if (SUCCEEDED(hr)) {
			hr = myColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
		if (SUCCEEDED(hr)) {
			myDepthFrame->AccessUnderlyingBuffer(&sz, &buf);
		}

		mapper->MapDepthFrameToColorSpace(512 * 424, buf, 512 * 424, depth2rgb);

		cv::resizeWindow("color image", 960, 540);
		cv::imshow("color image", i_rgb);
		//waitKey(1);

		char key_board = cvWaitKey(-1);
		if (key_board == 'y') {

			for (int i = 0; i < 424; i++) {
				for (int j = 0; j < 512; j++) {
					ColorSpacePoint p = depth2rgb[i * 512 + j];
					// Check if color pixel coordinates are in bounds
					if (p.X < 0 || p.Y < 0 || p.X > 1920 || p.Y > 1080) {
						continue;
					}
					else {

						i_depth.ptr<uint16_t>((int)p.Y)[(int)p.X] = buf[i * 512 + j];

					}

				}
			}

			cv::imwrite(filename_rgb, i_rgb);
			cv::imwrite(filename_depth, i_depth);

			rgb_txt << added + current_timestamp.substr(0, 7) + "." +
				current_timestamp.substr(7, current_timestamp.length()) << " " << "rgb" << "/"
				<< added + current_timestamp.substr(0, 7) + "." +
				current_timestamp.substr(7, current_timestamp.length())
				<< ".jpg\n";

			depth_txt << added + current_timestamp.substr(0, 7) + "." +
				current_timestamp.substr(7, current_timestamp.length()) << " " << "depth" << "/"
				<< added + current_timestamp.substr(0, 7) + "." +
				current_timestamp.substr(7, current_timestamp.length())
				<< ".png\n";
			cout << "frame " << frame_count << " saved!" << endl;

			frame_count++;
		}
		else if (key_board == 'q' || key_board == 'e')
		{
			break;
		}
		else
		{
			cout << "skipped current frame!" << endl;
		}


		// ÊÍ·Å×ÊÔ´
		SafeRelease(myColorFrame);
		SafeRelease(myColorFrameReference);
		SafeRelease(myMultiFrame);
		SafeRelease(myDepthFrame);
		SafeRelease(myDepthFrameReference);

		//if (_kbhit()) {
		//		
		//	exitflag = _getch();
		//	if (exitflag == 'q' || exitflag == 'e')
		//	{
		//		break;
		//	}
		//}

	}

	rgb_txt.close();
	depth_txt.close();

	destroyWindow("color image");

	SafeRelease(myColorFrame);
	SafeRelease(myColorFrameReference);
	SafeRelease(myDepthFrame);
	SafeRelease(myDepthFrameReference);
	SafeRelease(myMultiFrame);

	cv::destroyAllWindows();
	myKinectSensor->Close();
	std::system("pause");


	return 0;
}


