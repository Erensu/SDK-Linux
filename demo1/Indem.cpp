#include <iostream>
#include "imrsdk.h"
#include <thread>
#ifdef WIN32
#include <Windows.h>
#endif
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <future>

struct ImrDepthImageTarget
{
    float _cubesize;
    int _image_w;
    int _image_h;
    cv::Mat _depth_image;
    float* _deepptr;
};

void PoseCallback(int, void* pData, void* pParam) {
	ImrModulePose* pHeadPose = (ImrModulePose*)pData;
	//检查两次之间的间隔距离平方应当小于5cm
	static double sdX = 0, sdY = 0, sdZ = 0;
	double dx = (pHeadPose->_pose._position[0] - sdX);
	double dy = (pHeadPose->_pose._position[1] - sdY);
	double dz = (pHeadPose->_pose._position[2] - sdZ);
	double ds2 = dx * dx + dy * dy + dz * dz;
	sdX = pHeadPose->_pose._position[0];
	sdY = pHeadPose->_pose._position[1];
	sdZ = pHeadPose->_pose._position[2];
	//std::cout << "D: " << ds2 << std::endl;
	//EXPECT_TRUE(ds2 < 0.005);
}

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{

	cv::Mat img1=cv::Mat(height, width, CV_8UC1, pLeft);
	cv::Mat img2=cv::Mat(height, width, CV_8UC1, pRight);
	cv::imshow("left",img1);
	cv::imshow("right",img2);
	cv::waitKey(1);
	std::cout << "IMG: " << time<< std::endl;
}

void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
		//IMU temp;
		//temp.imu_time = time;
		//temp.acc[0] = accX;
		//temp.acc[1] = accY;
		//temp.acc[2] = accZ;
		//temp.gyp[0] = gyrX;
		//temp.gyp[1] = gyrY;
		//temp.gyp[2] = gyrZ;
		std::cout << "IMU: " << time<< std::endl;
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
   // ImrModulePose* pose = (ImrModulePose*)pData;
	std::cout << "SLAM: "<< std::endl;
}

void DepthImageCallback(int ret, void* pData, void* pParam) {
    ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
}

int main(){
    using namespace indem;
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };
    strcpy(config.slamPath, "slam.dll");
    config.bSlam = true;
    pSDK->Init(config);
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
	pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
	pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
    //深度解算
    //pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, this);
    std::this_thread::sleep_for(std::chrono::seconds(60 ));
    pSDK->Release();
    delete pSDK;
    return 1;
}
