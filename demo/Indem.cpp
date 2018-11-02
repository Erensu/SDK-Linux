#include <iostream>
#include <gtest/gtest.h>
#include "imrsdk.h"
#include <thread>
#include <queue>
#include <mutex>
#include <fstream>
#include <future>

int g_iImageWidth = 0, g_iImageHeight = 0,g_r_num=0,g_o_num=0,g_num=0;

double imgTime = 0.0;
std::atomic<bool> g_stop(false);
std::atomic<bool> g_save(false);   //是否存储数据到文件


struct ImrDepthImageTarget
{
	float _cubesize;
	int _image_w;
	int _image_h;
	float* _deepptr;
};

void HotPlugCallback(int iType, void* pParam) {
	if (iType) {
		std::cout << "Device Connect Success!";
	}
	else {
		std::cout << "Device Connect Fail!";
	}
}

void SeeThrougthCallback(int ret, void* pData, void* pParam) {
	ImrImage* pImage = reinterpret_cast<ImrImage*>(pData);

}
void DepthImageCallback(int ret, void* pData, void* pParam) {
	ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
}
void GetCameraCallback(double, ImrImage pData, void*) {
	EXPECT_TRUE(pData._image != NULL);
}

void HMDPoseCallback(int, void* pData, void* pParam) {
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

void PlaneCallback(ImrRecogPlaneTarget* res, void* pParam) {
	if (res->_planecount) {
		for (int idx = 0; idx < res->_planecount; ++idx) {
			//EXPECT_TRUE(res->_targets[idx]._pointNum > 0);
		}
	}
}

void SenmiticCallback(int ret, void* pData, void* pParam) {
	ImrRecognizeResult* res = (ImrRecognizeResult*)pData;
	if (res->_num > 0) {
		std::cout << "SenmiticMap: " << res->_num << std::endl;
	}
}

struct ImrSize {
	int32_t _width;
	int32_t _height;
};

void GetIMUCallback(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
	//	printf("%-18.2f   %-18f   %-18f   %-18f   %-18f   %-18f   %-18f\r\n", time, accX, accY, accZ, gyrX, gyrY, gyrZ);
}

void GetModuleCameraCallback(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
}

indem::CIMRSDK* CreateImrSDK(bool bOpenSlam, bool bDataSet = false)
{
	indem::CIMRSDK* pSDK = new indem::CIMRSDK();
	indem::MRCONFIG config = { 0 };

	//Init SDK
	config.bSlam = bOpenSlam;
	bool flag = pSDK->Init(config);
	if (flag) {
		std::cout << "Init SDK success!" << std::endl;
		return pSDK;
	}
	else
	{
		std::cout << "Init SDK failed!" << std::endl;
		pSDK->Release();
	}
	return NULL;
}

void IMUDataCallbackFunction(imrIMUData* data) {
	//std::cout << "data->_timeStamp===" << data->_timeStamp << std::endl;
}

void CameraCallbackFunction(imrCameraData* data)
{
}

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
}
double last_imutime = 0.0;
void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
	std::cout << std::setprecision(10) << time;
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
}

#if 0
/*Test: IMU data callback*/
TEST(Indem, ModuleIMUTest) {
	indem::CIMRSDK* pSDK = CreateImrSDK(true);
	if (pSDK != NULL) {
		g_stop = false;
		pSDK->RegistModuleIMUCallback(GetIMUCallback, NULL);

		std::this_thread::sleep_for(std::chrono::seconds(12*60*60));

		std::cout << "-----------------------END1--------------------" << std::endl;
		pSDK->RegistModuleIMUCallback(NULL, NULL);
		pSDK->Release();
		std::cout << "-----------------------END2--------------------" << std::endl;

		g_stop = true;

		std::cout << "g_r_num=" << g_r_num << ",g_o_num=" << g_o_num << std::endl;
	}
	std::cout << "-------------------ModuleIMUTest END----------------" << std::endl;
}
#endif

#if 0
/*Test:  camera data callback*/
TEST(Indem, ModuleCameraTest) {
    indem::CIMRSDK* pSDK = CreateImrSDK(true);
    std::cout << "-----------------------CreateImrSDK--------------------" << std::endl;
    if (pSDK != NULL) {
        pSDK->RegistModuleCameraCallback(GetModuleCameraCallback, NULL);

        std::this_thread::sleep_for(std::chrono::seconds(60));

        std::cout << "-----------------------END1--------------------" << std::endl;
        //pSDK->RegistModuleCameraCallback(NULL, NULL);
        pSDK->Release();
        std::cout << "-----------------------END2--------------------" << std::endl;
        delete pSDK;

        g_stop = true;
        threadSaveImage.join();
    }
}
#endif

#if 0
/*Test: DepthImage*/
TEST(Indem, ModuleDepthImageTest) {
	indem::CIMRSDK* pSDK = CreateImrSDK(true);
	std::cout << "-----------------------CreateImrSDK--------------------" << std::endl;
	if (pSDK != NULL) {
		pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, NULL);

		std::this_thread::sleep_for(std::chrono::seconds(60));

		std::cout << "-----------------------END1--------------------" << std::endl;
		pSDK->AddPluginCallback("depthimage", "depth", NULL, NULL);
		pSDK->Release();
		std::cout << "-----------------------END2--------------------" << std::endl;
	}
	std::cout << "-------------------ModuleCameraTest END----------------" << std::endl;
}
#endif

#if 0
TEST(Indem, driver) {//存储图像数据的Main函数
	g_stop = false;
	IMRDEVICE_HANDLE handlfd = imrOpenIMR();

	CameraConfig cc;
	cc._cb = CameraCallbackFunction;
	imrInitCamera(handlfd, cc);
	imrStartCamera(handlfd);

	imrConnectDevice(handlfd, NULL, 0, IMUDataCallbackFunction);

	sleep(1 * 60 * 60);
	g_stop = true;

	imrCloseIMR(NULL);
	std::cout << "TestDriverFinish" << std::endl;
}
#endif

#if 0
TEST(Indem, Initailize) {
    g_stop = false;
    using namespace indem;
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };

    config.bSlam = true;
    EXPECT_TRUE(pSDK->Init(config));

	pSDK->RegistModuleCameraCallback(SdkCameraCallBack,NULL);
	pSDK->RegistModuleIMUCallback(sdkImuCallBack,NULL);
	//pSDK->RegistModulePoseCallback(sdkSLAMResult,NULL);
	//pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, this);


    std::this_thread::sleep_for(std::chrono::seconds(60 * 60 * 24));

    std::cout << "-----------------------END1--------------------" << std::endl;

    pSDK->Release();
    std::cout << "-----------------------END2--------------------" << std::endl;
    delete pSDK;

    g_stop = true;
}
#endif


//深度解算   不跑Slam
#if 0
TEST(Testidnem,Initialize){
	g_stop = false;
	using namespace indem;

	CIMRSDK* pSDK = new CIMRSDK();
	MRCONFIG m_config = { 0 };

	m_config.imgFreq = 60;
	m_config.bSlam = true;
	EXPECT_TRUE(pSDK->Init(m_config));
	pSDK->RegistModuleCameraCallback(SdkCameraCallBack, NULL);
	pSDK->RegistModuleIMUCallback(sdkImuCallBack, NULL);
	pSDK->RegistModulePoseCallback(sdkSLAMResult, NULL);
	//pSDK->AddPluginCallback("depthimage", "depth", DepthImageCallback, this);
	std::this_thread::sleep_for(std::chrono::seconds(60 * 60));
	std::cout << "END!" << std::endl;

	pSDK->Release();
	delete pSDK;
	g_stop = true;
}
#endif 

//读取标定参数和设备信息
#if 0
TEST(Testidnem, GetInfo) 
{
	using namespace indem;

	CIMRSDK* pSDK = new CIMRSDK();

	MRCONFIG config = { 0 };
	config.imgFreq = 60;
	config.bSlam = false;
	EXPECT_TRUE(pSDK->Init(config));
	ImrModuleDeviceInfo info = pSDK->GetModuleInfo();
	std::cout << info._designer << std::endl;
	std::cout << info._id << std::endl;
	CameraCalibrationParameter ccp = pSDK->GetModuleParams();
	std::cout << ccp._baseline << std::endl;
}
#endif 