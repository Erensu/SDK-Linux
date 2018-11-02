#include <iostream>
#include <gtest/gtest.h>
#include "imrsdk.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include "imr_driver.h"
#ifdef WIN32
#include <Windows.h>
#endif
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include <queue>
#include <mutex>
#include <fstream>
#include <future>
#include <boost/shared_array.hpp>

int g_iImageWidth = 0, g_iImageHeight = 0,g_r_num=0,g_o_num=0,g_num=0;

double imgTime = 0.0;
std::atomic<bool> g_stop(false);
std::atomic<bool> g_save(false);   //是否存储数据到文件

struct IMG
{
	cv::Mat image;
	double img_time = 0.0;
};
struct IMU
{
	double imu_time = 0.0;
	double current_imgTime = 0.0;
	float acc[3];
	float gyp[3];
};
struct ImrDepthImageTarget
{
	float _cubesize;
	int _image_w;
	int _image_h;
	cv::Mat _depth_image;
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
	//cv::Mat img(pImage->_height, pImage->_width, CV_8UC1, pImage->_image);
	//imwrite("D:/indemTest.jpg", img);
}
void DepthImageCallback(int ret, void* pData, void* pParam) {
	//std::cout << "---------iiiiiiiiiiiiiii-----------" << std::endl;
	ImrDepthImageTarget* Depth = reinterpret_cast<ImrDepthImageTarget*>(pData);
	if (g_save)
	{
#ifdef WIN32
		//cv::imwrite("D:/TEST/TEst.png", Depth->_depth_image);
#else
		//static int picnumber = 0;
		//++picnumber;
		//std::string name = "imgs/" + std::to_string(picnumber)+ ".png";
		//cv::imwrite(name, Depth->_depth_image);
#endif // WIN32
	}
	//std::cout << "  1  " << std::endl;
	//cv::imshow("depthimage.png", Depth->_depth_image);
	//cv::waitKey(1);
}
void GetCameraCallback(double, ImrImage pData, void*) {
	//EXPECT_TRUE(pData._height == g_iImageHeight);
	//EXPECT_TRUE(pData._width == g_iImageWidth);
	EXPECT_TRUE(pData._image != NULL);
	//cv::Mat img(pData._height, pData._width, CV_8UC1, pData._image);
	//imwrite("D:/indemTest.jpg", img);
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
	if (g_save)
	{
		LOG(INFO) << std::setiosflags(std::ios::fixed) << std::setprecision(14) << std::setiosflags(std::ios::showpoint) 
			<< temp.imu_time << " ,"
			<< temp.gyp[0] << " ," << temp.gyp[1] << " ," << temp.gyp[2] << " ,"
			<< temp.acc[0] << " ," << temp.acc[1] << " ," << temp.acc[2] ;
	}
	//	printf("%-18.2f   %-18f   %-18f   %-18f   %-18f   %-18f   %-18f\r\n", time, accX, accY, accZ, gyrX, gyrY, gyrZ);
}

void GetModuleCameraCallback(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	//std::string filename = "imgs/" + std::to_string(time) + ".jpg";
	//cv::Mat img(height, width, CV_8UC1, pLeft);
	//imwrite(filename, img);
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
	if (g_save)
	{
	}
	//std::cout << "data->_timeStamp===" << data->_timeStamp << std::endl;
}

void CameraCallbackFunction(imrCameraData* data)
{
	if (g_save)
	{
		if (data == nullptr)
		{
			assert(data != nullptr);
			std::cout << "img_data is NULL " << std::endl;
		}
		else
		{
			/*uchar *image_uchar = data->_image;
			cv::Mat img = cv::Mat(800, 2560, CV_8UC3, data->_image);*/
			//boost::shared_array<unsigned char> _data;
			//_data = boost::shared_array<unsigned char>(new unsigned char[800 * 2560]);
			//memcpy(_data.get(), data->_image, 800 * 2560 * sizeof(unsigned char));
			//cv::Mat img(800, 2560, CV_8UC1, _data.get());
			IMG temp;
			//cv::Mat aa;
			//temp.image = img.clone();
			temp.img_time = data->_timeStamp;
			//{
			//	std::lock_guard<std::mutex> l(ti);
			//	imgTime = temp.img_time;
			//}
			{
				std::lock_guard<std::mutex> lck(im);
				Img_Queue.push(temp);
			}
			//img.release();
		}
	}
}

void  SdkCameraCallBack(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam)
{
	if (g_save)
	{
	}
}
double last_imutime = 0.0;
void sdkImuCallBack(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam)
{
	if (g_save)
	{
		LOG(INFO) << std::setprecision(10) << time;
	}
}

void sdkSLAMResult(int, void* pData, void* pParam)
{
	if (g_save)
	{
	}
}

#if 0
/*Test: IMU data callback*/
TEST(Indem, ModuleIMUTest) {
	indem::CIMRSDK* pSDK = CreateImrSDK(true);
	if (pSDK != NULL) {
		g_stop = false;
		std::thread savefie_thd(std::bind(SaveImu));
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
	std::cout << "driver" << std::endl;
	g_stop = false;
	g_save = true;
	IMRDEVICE_HANDLE handlfd = imrOpenIMR();
	DevicesInfo m_idiInfo = imrSearchDevices(handlfd);
	std::cout << "Find " << m_idiInfo.devNum << " Devices " << std::endl;

	CameraConfig cc;
	cc._cb = CameraCallbackFunction;
	imrInitCamera(handlfd, cc);
	imrStartCamera(handlfd);

	imrConnectDevice(handlfd, NULL, 0, IMUDataCallbackFunction);

#ifdef WIN32
	Sleep(10 * 60);
#else
	sleep(1 * 60 * 60);
#endif
	g_stop = true;

	imrCloseIMR(NULL);
	std::cout << "TestDriverFinish" << std::endl;
}
#endif

#if 0
TEST(Indem, Initailize) {
    using namespace g3;
    std::unique_ptr<LogWorker> m_pLogger = LogWorker::createLogWorker();
    m_pLogger->addDefaultLogger("SDK", ".");
    initializeLogging(m_pLogger.get());

    g_stop = false;
    using namespace indem;
    //RegistDisconnectCallback(HotPlugCallback, NULL);
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };
    //strcpy(config.slamPath, "G:/Software Develop kit/slam.dll");

    config.bSlam = true;
    EXPECT_TRUE(pSDK->Init(config));
	g_save = true;

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