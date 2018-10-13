#include <iostream>
#include <gtest/gtest.h>
#include "imrsdk.h"
#include <thread>
#ifdef WIN32
#pragma comment(lib,"indem.lib")
#endif

int g_iImageWidth = 0, g_iImageHeight = 0;

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
void GetCameraCallback(double, ImrImage pData, void*) {
    EXPECT_TRUE(pData._image != NULL);
    //cv::Mat img(pData._height, pData._width, CV_8UC1, pData._image);
    //imwrite("D:/indemTest.jpg", img);
}

void HMDPoseCallback(int, void* pData, void* pParam) {
    ImrHMDPose* pHeadPose = (ImrHMDPose*)pData;
    //检查两次之间的间隔距离平方应当小于5cm
    static double sdX = 0, sdY = 0, sdZ = 0;
    double dx = (pHeadPose->_pose._position[0] - sdX);
    double dy = (pHeadPose->_pose._position[1] - sdY);
    double dz = (pHeadPose->_pose._position[2] - sdZ);
    double ds2 = dx * dx + dy * dy + dz * dz;
    sdX = pHeadPose->_pose._position[0];
    sdY = pHeadPose->_pose._position[1];
    sdZ = pHeadPose->_pose._position[2];
    std::cout << "D: " << ds2 << std::endl;
    //EXPECT_TRUE(ds2 < 0.005);
}

struct ImrSize {
    int32_t _width;
    int32_t _height;
};

TEST(Indem, Initailize) {
    using namespace indem;
    //RegistDisconnectCallback(HotPlugCallback, NULL);
    CIMRSDK* pSDK = new CIMRSDK();
    MRCONFIG config = { 0 };
    //设置slam频率为60ms
    config.imgFreq = 60;
#ifdef CMAKE_INTDIR
    if (strcmp(CMAKE_INTDIR, "RelWithDebInfo") == 0) {
        strcpy(config.loadPath, "D:/DL");
    }
    else {//200服务器环境
#else
    {
#endif
#ifdef WIN32
        strcpy(config.loadPath, "D:/indemind/test/motionless");
#else
        strcpy(config.loadPath, "/opt/test/motionless");
#endif
    }
    //注册头显位姿回调函数
    pSDK->RegistHMDPoseCallback(HMDPoseCallback, NULL);
    EXPECT_TRUE(pSDK->Init(config));
    int plgNum = 0;
    char** names=new char*[8];
    for (int i=0;i<8;++i)
    {
        names[i] = new char[32];
    }
    pSDK->ListPluginsInfo(&plgNum, names);
    for (int i = 0; i < 8; ++i)
    {
        delete[] names[i];
    }
    delete[] names;
    //注册摄像头画面回调函数
    pSDK->AddPluginCallback("seethrough", "image", SeeThrougthCallback, NULL);
    std::cout << "Start Motionless Test" << std::endl;
    //获取图像大小
    ImrSize iSize = { 0 };
    if (pSDK->InvokePluginMethod("seethrough", "size", NULL, &iSize) == 0) {
        ASSERT_TRUE(iSize._width != 0);
        ASSERT_TRUE(iSize._height != 0);
        g_iImageWidth = iSize._width;
        g_iImageHeight = iSize._height;
    }
    std::this_thread::sleep_for(std::chrono::seconds(15));
    //关闭摄像头画面回调函数
    pSDK->AddPluginCallback("seethrough", "image", NULL, NULL);
    std::this_thread::sleep_for(std::chrono::seconds(60));
    pSDK->Release();
    delete pSDK;
}