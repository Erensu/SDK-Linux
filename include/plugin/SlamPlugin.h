#pragma once

#ifdef WIN32
#define SLAM_DLL_EXPORT extern "C" __declspec( dllexport )
#else
#define SLAM_DLL_EXPORT
#endif

#define SLAM_PLUGIN_VERSION     1

namespace indem {
    //相机标定参数
    struct CameraCalibrationParam {
        int _width;         //图像宽
        int _height;        //图像高
        int _channel;       //通道数

        double _Kl[9];      //3X3 左相机内参矩阵
        double _Kr[9];      //3X3 右相机内参矩阵
        double _Dl[4];      //4X1 左相机畸变差校正参数,鱼眼畸变
        double _Dr[4];      //4X1 右相机畸变差校正参数,鱼眼畸变
        double _Pl[12];     //3X4 基线校正后左相机投影矩阵
        double _Pr[12];     //3X4 基线校正后右相机投影矩阵
        double _Rl[9];      //3X3 基线校正后左相机旋转矩阵
        double _Rr[9];      //3X3 基线校正后左相机旋转矩阵
        double _TSCl[16];   //4X4 左相机系到传感器坐标系的变换
        double _TSCr[16];   //4X4 右相机系到传感器坐标系的变换
        double _baseline;   //基线，单位：m

        double _AMax;
        double _GMax;
        double _SigmaGC;
        double _SigmaAC;
        double _SigmaBg;
        double _SigmaBa;
        double _SigmaGwC;
        double _SigmaAwC;

        /*  加计参数,3X4矩阵,每个元素如下
        *    x   y   z
        *    11  12  13
        *    21  22  23
        *    31  32  33
        */
        double _Acc[12];
        /*  陀螺参数,3X4矩阵,每个元素如下
        *    x   y   z
        *    11  12  13
        *    21  22  23
        *    31  32  33
        */
        double _Gyr[12];
    };

    //异步获取位置信息时返回的状态
    enum SlamStatus {
        PS_VALID=0,         //表示返回了有效的位置
        PS_LOOP=1,          //表示当前发生了闭环
        PS_INVALID_ENV=2,   //表示算法不适应的环境
        PS_RELOCATE_FAIL,   //表示重定位失败
        PS_INIT_FAIL,       //表示初始化失败
        PS_INVALID          //表示当前是一个无效的位置
    };

    class ISlamPlugin {
    public:
        static int Version() { return SLAM_PLUGIN_VERSION; }
        virtual ~ISlamPlugin() {}

        /*
        * \brief 初始化,在创建后会被调用
        */
        virtual bool Init(CameraCalibrationParam pParams) = 0;
        /*
        * \brief 释放并删除自身。在SDK释放该接口会调用
        */
        virtual void Release() = 0;

        virtual void AddIMUAsync(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) = 0;
        virtual void AddIMGAsync(double time, unsigned char* pLeft,unsigned char* pRight,int width,int height,int channel) = 0;
        virtual SlamStatus GetPoseAsync(double* time, float* p, float* q) = 0;
        virtual bool InvokeCommand(const char* commandName, void* pIn, void* pOut)=0;
    };
}

SLAM_DLL_EXPORT indem::ISlamPlugin* SlamFactory();