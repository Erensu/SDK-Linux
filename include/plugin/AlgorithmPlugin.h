#pragma once

#ifdef WIN32
#define ALGORITHM_DLL_EXPORT extern "C" __declspec( dllexport )
#else
#define ALGORITHM_DLL_EXPORT
#endif

//接口版本号
#define INTERFACE_MAJOR_VERSION   1

#define _IN_    //传入参数
#define _OUT_   //传出参数

/*
* \brief 插件回调函数
* \param ret 回调传出的状态码
* \param pResult 插件传出的数据结构指针
* \param pParam 用户在添加回调函数时传入的参数
*/
typedef void ( *PluginCallback)(int ret, void* pResult, void* pParam);

namespace indem {
    struct CamaraParams {
        int _width;         //图像宽
        int _height;        //图像高
        int _channel;       //通道数

        double _Kl[9];      //3X3 左相机内参矩阵
        double _Kr[9];      //3X3 右相机内参矩阵
        double _Dl[4];      //4X1 左相机畸变差校正参数
        double _Dr[4];      //4X1 右相机畸变差校正参数
        double _Pl[12];     //3X4 基线校正后左相机投影矩阵
        double _Pr[12];     //3X4 基线校正后右相机投影矩阵
        double _Rl[9];      //3X3 基线校正后左相机旋转矩阵
        double _Rr[9];      //3X3 基线校正后左相机旋转矩阵
        double _TSCl[16];   //4X4 左相机系到传感器坐标系的变换
        double _TSCr[16];   //4X4 右相机系到传感器坐标系的变换

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

    struct PluginInfo {
        int major;              //插件主版本号
        int minor;              //插件次版本号
        char developer[64];    //插件开发团队名
    };

    struct Pose {
        float position[3];      //x,y,z
        float quart[4];         //w,x,y,z
    };

    class IAlgorithmPlugin {
    public:
        static int Version() { return INTERFACE_MAJOR_VERSION; }

        virtual ~IAlgorithmPlugin() {}

        /*
        * \brief 算法名
        */
        virtual const char* Name() = 0;
        /*
        * \brief 初始化,在创建后会被调用
        */
        virtual bool Init(CamaraParams pParams) = 0;
        /*
        * \brief SDK平台以1kHz的频率调用该接口,把融合后实际位姿传给算法
        */
        virtual void AddPoseAsync(double time, const Pose& pose) = 0;
        /*
        * \brief SDK平台以50Hz的频率调用该接口,把原始图像传给算法
        */
        virtual void AddImageAsync(double time, unsigned char* pLeft, unsigned char* pRight,int width,int height,int channel) = 0;
        /*
        * \brief 用户添加一个新的回调函数
        */
        virtual int AddCallback(const char* name, PluginCallback pCallback, void* pParam) = 0;
        /*
        * \brief 释放并删除算法自身。在SDK释放、获取插件信息结束的时候该接口都会调用
        */
        virtual void Release() = 0;
        /*
        * \brief 执行特定的操作
        */
        virtual bool InvokeCommand(const char* commandName, _IN_ void* pIn, _OUT_ void* pOut) { return true; }

        /*
        * \brief 获取插件信息
        */
        virtual PluginInfo GetPluginInfo()=0;
    };
}

#ifdef __cplusplus
extern "C" {
#endif
	/*
	* \brief 在SDK初始化、获取插件信息的时候该接口都会调用,因此该接口的实现应该尽可能的简单
	*/
	ALGORITHM_DLL_EXPORT indem::IAlgorithmPlugin* AlgorithmFactory();

#ifdef __cplusplus
}
#endif