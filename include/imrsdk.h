#ifndef _IMRSDK_H_
#define _IMRSDK_H_
#include "imrdata.h"
#ifdef WIN32
#ifdef  indem_EXPORTS
#define INDEM_DLL_EXPORT __declspec(dllexport)
#else
#define INDEM_DLL_EXPORT __declspec(dllimport)
#endif
#define CALL_METHOD __cdecl
#else
#define CALL_METHOD
#define INDEM_DLL_EXPORT
#endif

#define MRSDK_VERSION       1           //SDK版本

//预定义的插件返回码
#define PLG_NOT_EXIST       1           //插件不存在

struct MrDetectionData;
/*
* \brief 数据回调函数
* \param iType 返回的信息码
* \param pData 设备数据，例如ImrHMDPose,ImrTrig
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *DataCallback)(int, void* pData, void* pParam);
/*
* \brief 设备IMU数据回调函数
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *ModuleIMUCallback)(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, void* pParam);
/*
* \brief 设备IMU数据回调函数
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *ModuleImageCallback)(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel, void* pParam);

typedef void(CALL_METHOD *ImgCallback)(double time, ImrImage pData, void* pParam);
/*
 * \brief 设备断开连接回调函数
 * \param iType 返回的信息码,设备接入为1,断开为0
 * \param pParam 用户定义的参数
 */
typedef void(CALL_METHOD *HotplugCallback)(int iType, void* pParam);
/*
* \brief 物体识别回调函数,回调时机和方式待定[例如识别内容发生变化开始回调,返回变化的物体;或者变化时返回所有当前物体等方式]
* \param res 识别的信息结果
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *RecognizeCallback)(ImrRecognizeResult* res, void* pParam);

/*
* \brief 障碍物提醒回调函数
* \param distance 障碍物距离
* \param x,y,z 障碍物坐标
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *BarrierCallback)(ImrBarrierDetectionTarget& res, void* pParam);

/*
* \brief 障碍物更新回调函数
* \param distance 障碍物距离
* \param x,y,z 障碍物坐标
* \param pParam 用户定义的参数
*///ImrDenseMapUpdateTarget
typedef void(CALL_METHOD *BarrierUpdateCallback)(ImrDenseMapUpdateTarget& res, void* pParam);

/*
* \brief 平面检测回调函数
* \param res 识别的信息结果
* \param pParam 用户定义的参数
*/
typedef void(CALL_METHOD *PlaindetectCallback)(ImrRecogPlaneTarget* res, void* pParam);

/*
* \brief 插件回调函数
* \param ret 回调传出的状态码
* \param pResult 插件传出的数据结构指针
* \param pParam 用户在添加回调函数时传入的参数
*/
typedef void (CALL_METHOD *PluginCallback)(int ret, void* pResult, void* pParam);

namespace indem {

    //SDK的初始化配置
    struct MRCONFIG {
        char capturePath[128];      //设置捕获路径,空表示不捕获数据
        char loadPath[128];         //设置数据载入路径
        bool bSlam;                 //是否使用slam，使用slam=true
        char slamPath[128];         //设置用户自定义的slam动态库路径,空表示使用默认slam
        unsigned int imgFreq;       //设置图像发送频率
    };

    //可以检测的设备能力
    enum MRCapbility {
        GPU_NVidia,     //检测是否支持英伟达GPU
        HAND_SUPPORT,   //是否支持手柄

    };

    //手柄LED灯的闪烁模式
    enum MRHandPattern {
        PATTERN_1,
        PATTERN_2,
        PATTERN_3,
        PATTERN_4,
    };

    //手柄所能支持的模式
    enum MRHandMode {
        MODE_NONE,      //无手柄模式,暂未开放
        MODE_SINGLE,    //单人模式,默认
        MODE_MULTI,     //多人模式,暂未开放
    };

    /*
    * \brief 注册设备热插拔回调函数,在构造函数之后即可调用,不需要在init函数之后调用
    * \param cb 回调函数
    * \param param 传给回调函数使用的参数
    */
    INDEM_DLL_EXPORT void RegistDisconnectCallback(HotplugCallback cb, void* param);

    class CIMRSDKImpl;
    class INDEM_DLL_EXPORT CIMRSDK {
    public:
        CIMRSDK();
        ~CIMRSDK();

        /*
        * \brief 初始化配置，并启动数据接收线程及算法线程
        * \param capturePath 设置捕获文件路径,空则不捕获
        */
        bool Init(MRCONFIG config = { 0 });
        void Release();

        /*
        * \brief 检测硬件是否支持指定的能力
        * \param cap 枚举值,传递需要检测的设备能力
        * \return 0-不支持,1-支持，2-硬件支持但与SDK版本不匹配
        */
        int GetCapbility(MRCapbility cap);

        /*
        * \brief 查询模组信息
        */
        ImrModuleDeviceInfo GetModuleInfo();

        //获取所有稠密地图
        ImrBarrierDetectionMapTarget GetAllDenseMap();

        //获取稠密地图更新  1为更新 0为删除
        ImrDenseMapUpdateTarget GetDenseMapUpdate();

        //障碍物检测 开启或关闭八叉树地图
        void Open_Or_Close_OctoMap(bool useoctomap);

        //障碍物检测当前位姿回调
        void RegistSlamPoseToBarrierDetect(DataCallback cb, void* param);

        /*
        * \brief 添加头显位姿回调函数
        * \param cb 对应算法处理完毕后，用于数据传出的回调函数
        * \param param 传给回调函数使用的参数
        */
        void RegistModulePoseCallback(DataCallback cb, void* param);
        /*
        * \brief 添加手柄位姿回调函数
        * \param cb 对应算法处理完毕后，用于数据传出的回调函数
        * \param param 传给回调函数使用的参数
        */
        void RegistHandPoseCallback(DataCallback cb, void* param);

        /*
        * \brief 注册相机原始图像回调函数
        * \param cb 数据传出的回调函数
        * \param param 传给回调函数使用的参数
        */
        void RegistModuleCameraCallback(ModuleImageCallback cb, void* param);
        /*
        * \brief 注册头显原始IMU回调函数
        * \param cb 数据传出的回调函数
        * \param param 传给回调函数使用的参数
        */
        void RegistModuleIMUCallback(ModuleIMUCallback cb, void* param);

        /*
        * \brief 注册按键回调函数
        * \param cb 对应算法处理完毕后，用于数据传出的回调函数
        * \param param 传给回调函数使用的参数
        */
        void RegistKeyCallback(DataCallback cb, void* param);
        /*
        * \brief 设置当前位置为居中原点
        */
        bool ResetCenter();

        /*
        * \brief 获得当前所有设备的信息,如果没有手柄配对,则返回的手柄信息为0
        */
        ImrDeviceInfo GetDevicesInfo();
        /*
        * \brief 查找手柄,如果millionSec后仍然没有找到,返回0结果
        * \param millionSec 最长等待毫秒数
        */
        ImrHandDevices SearchHandDevices(int millionSec);
        /*
        * \brief 配对手柄
        * \param handID 手柄ID
        */
        bool MatchHandDevice(const char* handID);
        /*
        * \brief 获得指定ID手柄的信息
        */
        ImrHandDevice GetHandDeviceInfo(const char* deviceID);

        /*
        * \brief 设置手柄的使用模式
        */
        void SetHandMode(MRHandMode mode);

        /*
        * \brief 设置手柄的闪烁模式
        */
        void SetHandPattern(MRHandPattern pattern);
        /*
        * \brief 设置指定手柄主从状态
        * \param master true-主,false-从
        */
        void SetHMDDeviceStatus(const char* deviceID, bool master);
        /*
        * \brief 获取指定手柄主从状态
        * \return 1-主,2-从,0-未设置
        */
        int GetHMDDeviceStatus(const char* deviceID);

        /*
        * \brief 获得指定ID设备的电量大小
        * \param deviceID 设备ID
        * \reutrn 电量大小：0-100
        */
        int GetPowerStatus(const char* deviceID);
        /*
        * \brief 导入地图
        * \param fullpath 导出文件的全路径
        * \reutrn 成功返回true
        */
        bool ImportMap(const char* fullpath);
        /*
        * \brief 导出地图
        * \param fullpath 导出文件的全路径
        * \reutrn 成功返回true
        */
        bool ExportMap(const char* fullpath);
        /*
        * \brief 导出稠密地图
        * \param fullpath 导出文件的全路径
        * \reutrn 成功返回true
        */
        bool ExportDenseMap(const char* fullpath);
        /*
        * \brief 导入稠密地图
        * \param fullpath 导出文件的全路径
        * \reutrn 成功返回true
        */
        bool ImportDenseMap(const char* fullpath);
        /*
        * \brief 添加虚拟物体,调用后会通过RecognizeCallback回调函数传递出来
        * \param name 虚拟物体的名字
        * \param position 虚拟物体的坐标x,y,z
        * \param angle 虚拟物体的水平角度,绕z轴
        * \param bbox 虚拟物体的大小
        * \param visible 虚拟物体是否可见
        * \reutrn 成功返回物体id,失败返回-1
        */
        const char* AddVirtualObject(const char* name, float* position, float angle, float* bbox, bool visible);
        /*
        * \brief 更新虚拟物体,调用后会通过RecognizeCallback回调函数传递出来
        * \param iID 虚拟物体的ID
        * \param position 虚拟物体的坐标x,y,z
        * \param angle 虚拟物体的水平角度,绕z轴
        * \param bbox 虚拟物体的大小
        * \param visible 虚拟物体是否可见
        */
        void UpdateVirtualObject(const char* iID, float* position, float angle, float* bbox, bool visible);

        /*
        * \brief 添加物体识别回调函数,如果不添加回调函数,则物体识别功能默认关闭
        * \param cb 对应算法处理完毕后，用于数据传出的回调函数
        * \param param 传给回调函数使用的参数
        * \return 1-成功
        */
        int AddRecognizeCallback(RecognizeCallback cb, void* param);
        /*
        * \brief 平面识别回调函数
        * \param cb 对应算法处理完毕后，用于数据传出的回调函数
        * \param param 传给回调函数使用的参数
        * \return true-成功
        */
        bool AddPlaindetectCallback(PlaindetectCallback cb, void* param);

        /**
        * @brief 为插件添加一个新的回调函数,如果两个名字相同,新的回调函数将替换掉旧的回调函数,如果插件回调函数数量为0,那么插件功能将关闭
        * @param pluginName     插件的名称
        * @param callbackName   给回调函数指定的名称
        * @param cb             回调函数指针
        * @param param          传给回调函数使用的用户数据指针
        * @return   状态码,成功为0
        */
        int AddPluginCallback(const char* pluginName, const char* callbackName, PluginCallback cb, void* param);

        /**
        * @brief 调用插件预定义的接口函数
        * @param pluginName     插件的名称
        * @param methodName     指定的函数名称
        * @param inParam        需要传给函数的参数
        * @param outParam       函数传出的结果
        * @return   状态码,成功为0;其他应该定义为该函数的返回值
        */
        int InvokePluginMethod(const char* pluginName, const char* methodName, void* inParam, void* outParam);

        /**
        * @brief 显示当前加载的所有插件
        * @param pluginNum      插件的数量
        * @param pluginName     每个插件的名称,需要传入足够大的空间来存储返回结果
        */
        static void ListPluginsInfo(int* pluginNum, char** pluginsName);

        /**
        * @brief 显示插件的信息
        * @param pluginName     要查看的插件的名称
        * @param major          插件主版本号
        * @param minor          插件次版本号
        * @param developer      插件开发团队名称
        */
        static void ListPluginInfo(const char* pluginsName, int* major, int* minor, char* developer);
    private:
        CIMRSDKImpl* m_pImpl;
    };

}
#endif