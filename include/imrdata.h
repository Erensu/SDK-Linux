#pragma once
//该文件存放SDK对外的数据接口
//坐标系定义：x轴右，y轴上,z轴后
struct ImrPose {
	//[x,y,z]
	float _position[3];
	//[w,x,y,z]
	float _rotation[4];
};
//头显位姿
struct ImrModulePose {
	ImrPose _pose;
	bool _isLoop;   //闭环标志
    int  _score;    //-1和0是有效数据,其它无效数据
};

//头显摄像头图像数据
struct ImrImage {
    double _time;
    int _width;
    int _height;
	unsigned char* _image;
};

//头显摄像头双目原始图像数据
struct ImrImages {
    double _time;
    int _width;
    int _height;
    unsigned char* _image[2];
};

//模组信息
struct ImrModuleDeviceInfo {
    char _id[32];               //模组ID
    char _designer[32];         //模组开发商
    char _fireware_version[32]; //固件版本
    char _hardware_version[32];
    char _lens[32];
    char _imu[32];
    char _viewing_angle[32];
    char _baseline[32];
};

//相机标定参数
struct CameraCalibrationParameter {
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