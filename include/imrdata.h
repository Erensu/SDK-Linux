#pragma once
//该文件存放SDK对外的数据接口
//坐标系定义：x轴右，y轴上,z轴后
struct ImrPose {
	double time;
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

struct ImrHandPose {
    char _direction;
    ImrPose _pose;
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

enum HAND_CONTROLLER_TYPE {
    LEFT_CONTROLLER = 0,
    RIGHT_CONTROLLER,
};
//按键数据 触发器 触摸板
struct ImrInput {
    HAND_CONTROLLER_TYPE _type;
	bool _bGrip;		//Grip 侧按键
	bool _bMenu;		//Menu 菜单键
	bool _bSystem;		//System 系统键
	bool _bTrigger;		//Tigger 扳机键
	bool _bUp;			//触摸板 按键 上
	bool _bDown;		//触摸板 按键 下
	bool _bLeft;		//触摸板 按键 左
	bool _bRight;		//触摸板 按键 右
	bool _bCenter;		//触摸板 按键 中间

	float _x_Trigger;	//Trigger 键 X轴数据
	float _y_Trigger;	//Trigger 键 Y轴数据  Y=0

	float _x_TouchPad;	//触摸板 x轴
	float _y_TouchPad;	//触摸板 y轴
};
//单个手柄状态信息
struct ImrHandDevice {
	char _ID[64];				//手柄ID
	bool _valid;			//是否可用
	char _direct;			//左手为'L',右手为'R'
	bool _serve;			//主从状态,主为true，从为false
	bool _matched;			//是否配对,已配对true,未配对false
};

//手柄状态信息
struct ImrHandDevices {
	short _handNum;			//当前手柄个数
	ImrHandDevice _hand[2];	//手柄信息
};
//模组信息
struct ImrModuleDeviceInfo {
    char _ID[64];				//头显ID
    char _designer[32];
    char _version[16];		    //固件版本
};


//设备信息
struct ImrDeviceInfo {
	ImrHandDevices _hands;	    //手柄信息
    ImrModuleDeviceInfo _head;	//头显信息
};

//物体识别功能的配置信息
struct ImrTargetDesc {
	int _targetNum;				//可识别的目标数量
	const char** _targetsName;	//可识别的目标名称
};

//被识别的物体信息
struct ImrTarget {
	char _name[64];			    //物体名
	char _id[64];			    //物体ID编号
    float _position[3];		    //物体坐标[x,y,z]
    float _rotation[4];         //姿态[w,x,y,z]
    float _box[3];				//物体大小[lengh,width,height]-[x,z,y]
    float _rect[4];             //物体在图像的位置和大小,x,y,dx,dy
	int _state;				    //0-初始状态,1-新物体,2-已删除,3-状态变更
    short _visible;             //物体可见性，0-可见，1-不可见
    bool _reality;              //物体是否是真实的,true-真实，false-虚拟的
};

//物体识别结果
struct ImrRecognizeResult {
	int _num;					//识别出的物体数量
	ImrTarget* _targets;		//识别出的物体信息
};

struct ImrPoint2 {
    float _x;
    float _y;
};
//平面检测结果
struct ImrPlaneTarget {
	int _id;                      //被识别平面ID
	float _surface[2];            //平面尺寸（长宽，只保留方形平面）
	float _position[3];           //平面中心点坐标（n系下）
	float _center_uv[2];          //像素坐标UV
	float _normal[3];             //平面垂向法线方向（n系下）

	float pixela[2];            //像素四个点坐标
	float pixelb[2];
	float pixelc[2];
	float pixeld[2];
	float worlda[3];            //世界系顶点坐标
	float worldb[3];            //世界系顶点坐标
	float worldc[3];            //世界系顶点坐标
	float worldd[3];            //世界系顶点坐标

    int _pointNum;              //点云数量
    ImrPoint2* _points;         //点云数据
};
//AR
struct ImrRecogPlaneTarget
{
	int _planecount;              //平面数量
    ImrPlaneTarget* _targets;
};
struct MrPoint {
	float _x;
	float _y;
	float _z;
	float _flag;
};
struct MrMapPoint {
	float _x;
	float _y;
	float _z;
	//float _chang;
	//float _kuan;
	//float _gao;
};
struct MrMapUpdatePoint {
	int update;
	float _x;
	float _y;
	float _z;
};
//检测深度数据(包括障碍物的点)
struct MrDetectionData {
	int _num;
	MrPoint* _points;
};

struct ImrBarrierDetectionTarget
{
    float _u;
    float _v;
    int _num;
    MrPoint* _targets;
};

struct ImrDenseMapUpdateTarget
{
    int _num;
    float _size;
    MrMapUpdatePoint* _targets;
};

struct ImrBarrierDetectionMapTarget {
    bool _initialiazed;
    int _num;
    MrMapPoint* _targets;
};