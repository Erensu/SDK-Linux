#pragma once

#ifdef WIN32
#define SLAM_DLL_EXPORT extern "C" __declspec( dllexport )
#else
#define SLAM_DLL_EXPORT
#endif

#define SLAM_PLUGIN_VERSION     1

namespace indem {
    //����궨����
    struct CameraCalibrationParam {
        int _width;         //ͼ���
        int _height;        //ͼ���
        int _channel;       //ͨ����

        double _Kl[9];      //3X3 ������ڲξ���
        double _Kr[9];      //3X3 ������ڲξ���
        double _Dl[4];      //4X1 ����������У������,���ۻ���
        double _Dr[4];      //4X1 ����������У������,���ۻ���
        double _Pl[12];     //3X4 ����У���������ͶӰ����
        double _Pr[12];     //3X4 ����У���������ͶӰ����
        double _Rl[9];      //3X3 ����У�����������ת����
        double _Rr[9];      //3X3 ����У�����������ת����
        double _TSCl[16];   //4X4 �����ϵ������������ϵ�ı任
        double _TSCr[16];   //4X4 �����ϵ������������ϵ�ı任
        double _baseline;   //���ߣ���λ��m

        double _AMax;
        double _GMax;
        double _SigmaGC;
        double _SigmaAC;
        double _SigmaBg;
        double _SigmaBa;
        double _SigmaGwC;
        double _SigmaAwC;

        /*  �ӼƲ���,3X4����,ÿ��Ԫ������
        *    x   y   z
        *    11  12  13
        *    21  22  23
        *    31  32  33
        */
        double _Acc[12];
        /*  ���ݲ���,3X4����,ÿ��Ԫ������
        *    x   y   z
        *    11  12  13
        *    21  22  23
        *    31  32  33
        */
        double _Gyr[12];
    };

    //�첽��ȡλ����Ϣʱ���ص�״̬
    enum SlamStatus {
        PS_VALID=0,         //��ʾ��������Ч��λ��
        PS_LOOP=1,          //��ʾ��ǰ�����˱ջ�
        PS_INVALID_ENV=2,   //��ʾ�㷨����Ӧ�Ļ���
        PS_RELOCATE_FAIL,   //��ʾ�ض�λʧ��
        PS_INIT_FAIL,       //��ʾ��ʼ��ʧ��
        PS_INVALID          //��ʾ��ǰ��һ����Ч��λ��
    };

    class ISlamPlugin {
    public:
        static int Version() { return SLAM_PLUGIN_VERSION; }
        virtual ~ISlamPlugin() {}

        /*
        * \brief ��ʼ��,�ڴ�����ᱻ����
        */
        virtual bool Init(CameraCalibrationParam pParams) = 0;
        /*
        * \brief �ͷŲ�ɾ��������SDK�ͷŸýӿڻ����
        */
        virtual void Release() = 0;

        virtual void AddIMUAsync(double time, float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ) = 0;
        virtual void AddIMGAsync(double time, unsigned char* pLeft,unsigned char* pRight,int width,int height,int channel) = 0;
        virtual SlamStatus GetPoseAsync(double* time, float* p, float* q) = 0;
        virtual bool InvokeCommand(const char* commandName, void* pIn, void* pOut)=0;
    };
}

SLAM_DLL_EXPORT indem::ISlamPlugin* SlamFactory();