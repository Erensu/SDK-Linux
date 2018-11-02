#pragma once

#ifdef WIN32
#define ALGORITHM_DLL_EXPORT extern "C" __declspec( dllexport )
#else
#define ALGORITHM_DLL_EXPORT
#endif

//�ӿڰ汾��
#define INTERFACE_MAJOR_VERSION   1

#define _IN_    //�������
#define _OUT_   //��������

/*
* \brief ����ص�����
* \param ret �ص�������״̬��
* \param pResult ������������ݽṹָ��
* \param pParam �û�����ӻص�����ʱ����Ĳ���
*/
typedef void ( *PluginCallback)(int ret, void* pResult, void* pParam);

namespace indem {
    struct CamaraParams {
        int _width;         //ͼ���
        int _height;        //ͼ���
        int _channel;       //ͨ����

        double _Kl[9];      //3X3 ������ڲξ���
        double _Kr[9];      //3X3 ������ڲξ���
        double _Dl[4];      //4X1 ����������У������
        double _Dr[4];      //4X1 ����������У������
        double _Pl[12];     //3X4 ����У���������ͶӰ����
        double _Pr[12];     //3X4 ����У���������ͶӰ����
        double _Rl[9];      //3X3 ����У�����������ת����
        double _Rr[9];      //3X3 ����У�����������ת����
        double _TSCl[16];   //4X4 �����ϵ������������ϵ�ı任
        double _TSCr[16];   //4X4 �����ϵ������������ϵ�ı任

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

    struct PluginInfo {
        int major;              //������汾��
        int minor;              //����ΰ汾��
        char developer[64];    //��������Ŷ���
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
        * \brief �㷨��
        */
        virtual const char* Name() = 0;
        /*
        * \brief ��ʼ��,�ڴ�����ᱻ����
        */
        virtual bool Init(CamaraParams pParams) = 0;
        /*
        * \brief SDKƽ̨��1kHz��Ƶ�ʵ��øýӿ�,���ںϺ�ʵ��λ�˴����㷨
        */
        virtual void AddPoseAsync(double time, const Pose& pose) = 0;
        /*
        * \brief SDKƽ̨��50Hz��Ƶ�ʵ��øýӿ�,��ԭʼͼ�񴫸��㷨
        */
        virtual void AddImageAsync(double time, unsigned char* pLeft, unsigned char* pRight,int width,int height,int channel) = 0;
        /*
        * \brief �û����һ���µĻص�����
        */
        virtual int AddCallback(const char* name, PluginCallback pCallback, void* pParam) = 0;
        /*
        * \brief �ͷŲ�ɾ���㷨������SDK�ͷš���ȡ�����Ϣ������ʱ��ýӿڶ������
        */
        virtual void Release() = 0;
        /*
        * \brief ִ���ض��Ĳ���
        */
        virtual bool InvokeCommand(const char* commandName, _IN_ void* pIn, _OUT_ void* pOut) { return true; }

        /*
        * \brief ��ȡ�����Ϣ
        */
        virtual PluginInfo GetPluginInfo()=0;
    };
}

#ifdef __cplusplus
extern "C" {
#endif
	/*
	* \brief ��SDK��ʼ������ȡ�����Ϣ��ʱ��ýӿڶ������,��˸ýӿڵ�ʵ��Ӧ�þ����ܵļ�
	*/
	ALGORITHM_DLL_EXPORT indem::IAlgorithmPlugin* AlgorithmFactory();

#ifdef __cplusplus
}
#endif