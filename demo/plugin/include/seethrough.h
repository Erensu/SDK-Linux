#include "AlgorithmPlugin.h"
#include <unordered_map>
#include <opencv2/core.hpp>
#include "QueueWorker.h"

/************************************************************************/
/* see throughģ��
* ������ͼ��ص������ӿڶ������£�
* �ص�������: "image"
* �������������� ImrImage
* SDK����ʾ����
*       void ImageDataCallback(int, void* pData, void* pParam) {
*           ImrImage* pImage = reinterpret_cast<ImrImage*>(pData);
*       }
*       m_pSDK->AddPluginCallback("seethrough", "image", ImageDataCallback, this);
*
* ��ȡͼ���Сָ�
* ָ������"size"
* ����������pOut 
*     ���� ImageSize
* SDK����ʾ����
*       ImageSize iSize;
*       pSDK->InvokePluginMethod("seethrough", "size", NULL, &iSize);
*/
/************************************************************************/

namespace indem {
    struct ImageSize {
        int32_t _width;
        int32_t _height;
    };

    struct ImrImage {
        double _time;
        int _width;
        int _height;
        unsigned char* _image;
    };

    struct SeeThroughImage {
        double _time;
        int32_t _width;
        int32_t _height;
        cv::Mat _image;
    };
    class CSeeThrough : public IAlgorithmPlugin {
    public:
        CSeeThrough();
        ~CSeeThrough();

        virtual const char* Name() { return "seethrough"; }
        virtual bool Init(CamaraParams pParams);
        virtual void AddPoseAsync(double time, const Pose& pose);
        virtual void AddImageAsync(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel);
        virtual int AddCallback(const char* name, PluginCallback pCallback, void* pParam);
        virtual void Release();

        bool InvokeCommand(const char* commandName, _IN_ void* pIn, _OUT_ void* pOut);
        PluginInfo GetPluginInfo();

    private:
        void RectImageTask(const SeeThroughImage& img);

    private:
        cv::Mat m_mHeadKl, m_mHeadKr;
        cv::Mat m_mHeadDl, m_mHeadDr;
        cv::Mat m_mHeadRl, m_mHeadRr;
        cv::Mat m_mHeadPl, m_mHeadPr;
        cv::Mat X;
        cv::Mat Y;

        typedef std::unordered_map<std::string, std::pair<PluginCallback, void*> > CallbackMap;
        //�ص�����
        CallbackMap m_mCallbacks;

        QueueWorker<SeeThroughImage> m_qWorker;
        ImageSize m_iSize;          //���ͼ���С
    };
}