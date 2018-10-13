#include "seethrough.h"
#include <windows.h>
#include <opencv2/calib3d.hpp>
#include <experimental/filesystem>
#include <opencv2/imgproc.hpp>

ALGORITHM_DLL_EXPORT indem::IAlgorithmPlugin* AlgorithmFactory() {
    indem::IAlgorithmPlugin* pAlg = new indem::CSeeThrough();
    return pAlg;
}

namespace indem {
    CSeeThrough::CSeeThrough()
    {

    }

    CSeeThrough::~CSeeThrough()
    {

    }
#define MAX_PATH 256
    bool CSeeThrough::Init(CamaraParams pParams)
    {
        m_mHeadKl = cv::Mat(3, 3, CV_64FC1, pParams._Kl);
        m_mHeadKr = cv::Mat(3, 3, CV_64FC1, pParams._Kr);
        m_mHeadDl = cv::Mat(4, 1, CV_64FC1, pParams._Dl);
        m_mHeadDr = cv::Mat(4, 1, CV_64FC1, pParams._Dr);
        m_mHeadRl = cv::Mat(3, 3, CV_64FC1, pParams._Rl);
        m_mHeadRr = cv::Mat(3, 3, CV_64FC1, pParams._Rr);
        m_mHeadPl = cv::Mat(3, 4, CV_64FC1, pParams._Pl);
        m_mHeadPr = cv::Mat(3, 4, CV_64FC1, pParams._Pr);
        //读取配置信息,确定去畸变参数
        cv::Size size(pParams._width, pParams._height);
        int width = size.width;
        int height = size.height;
        std::vector<cv::Point2f> vecCxyBoundaryPoints;
        std::vector<cv::Point2f> vecUndistCxyBoundaryPoints;
        double cx0 = m_mHeadKl.at<double>(0, 2);
        double cy0 = m_mHeadKl.at<double>(1, 2);
        vecCxyBoundaryPoints.push_back(cv::Point2f(cx0, 0));
        vecCxyBoundaryPoints.push_back(cv::Point2f(cx0, height));
        vecCxyBoundaryPoints.push_back(cv::Point2f(0, cy0));
        vecCxyBoundaryPoints.push_back(cv::Point2f(width, cy0));
        cv::fisheye::undistortPoints(vecCxyBoundaryPoints, vecUndistCxyBoundaryPoints,
            m_mHeadKl, m_mHeadDl, m_mHeadRl, m_mHeadPl);

        int minx = INT_MAX;
        int miny = INT_MAX;
        int maxx = INT_MIN;
        int maxy = INT_MIN;

        for (auto& it : vecUndistCxyBoundaryPoints)
        {
            minx = min(minx, ((int)floor(it.x)));
            miny = min(miny, ((int)floor(it.y)));
            maxx = max(maxx, ((int)ceil(it.x)));
            maxy = max(maxy, ((int)ceil(it.y)));
        }
        cv::Mat Pl = m_mHeadPl.clone();
        Pl.at<double>(0, 2) -= minx;
        Pl.at<double>(1, 2) -= miny;
        size.width = maxx - minx;
        size.height = maxy - miny;
        m_iSize._width = size.width;
        m_iSize._height = size.height;
        cv::Mat remapX(size, CV_32FC1);
        cv::Mat remapY(size, CV_32FC1);

        cv::fisheye::initUndistortRectifyMap(
            m_mHeadKl, m_mHeadDl, m_mHeadRl, Pl,
            size, CV_32FC1, remapX, remapY);

        remapX.copyTo(X);
        remapY.copyTo(Y);
        m_qWorker.SetWorker(std::bind(&CSeeThrough::RectImageTask, this, std::placeholders::_1));
        return true;
    }

    void CSeeThrough::AddPoseAsync(double time, const Pose& pose)
    {

    }

    void CSeeThrough::AddImageAsync(double time, unsigned char* pLeft, unsigned char* pRight, int width, int height, int channel)
    {
        if (m_mCallbacks.size()) {
            auto itr=m_mCallbacks.find("image");
            if (itr != m_mCallbacks.end()) {
                SeeThroughImage image;
                image._time = time;
                image._height = height;
                image._width = width;
                image._image = cv::Mat(height, width, CV_8UC1, pLeft);
                m_qWorker.PushData(image);
            }
        }
    }

    int CSeeThrough::AddCallback(const char* name, PluginCallback pCallback, void* pParam)
    {
        if (pCallback == NULL) {
            m_mCallbacks.erase(name);
            if (m_mCallbacks.size() == 0) {
                //关闭算法功能
            }
            return 0;
        }
        m_mCallbacks[name] = { pCallback,pParam };
        if (m_mCallbacks.size() == 1) {
            //启用算法
        }
        return 0;
    }

    void CSeeThrough::Release()
    {
        delete this;
    }

    bool CSeeThrough::InvokeCommand(const char* commandName, _IN_ void* pIn, _OUT_ void* pOut)
    {
        if (strcmp(commandName, "size") == 0) {
            ImageSize* pSize = reinterpret_cast<ImageSize*>(pOut);
            pSize->_height = m_iSize._height;
            pSize->_width = m_iSize._width;
            return true;
        }
        return false;
    }

    indem::PluginInfo CSeeThrough::GetPluginInfo()
    {
        indem::PluginInfo info = { 0 };
        strcpy(info.developer, "webberg");
        info.major = 1;
        info.minor = 0;
        return std::move(info);
    }

    void CSeeThrough::RectImageTask(const SeeThroughImage& img)
    {
        cv::Mat mImg;
        cv::remap(img._image, mImg, X, Y, cv::INTER_LINEAR);
        ImrImage image;
        image._time = img._time;
        image._height = mImg.rows;
        image._width = mImg.cols;
        image._image = mImg.data;
        auto& pCallback = m_mCallbacks["image"];
        ((PluginCallback)pCallback.first)(0, reinterpret_cast<void*>(&image), pCallback.second);
    }

}

