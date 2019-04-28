#ifndef MYNTCAMERA_H
#define MYNTCAMERA_H

#include "idepthcamera.h"
#include <mynteye/api/api.h>

class DEPTHCAMERASHARED_EXPORT MyntCamera : public IDepthCamera
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertCvMarixToPclPointCloud(const cv::Mat &cvMatrix);

public:
    MyntCamera();
    int getWidth() override;
    int getHeight() override;
    int getFramerate() override;
    void start() override;
    void stop() override;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentPointCloud() override;
    ~MyntCamera() override;

private:
    std::shared_ptr<mynteye::API> mynteyeApi;
    std::shared_ptr<mynteye::Device> mynteyeDevice;
};

#endif // MYNTCAMERA_H
