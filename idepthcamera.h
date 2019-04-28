#ifndef IDEPTHCAMERA_H
#define IDEPTHCAMERA_H

#include "depthcamera_global.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DEPTHCAMERASHARED_EXPORT IDepthCamera
{
public:
    virtual int getWidth() = 0;
    virtual int getHeight() = 0;
    virtual int getFramerate() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentPointCloud() = 0;
    virtual ~IDepthCamera() = default;
};

#endif // IDEPTHCAMERA_H
