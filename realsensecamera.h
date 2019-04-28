#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <librealsense2/rs.hpp>

#include <QDebug>

#include "idepthcamera.h"

class DEPTHCAMERASHARED_EXPORT RealsenseCamera : public IDepthCamera
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRsPointsToPclPointCloud(const rs2::points& points, const rs2::video_frame& color);
    std::tuple<int, int, int> extractRGBFromRsTexture(const rs2::video_frame &texture, const rs2::texture_coordinate &textureCoords);

public:
    RealsenseCamera();
    int getWidth() override;
    int getHeight() override;
    int getFramerate() override;
    void start() override;
    void stop() override;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCurrentPointCloud() override;
    ~RealsenseCamera() override;

private:
    const int cameraFramerate = 60;
    const int cameraWidth = 848;
    const int cameraHeight = 480;
    rs2::device rsCamera;
    rs2::context rsContext;
    rs2::pipeline rsPipeline;
    bool isStarted = false;
};

#endif // REALSENSECAMERA_H
