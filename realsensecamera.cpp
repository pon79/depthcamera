#include "realsensecamera.h"

RealsenseCamera::RealsenseCamera() : rsPipeline(rsContext)
{
    auto cameraList = rsContext.query_devices();
    if (cameraList.size() == 0)
        throw std::runtime_error("Can't found realsense camera");
    rsCamera = cameraList.front();
}

int RealsenseCamera::getWidth()
{
    return cameraWidth;
}

int RealsenseCamera::getHeight()
{
    return cameraHeight;
}

int RealsenseCamera::getFramerate()
{
    return cameraFramerate;
}

void RealsenseCamera::start()
{
    if(isStarted)
    {
        qWarning() << QStringLiteral("Attempt to start already started camera");
        return;
    }

    isStarted = true;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, cameraWidth, cameraHeight, RS2_FORMAT_ANY, cameraFramerate);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, cameraWidth, cameraHeight, RS2_FORMAT_RGB8, cameraFramerate);
    rsPipeline.start(cfg);
}

void RealsenseCamera::stop()
{
    if(!isStarted)
    {
        qWarning() << QStringLiteral("Attempt to stop not started camera");
        return;
    }

    isStarted = false;
    rsPipeline.stop();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealsenseCamera::getCurrentPointCloud()
{
    auto&& frames = rsPipeline.wait_for_frames();
    auto&& depthFrame = frames.get_depth_frame();
    auto&& colorFrame = frames.get_color_frame();

    rs2::pointcloud rsPointCloud;
    rsPointCloud.map_to(colorFrame);
    return convertRsPointsToPclPointCloud(rsPointCloud.calculate(depthFrame), colorFrame);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealsenseCamera::convertRsPointsToPclPointCloud(const rs2::points& points,
                                                                                       const rs2::video_frame& colorFrame)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    auto streamProfile = points.get_profile().as<rs2::video_stream_profile>();
    pclPointCloud->width = static_cast<uint32_t>(streamProfile.width());
    pclPointCloud->height = static_cast<uint32_t>(streamProfile.height());
    pclPointCloud->is_dense = false;
    pclPointCloud->points.resize(points.size());

    auto textureCoordinates = points.get_texture_coordinates();
    auto vertex = points.get_vertices();
    std::tuple<uint8_t, uint8_t, uint8_t> rgbColor;

    for (size_t i = 0; i < points.size(); i++)
    {
        pclPointCloud->points[i].x = vertex[i].x;
        pclPointCloud->points[i].y = vertex[i].y;
        pclPointCloud->points[i].z = vertex[i].z;

        rgbColor = extractRGBFromRsTexture(colorFrame, textureCoordinates[i]);
        pclPointCloud->points[i].r = std::get<2>(rgbColor);
        pclPointCloud->points[i].g = std::get<1>(rgbColor);
        pclPointCloud->points[i].b = std::get<0>(rgbColor);
    }

   return pclPointCloud;
}

std::tuple<int, int, int> RealsenseCamera::extractRGBFromRsTexture(const rs2::video_frame &texture,
                                                                   const rs2::texture_coordinate &textureCoords)
{
    int width  = texture.get_width();
    int height = texture.get_height();

    // Normals to Texture Coordinates conversion
    int xValue = std::min(std::max(int(textureCoords.u * width  + .5f), 0), width - 1);
    int yValue = std::min(std::max(int(textureCoords.v * height + .5f), 0), height - 1);

    int bytes = xValue * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = yValue * texture.get_stride_in_bytes(); // Get line width in bytes
    int textIndex = bytes + strides;

    const auto newTexture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = newTexture[textIndex];
    int NT2 = newTexture[textIndex + 1];
    int NT3 = newTexture[textIndex + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

RealsenseCamera::~RealsenseCamera()
{
    stop();
}
