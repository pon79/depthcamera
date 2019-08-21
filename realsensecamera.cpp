#include "realsensecamera.h"

#include <QDebug>

#include "rs.hpp"

RealsenseCamera::RealsenseCamera()
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    // Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();

    // Get the depth frame's dimensions
    float width = depth.get_width();
    float height = depth.get_height();

    // Query the distance from the camera to the object in the center of the image
    float dist_to_center = depth.get_distance(width / 2, height / 2);

    // Print the distance
      qDebug().noquote() << "The camera is facing an object " << dist_to_center << " meters away \r";



//    rs2::config cfg;
//    cfg.enable_stream(RS2_STREAM_DEPTH, -1, cameraWidth, cameraHeight, RS2_FORMAT_ANY, cameraFramerate);
//    cfg.enable_stream(RS2_STREAM_COLOR, -1, cameraWidth, cameraHeight, RS2_FORMAT_RGB8, cameraFramerate);
//    rsPipeline.start(cfg);

//    auto cameraList = rsContext.query_devices();
//    if (cameraList.size() == 0)
//        throw std::runtime_error("Can't found realsense camera");
//    rsCamera = cameraList.front();

}
