#include "myntcamera.h"

MyntCamera::MyntCamera()
{
    mynteyeApi = mynteye::API::Create(0, nullptr);
    if(!mynteyeApi)
        throw std::runtime_error("Couldn't initialize MYNT EYE device");

    mynteyeDevice = mynteyeApi->device();
}

int MyntCamera::getWidth()
{
    return 0;
}

int MyntCamera::getHeight()
{
    return 0;
}

int MyntCamera::getFramerate()
{
    return mynteyeApi->GetOptionValue(mynteye::Option::FRAME_RATE);
}

void MyntCamera::start()
{
    mynteyeApi->EnableStreamData(mynteye::Stream::POINTS);
    mynteyeApi->Start(mynteye::Source::VIDEO_STREAMING);
}

void MyntCamera::stop()
{
    mynteyeApi->Stop(mynteye::Source::VIDEO_STREAMING);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MyntCamera::getCurrentPointCloud()
{
    cv::Mat pointsFromCamera;
    while(pointsFromCamera.empty())
    {
        mynteyeApi->WaitForStreams();
        pointsFromCamera = mynteyeApi->GetStreamData(mynteye::Stream::POINTS).frame;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentPointCloud = convertCvMarixToPclPointCloud(pointsFromCamera);
    return currentPointCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MyntCamera::convertCvMarixToPclPointCloud(const cv::Mat &cvMatrix)
{
    const int matrixRows = cvMatrix.rows;
    const int matrixCols = cvMatrix.cols;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>(static_cast<uint32_t>(matrixRows),
                                                                                               static_cast<uint32_t>(matrixCols)));

    for (int row = 0; row < matrixRows; row++)
    {
      for (int col = 0; col < matrixCols; col++)
      {
        pcl::PointXYZRGB pclPoint;

        auto &&cvPoint = cvMatrix.at<cv::Point3f>(row, col);

        if(std::isfinite(cvPoint.x) && std::isfinite(cvPoint.y) && std::isfinite(cvPoint.z))
        {
          pclPoint.x = cvPoint.x;
          pclPoint.y = cvPoint.y;
          pclPoint.z = cvPoint.z;
        }

        pclPointCloud->points.at(static_cast<size_t>(row*matrixCols + col)) = pclPoint;
      }
    }

    pclPointCloud->width = static_cast<uint32_t>(matrixCols);
    pclPointCloud->height = static_cast<uint32_t>(matrixRows);

    return pclPointCloud;
}

MyntCamera::~MyntCamera()
{
    stop();
}
