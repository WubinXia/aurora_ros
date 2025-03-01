#pragma once

#include <vector>
#include <rpos/core/pose.h>
#include <cstdint>

namespace rpos { namespace message { namespace depth_camera {

    /**
    * @brief depth camera intrinsics, fx, fy, cx, cy
    */
    struct Intrinsics
    {
        float fx;
        float fy;
        float cx;
        float cy;
        void operator *=(const float scale)
        {
            fx *= scale;
            fy *= scale;
            cx *= scale;
            cy *= scale;
        }
    };
    
    /**
    * @brief Depth camera frame
    */
    struct DepthCameraFrame
    {
        /**
        * @brief Min distance of the sensor (in meters)
        */
        float minValidDistance;

        /**
        * @brief Max distance of the sensor (in meters)
        */
        float maxValidDistance;

        /**
        * @brief The pitch of the first line of the frame (in rad)
        */
        float minFovPitch;

        /**
        * @brief The pitch of the last line of the frame (in rad)
        */
        float maxFovPitch;

        /**
        * @brief The yaw of the last column of the frame (in rad)
        */
        float minFovYaw;

        /**
        * @brief The yaw of the first column of the frame (in rad)
        */
        float maxFovYaw;

        /**
        * @brief how many columns in this frame
        */
        int cols;

        /**
        * @brief how many rows in this frame
        */
        int rows;

        /**
        * @brief depth camera intrinsics, fx, fy, cx, cy
        */
        Intrinsics intrinsics;

        // TODO:
        // We now use std::vector to store data, which will make *copying* frames very expensive
        // we should use a data structure support CoW (copy on write) or just use shared pointer to store the internal data
        /**
        * @brief The data (order: row by row, cell by cell in each row)
        */
        std::vector<float> data;
    };

    struct LegacyFlattenDepthCameraScanPoint//to be comparable with old sdk or old onlineslam
    {
        float dist;     // in meter
        float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
    };

    struct FlattenDepthCameraScanPoint
    {
        float dist;     // in meter
        float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
        float height;
        float safeDistance;

        FlattenDepthCameraScanPoint() :
              dist(0)
            , angle(0)
            , height(0)
            , safeDistance(0) {}
    };

    struct DepthCameraTransformParameters
    {
        bool enableUseHeightFilter;
        bool enableUseMorphologyFilter;
        rpos::core::Pose depthCamPose;
        float minHeightFromGround;
        float maxHeightFromGround;
        bool isInversion;

        DepthCameraTransformParameters(bool enableHeightFilter, bool enableMorphologyFilter, rpos::core::Pose pose, bool inversion, float minHeight, float maxHeight)
            : enableUseHeightFilter(enableHeightFilter), enableUseMorphologyFilter(enableMorphologyFilter), depthCamPose(pose), minHeightFromGround(minHeight), maxHeightFromGround(maxHeight), isInversion(inversion)
        {}
    };

    typedef std::vector<LegacyFlattenDepthCameraScanPoint> LegacyFlattenDepthCameraScan;
    typedef std::vector<FlattenDepthCameraScanPoint> FlattenDepthCameraScan;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    struct DepthCameraInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DepthCameraInfo()
            : extrinsics(Eigen::Matrix4d::Identity())
            , depthScale(0.001f)
            , groundFilterHeightInMeter(0.15f)
        {};
        std::string manufacturer;
        std::string model;
        std::string serialNumber;
        Intrinsics intrinsics;
        Eigen::Matrix4d extrinsics;
        int depthFrameWidth;
        int depthFrameHeight;
        int depthFps;
        float depthScale;
        float groundFilterHeightInMeter;
        //metadata
    };

    struct DepthCamImageFeature
    {
        std::string cameraDesp;
        std::vector<uint8_t> depthData;
    };

    struct RGBCameraImageFeature
    {
        std::string cameraDesp;
        std::vector<uint8_t> rgbData;
    };

    typedef std::map<std::string, DepthCameraInfo> DepthCamInfoMap;
    typedef std::map<std::string, Eigen::Matrix4d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix4d> > > DepthCamExtrinsicMap;
}}}