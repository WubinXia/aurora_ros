#pragma once
#include <rpos/core/pose.h>
#include <rpos/core/system_event.h>
#include <vector>
#include <string>
#include <cstdint>
namespace rpos { namespace core {

    struct DiagnosisInfoScanData
    {
        float dist; // in meter
        float angle; // in degree
        std::uint8_t quality;
    };

    struct DiagnosisInfoDepthCamScanData
    {
        float dist; // in meter
        float angle; // in degree
        float height;
        float safeDistance;
    };

    struct DiagnosisInfoLidarScan
    {
        std::vector<DiagnosisInfoScanData> lidarScan;
        Pose lidarPose;
    };

    struct DiagnosisInfoDepthCameraScan
    {
        std::vector<DiagnosisInfoDepthCamScanData> depthCameraScan;
        Pose depthCameraPose;
    };

    struct DiagnosisInfoSensor
    {
        unsigned int sensorId;
        std::string sensorType;
        Pose sensorPose;
        float rawDist;
        bool valid;
    };
    typedef std::vector<DiagnosisInfoSensor> DiagnosisInfoSensorsData;

    struct DiagnosisInfoInternalSystemEvent
    {
        InternalSystemEvent internalSystemEvent;
    };
  
    struct DiagnosisInfoMapDiscrepancy
    {
        std::vector<rpos::core::Vector2i> indexes;
        std::vector<rpos::core::RectangleF> areas;
        std::string task;
    };

    template<class T>
    struct DiagnosisInfoNameTrait
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<DiagnosisInfoLidarScan>
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<DiagnosisInfoSensorsData>
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<DiagnosisInfoDepthCameraScan>
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<std::string>
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<DiagnosisInfoInternalSystemEvent>
    {
        static const char* name;
    };

    template<>
    struct RPOS_CORE_API DiagnosisInfoNameTrait<DiagnosisInfoMapDiscrepancy>
    {
        static const char* name;
    };
}}