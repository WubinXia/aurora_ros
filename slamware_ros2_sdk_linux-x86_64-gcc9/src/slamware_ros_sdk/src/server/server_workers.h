
#pragma once

#include "server_worker_base.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include "nav_msgs/srv/get_map.hpp"
#include <nav_msgs/msg/path.hpp>
#include <slamware_ros_sdk/msg/system_status.hpp>
#include <slamware_ros_sdk/msg/relocalization_status.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <aurora_pubsdk_inc.h>
using namespace rp::standalone::aurora;

namespace slamware_ros_sdk {

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotDeviceInfoWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerRobotDeviceInfoWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotDeviceInfoWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<slamware_ros_sdk::msg::RobotDeviceInfo>::SharedPtr pubRobotDeviceInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerRobotPoseWorker: public ServerWorkerBase
    {
    public:
        ServerRobotPoseWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotPoseWorker();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubRobotPose_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapUpdateWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase          super_t;

    public:
        ServerExploreMapUpdateWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerExploreMapUpdateWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rpos::features::location_provider::Map getMapByPltfm_(slamware_platform_t& pltfm, const rpos::core::RectangleF& area) const;

        void requestReinitMap_();
        bool checkToReinitMap_(slamware_platform_t& pltfm, const ServerWorkData_Ptr& wkDat);

        bool checkRecvResolution_(float recvResolution, const ServerWorkData_Ptr& wkDat);

        bool updateMapInCellIdxRect_(slamware_platform_t& pltfm
            , const rpos::core::RectangleI& reqIdxRect
            , const ServerWorkData_Ptr& wkDat
            );

        bool syncWholeMap_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            );

        bool updateMapNearRobot_(const ServerParams& srvParams
            , slamware_platform_t& pltfm
            , const ServerWorkData_Ptr& wkDat
            );

    private:
        bool shouldReinitMap_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerExploreMapPublishWorker: public ServerWorkerBase
    {
    public:
        ServerExploreMapPublishWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerExploreMapPublishWorker();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapDat_;
        rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pubMapInfo_;
    };

    //////////////////////////////////////////////////////////////////////////

    class ServerLaserScanWorker: public ServerWorkerBase
    {
    public:
        ServerLaserScanWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerLaserScanWorker();

    protected:
        virtual void doPerform();

    private:
        void fillRangeMinMaxInMsg_(const std::vector<rpos::core::LaserPoint> & laserPoints
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;

        float calcAngleInNegativePiToPi_(float angle) const;
        
        std::uint32_t calcCompensateDestIndexBySrcAngle_(float srcAngle
            , bool isAnglesReverse
            ) const;
        bool isSrcAngleMoreCloseThanOldSrcAngle_(float srcAngle, float destAngle, float oldSrcAngle) const;
        void compensateAndfillRangesInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , bool isClockwise
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;
        void compensateAndfillRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , bool isClockwise
            , sensor_msgs::msg::LaserScan& msgScan
            , std::vector<float>& tmpSrcAngles
            ) const;
        void fillOriginalRangesInMsg_(const std::vector<rpos::core::LaserPoint>& laserPoints
            , bool isLaserDataReverse
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;
        void fillOriginalRangeInMsg_(const rpos::core::LaserPoint& laserPoint
            , int index
            , sensor_msgs::msg::LaserScan& msgScan
            ) const;

    private:
        std::uint32_t compensatedAngleCnt_;
        float absAngleIncrement_;

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaserScan_;
        std::uint64_t latestLidarStartTimestamp_;
    };

    class ServerRobotBasicStateWorker: public ServerWorkerBase
    {
    public:
        ServerRobotBasicStateWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerRobotBasicStateWorker();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<slamware_ros_sdk::msg::RobotBasicState>::SharedPtr pubRobotBasicState_;
    };
    
    class ServerReadEventsWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        ServerReadEventsWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerReadEventsWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        boost::shared_ptr<rpos::robot_platforms::objects::SystemEventProvider> systemEventProvider_;
    };


    class ServerSystemStatusWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        ServerSystemStatusWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerSystemStatusWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<slamware_ros_sdk::msg::SystemStatus>::SharedPtr pubSystemStatus_;
        rclcpp::Publisher<slamware_ros_sdk::msg::RelocalizationStatus>::SharedPtr pubRelocalizaitonStatus_;
        boost::optional<slamtec_aurora_sdk_device_status_t> lastDeviceStatus_;
        boost::optional<slamtec_aurora_sdk_relocalization_status_types> lastRelocalizationStatus_;
    };   

    //////////////////////////////////////////////////////////////////////////

    class ServerStereoImageWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        ServerStereoImageWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerStereoImageWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubLeftImage_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubRightImage_;
        //publish key points
        // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubLefKeyPoints_;
        // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pubRightKeyPoints_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubStereoKeyPoints_;
    };

    // // lef riht camera info pub
    // class ServerStereoCameraInfoWorker: public ServerWorkerBase
    // {
    // public:
    //     typedef ServerWorkerBase super_t;
        
    // public:
    //     ServerStereoCameraInfoWorker(SlamwareRosSdkServer* pRosSdkServer
    //         , const std::string& wkName
    //         , const boost::chrono::milliseconds& triggerInterval
    //         );
    //     virtual ~ServerStereoCameraInfoWorker();

    //     virtual bool reinitWorkLoop();

    // protected:
    //     virtual void doPerform();

    // private:
    //     rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubLeftCameraInfo_;
    //     rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubRightCameraInfo_;
    // };

    class ServerImuRawDataWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        ServerImuRawDataWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerImuRawDataWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        uint64_t lastTimestamp_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImuRawData_;
    };

    class ServerPointCloudWorker : public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;

    public:
        ServerPointCloudWorker(SlamwareRosSdkServer *pRosSdkServer, const std::string &wkName, const boost::chrono::milliseconds &triggerInterval);
        virtual ~ServerPointCloudWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointCloud_;
    };

    //////////////////////////////////////////////////////////////////////////
    
    class RosConnectWorker: public ServerWorkerBase
    {
    public:
        typedef ServerWorkerBase super_t;
        
    public:
        RosConnectWorker(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~RosConnectWorker();

        virtual bool reinitWorkLoop();

    protected:
        virtual void doPerform();

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubRosConnect_;
        
    };

    //////////////////////////////////////////////////////////////////////////
    
}
