
#pragma once

#include "server_params.h"
#include "server_work_data.h"

#include <tf2_ros/transform_broadcaster.h>

#include <rpos/robot_platforms/slamware_core_platform.h>

#include <boost/chrono.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace slamware_ros_sdk {

    class SlamwareRosSdkServer;

    class ServerWorkerBase
    {
    public:
        typedef boost::chrono::steady_clock         clock_t;
        typedef clock_t::time_point                 time_point_t;

        typedef rpos::robot_platforms::SlamwareCorePlatform     slamware_platform_t;

    public:
        ServerWorkerBase(SlamwareRosSdkServer* pRosSdkServer
            , const std::string& wkName
            , const boost::chrono::milliseconds& triggerInterval
            );
        virtual ~ServerWorkerBase();

        const std::string& getWorkerName() const { return workerName_; }

        time_point_t getNextTimepointToTrigger() const { return nextTimepointToTrigger_; }
        
        bool isWorkLoopInitOk() const { return isWorkLoopInitOk_; }
        virtual void resetOnWorkLoopBegin();
        virtual bool reinitWorkLoop();

        virtual void checkToPerform();

    protected:
        virtual void doPerform() = 0;

    protected:
        SlamwareRosSdkServer* rosSdkServer() const { return rosSdkServer_; }

        std::shared_ptr<rclcpp::Node> rosNodeHandle() const;
        const ServerParams& serverParams() const;
        std::unique_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster() const;

        ServerWorkData_ConstPtr workData() const;
        ServerWorkData_Ptr mutableWorkData();

    private:
        SlamwareRosSdkServer* rosSdkServer_;
        std::string workerName_;

    protected:
        bool isWorkLoopInitOk_;
        boost::chrono::milliseconds triggerInterval_;
        time_point_t nextTimepointToTrigger_;
    };

    typedef boost::shared_ptr<ServerWorkerBase>         ServerWorkerBase_Ptr;
    
}
