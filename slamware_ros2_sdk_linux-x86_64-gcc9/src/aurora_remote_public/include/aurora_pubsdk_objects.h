/*
 *  SLAMTEC Aurora
 *  Copyright 2013 - 2024 SLAMTEC Co., Ltd.
 *
 *  http://www.slamtec.com
 *
 *  Aurora Remote SDK
 *  Data Objects
 *
 */


#pragma once
/**
 * @defgroup SDK_Basic_Data_Types SDK Basic Data Types
 * @brief Basic data types used in the SDK
 *
 * @{
 */


/**
 * @brief The session handle type
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The session handle is used to identify a session.
 */
typedef void* slamtec_aurora_sdk_session_handle_t;

/**
 * @brief The map storage session handle type
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map storage session handle is used to identify a map storage session.
 */
typedef void* slamtec_aurora_sdk_mapstorage_session_handle_t;

/**
 * @brief Check if the handle is valid
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details Caller can use this function to check if the handle is valid.
 */
static inline int slamtec_aurora_sdk_is_valid_handle(void* handle) {
    return handle != NULL;
}


/**
 * @brief The version info structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The version info structure contains the version information of the SDK.
 */
typedef struct _slamtec_aurora_sdk_version_info_t
{
    /**
     * @brief The SDK name
     * @details The SDK name is the name of the SDK.
     */
    const char* sdk_name; 

    /**
     * @brief The SDK version string
     * @details The SDK version string is the version string of the SDK.
     */
    const char* sdk_version_string;

    /**
     * @brief The SDK build time
     * @details The SDK build time is the build time of the SDK.
     */
    const char* sdk_build_time;

    /**
     * @brief The SDK feature flags
     * @details The SDK feature flags are the feature flags of the SDK.
     */
    uint32_t sdk_feature_flags;
} slamtec_aurora_sdk_version_info_t;


/**
 * @brief The session config structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The session config structure contains the session configuration.
 */
typedef struct _slamtec_aurora_sdk_session_config_t
{
    /**
     * @brief The session config version
     * @details The session config version is the version of the session config. Set to 0 for now.
     */
    uint32_t version;

    /**
     * @brief The reserved field
     * @details The reserved field must be 0.
     */
    uint32_t reserved; // must be 0
} slamtec_aurora_sdk_session_config_t;


/**
 * @brief The connection info structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The connection info structure describes a connection method to a server.
 */
typedef struct _slamtec_aurora_sdk_connection_info_t
{
    /**
     * @brief The protocol type
     * @details The protocol type is the protocol type of the connection. For example "tcp", "udp".
     * @details The protocol type must be null-terminated.
     * @details Set to SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PROTOCOL for most cases.
     */
    char protocol_type[16]; //null-terminated string

    /**
     * @brief The address
     * @details The address is the address of the connection.
     * @details The address must be null-terminated.
     */
    char address[64]; // null-terminated string

    /**
     * @brief The port
     * @details The port is the port of the connection.
     * @details Set to SLAMTEC_AURORA_SDK_REMOTE_SERVER_DEFAULT_PORT for most cases.
     */
    uint16_t port; 
} slamtec_aurora_sdk_connection_info_t;


/**
 * @brief The server connection info structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The server connection info structure describes multiple connection methods to a server.
 */
typedef struct _slamtec_aurora_sdk_server_connection_info_t
{
    // multiple connection method of a server can be supported
    // for example, a server can support both tcp and udp
    // or a server can support both ipv4 and ipv6

    // the sdk will try to connect to the server with the first connection method
    /**
     * @brief The connection methods
     * @details The connection methods are the connection methods to the server.
     * @details Multiple connection method of a server can be supported.
     * @details The sdk will try to connect to the server with the first connection method.
     * @details The maximum number of connection methods is 8.
     */
    slamtec_aurora_sdk_connection_info_t connection_info[8];

    /**
     * @brief The number of connection methods
     * @details The number of connection methods is the number of connection methods to the server.
     */
    uint32_t connection_count;
} slamtec_aurora_sdk_server_connection_info_t;

// -- map storage related

/**
 * @brief The map storage session type
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map storage session type should be a value selected from enum slamtec_aurora_sdk_mapstorage_session_type_types.
 */
typedef uint32_t slamtec_aurora_sdk_mapstorage_session_type_t;


/**
 * @brief The map storage session type
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map storage session type is the type of the map storage session.
 */
enum slamtec_aurora_sdk_mapstorage_session_type_types {
    /**
     * @brief The upload session type
     * @details This type tells the SDK to upload a map to the server.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_TYPE_UPLOAD = 0,
    /**
     * @brief The download session type
     * @details This type tells the SDK to download a map from the server.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_TYPE_DOWNLOAD = 1,
};


/**
 * @brief The map storage session status flags
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map storage session status flags are the status flags of the map storage session.
 */
enum slamtec_aurora_sdk_mapstorage_session_status_flags_t {
    /**
     * @brief The finished status flag
     * @details This flag tells the SDK that the map storage session is finished without any error.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_FINISHED = 2,
    /**
     * @brief The working status flag
     * @details This flag tells the SDK that the map storage session is working.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_WORKING = 1,
    /**
     * @brief The idle status flag
     * @details This flag tells the SDK that the map storage session is idle.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_IDLE = 0,
    /**
     * @brief The failed status flag
     * @details This flag tells the SDK that the previous map storage session is failed.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_FAILED = -1,
    /**
     * @brief The aborted status flag
     * @details This flag tells the SDK that the previous map storage session is aborted.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_ABORTED = -2,
    /**
     * @brief The rejected status flag
     * @details This flag tells the SDK that the previous map storage session is rejected.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_REJECTED = -3,
    /**
     * @brief The timeout status flag
     * @details This flag tells the SDK that the previous map storage session is timeout.
     */
    SLAMTEC_AURORA_SDK_MAPSTORAGE_SESSION_STATUS_TIMEOUT = -4,
};

/**
 * @brief The map storage session status structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map storage session status structure contains the status of the map storage session.
 */
typedef struct _slamtec_aurora_sdk_mapstorage_session_status_t {
    /**
     * @brief The progress of the map storage session
     * @details The progress of the map storage session is the progress of the map storage session.
     * @details The progress is a value between 0 and 100.
     */
    float progress; //0-100

    /**
     * @brief The status flags of the map storage session
     * @details The status flags of the map storage session are the status flags of the map storage session.
     * @details The status flags should be a value selected from enum slamtec_aurora_sdk_mapstorage_session_status_flags_t.
     */
    int8_t flags; // value selected from enum slamtec_aurora_sdk_mapstorage_session_status_flags_t
} slamtec_aurora_sdk_mapstorage_session_status_t;


// -- tracking and mapping data

/**
 * @brief The 3D position structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The 3D position structure contains the 3D position.
 */
typedef struct _slamtec_aurora_sdk_position3d_t {
    /**
     * @brief The x coordinate
     * @details The x coordinate is the x coordinate of the 3D position.
     */
    double x;
    /**
     * @brief The y coordinate
     * @details The y coordinate is the y coordinate of the 3D position.
     */
    double y;
    /**
     * @brief The z coordinate
     * @details The z coordinate is the z coordinate of the 3D position.
     */
    double z;
} slamtec_aurora_sdk_position3d_t;;

/**
 * @brief The quaternion structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The quaternion structure contains the quaternion of a rotation.
 */ 
typedef struct _slamtec_aurora_sdk_quaternion_t {
    /**
     * @brief The x component of the quaternion
     * @details The x component of the quaternion is the x component of the quaternion.
     */
    double x;
    /**
     * @brief The y component of the quaternion
     * @details The y component of the quaternion is the y component of the quaternion.
     */
    double y;
    /**
     * @brief The z component of the quaternion
     * @details The z component of the quaternion is the z component of the quaternion.
     */
    double z;
    /**
     * @brief The w component of the quaternion
     * @details The w component of the quaternion is the w component of the quaternion.
     */
    double w;
} slamtec_aurora_sdk_quaternion_t;

/**
 * @brief The Euler angle structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The Euler angle structure contains the Euler angle of a rotation.
 */
typedef struct _slamtec_aurora_sdk_euler_angle_t {
    /**
     * @brief The roll angle
     * @details The roll angle is the roll angle of the Euler angle.
     */
    double roll;
    /**
     * @brief The pitch angle
     * @details The pitch angle is the pitch angle of the Euler angle.
     */
    double pitch;
    /**
     * @brief The yaw angle
     * @details The yaw angle is the yaw angle of the Euler angle.
     */
    double yaw;
} slamtec_aurora_sdk_euler_angle_t;

/**
 * @brief The SE3 pose structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The SE3 pose structure contains the SE3 pose of a transformation.
 */
typedef struct _slamtec_aurora_sdk_pose_se3_t {
    /**
     * @brief The translation of the SE3 pose
     * @details The translation of the SE3 pose is the translation of the SE3 pose.
     */
    slamtec_aurora_sdk_position3d_t translation;
    /**
     * @brief The quaternion of the SE3 pose
     * @details The quaternion of the SE3 pose is the quaternion of the SE3 pose.
     */
    slamtec_aurora_sdk_quaternion_t quaternion;
} slamtec_aurora_sdk_pose_se3_t;

/**
 * @brief The pose structure using translation and Euler angle
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The pose structure using translation and Euler angle contains the pose of a transformation.
 */
typedef struct _slamtec_aurora_sdk_pose_t {
    /**
     * @brief The translation of the pose
     * @details The translation of the pose is the translation of the pose.
     */
    slamtec_aurora_sdk_position3d_t translation;
    /**
     * @brief The Euler angle of the pose
     * @details The Euler angle of the pose is the Euler angle of the pose.
     */
    slamtec_aurora_sdk_euler_angle_t rpy;
} slamtec_aurora_sdk_pose_t;


/**
 * @brief The mapping flag types
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details Multiple mapping flags can be combined using bitwise OR.
 */
enum slamtec_aurora_sdk_mapping_flag_types {
    /**
     * @brief The none mapping flag
     */
    SLAMTEC_AURORA_SDK_MAPPING_FLAG_NONE = 0,
    /**
     * @brief The local mapping flag
     * @details This flag tells the SDK that the mapping is in local mode (mapping disabled).
     */
    SLAMTEC_AURORA_SDK_MAPPING_FLAG_LOC_MODE = (0x1 << 0),
    /**
     * @brief The loop closure disabled mapping flag
     * @details This flag tells the SDK that the loop closure is disabled.
     */
    SLAMTEC_AURORA_SDK_MAPPING_FLAG_LC_DISABLED = (0x1 << 2),
    /**
     * @brief The lost mapping flag
     * @details This flag tells the SDK that the tracking is lost.
     */
    SLAMTEC_AURORA_SDK_MAPPING_FLAG_LOSTED = (0x1 << 16),
    /**
     * @brief The storage in progress mapping flag
     * @details This flag tells the SDK that the mapping is in storage in progress.
     */
    SLAMTEC_AURORA_SDK_MAPPING_FLAG_STORAGE_IN_PROGRESS = (0x1 << 17),
};

/**
 * @brief The mapping flag value
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The mapping flag value is the value of the mapping flag.
 */
typedef uint32_t slamtec_aurora_sdk_mapping_flag_t; //bitwise OR of enum slamtec_aurora_sdk_mapping_flag_types

/**
 * @brief The device status types
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The device status types are the status types of the device.
 */
enum slamtec_aurora_sdk_device_status_types {
    /**
     * @brief The device has been inited
     */
    SLAMTEC_AURORA_SDK_DEVICE_INITED = 0,
    /**
     * @brief The device initialization failed
     */
    SLAMTEC_AURORA_SDK_DEVICE_INIT_FAILED,
    /**
     * @brief The device has detected a loop closure
     */
    SLAMTEC_AURORA_SDK_DEVICE_LOOP_CLOSURE,
    /**
     * @brief The device optimization completed
     */
    SLAMTEC_AURORA_SDK_DEVICE_OPTIMIZATION_COMPLETED,
    /**
     * @brief The device tracking is lost
     */
    SLAMTEC_AURORA_SDK_DEVICE_TRACKING_LOST,
    /**
     * @brief The device tracking is recovered
     */
    SLAMTEC_AURORA_SDK_DEVICE_TRACKING_RECOVERED,
    /**
     * @brief The device map is updated
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_UPDATED,
    /**
     * @brief The device map is cleared
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_CLEARED,
    /**
     * @brief The device map is switched
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_SWITCHED,
    /**
     * @brief The device map loading started
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_STARTED,
    /**
     * @brief The device map saving started
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_STARTED,
    /**
     * @brief The device map loading completed
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_LOADING_COMPLETED,
    /**
     * @brief The device map saving completed
     */
    SLAMTEC_AURORA_SDK_DEVICE_MAP_SAVING_COMPLETED,
};

/**
 * @brief The device status value
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The device status value is the value of the device status.
 */
typedef uint32_t slamtec_aurora_sdk_device_status_t; //value selected from enum slamtec_aurora_sdk_device_status_types

/**
 * @brief The device status structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The device status structure contains the status of the device.
 */
typedef struct slamtec_aurora_sdk_device_status
{
    slamtec_aurora_sdk_device_status_t status;
    uint64_t timestamp_ns;
};

enum slamtec_aurora_sdk_relocalization_status_types {
    SLAMTEC_AURORA_SDK_RELOCALIZATION_NONE = 0,
    SLAMTEC_AURORA_SDK_RELOCALIZATION_STARTED,
    SLAMTEC_AURORA_SDK_RELOCALIZATION_SUCCEED,
    SLAMTEC_AURORA_SDK_RELOCALIZATION_FAILED,
    SLAMTEC_AURORA_SDK_RELOCALIZATION_ABORTED
};

typedef struct slamtec_aurora_sdk_relocalization_status
{
    slamtec_aurora_sdk_relocalization_status_types status;
    uint64_t timestamp_ns;
};

/**
 * @brief The image description structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The image description structure contains the description of an image.
 */
typedef struct _slamtec_aurora_sdk_image_desc_t {
    /**
     * @brief The width of the image
     * @details The width of the image is the width of the image.
     */
    uint32_t width;
    /**
     * @brief The height of the image
     * @details The height of the image is the height of the image.
     */
    uint32_t height;
    /**
     * @brief The stride of the image
     * @details The stride of the image is the stride of the image.
     */
    uint32_t stride;
    /**
     * @brief The format of the image
     * @details The format of the image is the format of the image.
     * @details 0: gray, 1: rgb, 2: rgba
     */
    uint32_t format; // 0: gray, 1: rgb, 2: rgba
    /**
     * @brief The size of the image data in bytes
     * @details The size of the image data is the size of the image data in bytes.
     */
    uint32_t data_size;
} slamtec_aurora_sdk_image_desc_t;


/**
 * @brief The keypoint structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The keypoint structure contains the keypoint of an image.
 */
typedef struct _slamtec_aurora_sdk_keypoint_t {
    /**
     * @brief The x coordinate of the keypoint
     * @details The x coordinate of the keypoint is the x coordinate of the keypoint.
     */
    float x;
    /**
     * @brief The y coordinate of the keypoint
     * @details The y coordinate of the keypoint is the y coordinate of the keypoint.
     */
    float y;
    /**
     * @brief The flags of the keypoint
     * @details The flags of the keypoint are the flags of the keypoint.
     * @details 0: unmatched, non-zero: matched
     */
    uint8_t flags;  // 0: unmatched, non-zero: matched
} slamtec_aurora_sdk_keypoint_t;


/**
 * @brief The tracking data buffer structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The tracking data buffer structure contains the buffer to hold image data and keypoints of a tracking frame
 * @details The buffer should be provided by the caller when invoking the peek interface
 */
typedef struct _slamtec_aurora_sdk_tracking_data_buffer_t {
    /**
     * @brief The buffer to hold image data of the left camera
     * @details The buffer should be provided by the caller, 
     * @details nullptr to disable image data copy
     */
    void* imgdata_left; //buffer to hold image data of the left camera
                        //the buffer should be provided by the caller
                        //nullptr to disable image data copy

    /**
     * @brief The size of the buffer to hold image data of the left camera
     * @details The size of the buffer is the size of the buffer.
     */
    size_t imgdata_left_size; // size of the buffer


    /**
     * @brief The buffer to hold image data of the right camera
     * @details The buffer should be provided by the caller, 
     * @details nullptr to disable image data copy
     */
    void* imgdata_right; 
    /**
     * @brief The size of the buffer to hold image data of the right camera
     * @details The size of the buffer is the size of the buffer.
     */
    size_t imgdata_right_size;


    /**
     * @brief The buffer to hold keypoints of the left camera
     * @details The buffer should be provided by the caller
     */
    slamtec_aurora_sdk_keypoint_t* keypoints_left; //buffer to hold keypoints of the left camera
    /**
     * @brief The size of the buffer to hold keypoints of the left camera
     * @details The size of the buffer is the size of the buffer.
     */
    size_t keypoints_left_buffer_count; // size of the buffer

    /**
     * @brief The buffer to hold keypoints of the right camera
     * @details The buffer should be provided by the caller
     */
    slamtec_aurora_sdk_keypoint_t* keypoints_right;

    /**
     * @brief The size of the buffer to hold keypoints of the right camera
     * @details The size of the buffer is the size of the buffer.
     */
    size_t keypoints_right_buffer_count;
} slamtec_aurora_sdk_tracking_data_buffer_t;


/**
 * @brief The tracking status types
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The tracking status types are the status types of the tracking.
 */
enum slamtec_aurora_sdk_tracking_status_t {
    SLAMTEC_AURORA_TRACKING_STATUS_UNKNOWN = 0,
    SLAMTEC_AURORA_TRACKING_STATUS_SYS_NOT_READY,
    SLAMTEC_AURORA_TRACKING_STATUS_NOT_INIT,
    SLAMTEC_AURORA_TRACKING_STATUS_NO_IMG,
    SLAMTEC_AURORA_TRACKING_STATUS_OK,
    SLAMTEC_AURORA_TRACKING_STATUS_LOST,
    SLAMTEC_AURORA_TRACKING_STATUS_LOST_RECOVERED,
};




/**
 * @brief The tracking info structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The tracking info structure contains the tracking information.
 * @details NOTICE: the image provided by tracking info has been lossy compressed during transmission
 * @details to retrieve the original image, please use the raw image data callback
 * @details The raw image subscription must be enabled first
 */
typedef struct _slamtec_aurora_sdk_tracking_info {
    /**
     * @brief The timestamp of the tracking
     * @details The nanoseconds timestamp of the tracking frame
     */
    uint64_t timestamp_ns;

    /**
     * @brief The description of the left image
     * @details The description of the left image is the description of the left image.
     */
    slamtec_aurora_sdk_image_desc_t left_image_desc;
    /**
     * @brief The description of the right image
     * @details The description of the right image is the description of the right image.
     */
    slamtec_aurora_sdk_image_desc_t right_image_desc;

    /**
     * @brief Whether the tracking is stereo
     */
    uint32_t is_stereo;

    /**
     * @brief The tracking status value
     * @details The tracking status value is the value of the slamtec_aurora_sdk_tracking_status_t
     */
    uint32_t tracking_status; // from slamtec_aurora_sdk_tracking_status_t

    /**
     * @brief The pose of the tracking (base to world)
     * @details The pose of the tracking is the pose of the tracking.
     */
    slamtec_aurora_sdk_pose_se3_t pose;

    /**
     * @brief The count of the keypoints of the left image
     * @details The count of the keypoints of the left image is the count of the keypoints of the left image.
     */
    uint32_t keypoints_left_count;
    /**
     * @brief The count of the keypoints of the right image
     * @details The count of the keypoints of the right image is the count of the keypoints of the right image.
     */
    uint32_t keypoints_right_count;
} slamtec_aurora_sdk_tracking_info_t;


/**
 * @brief The IMU data structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The IMU data structure contains the IMU data.
 */
typedef struct _slamtec_aurora_sdk_imu_data_t {
    /**
     * @brief The timestamp of the IMU data
     * @details The nanoseconds timestamp of the IMU data
     */
    uint64_t timestamp_ns;
    /**
     * @brief The ID of the IMU
     * @details The ID of the IMU is the ID of the IMU.
     */
    uint32_t imu_id;
    /**
     * @brief The acceleration data of the IMU
     * @details The acceleration data of the IMU is the acceleration data of the IMU.
     * @details in g (1g = 9.8m/s^2)
     */
    double acc[3];  // in g (1g = 9.8m/s^2)
    /**
     * @brief The gyro data of the IMU
     * @details The gyro data of the IMU is the gyro data of the IMU.
     * @details in dps
     */
    double gyro[3]; // in dps
} slamtec_aurora_sdk_imu_data_t;


/**
 * @brief The IMU info structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The IMU info structure contains the IMU info.
 */
typedef struct _slamtec_aurora_sdk_imu_info_t {
    /**
     * @brief Whether the IMU data is valid
     * @details non-zero for valid data
     */
    int valid; // non-zero for valid data


    /**
     * @brief The transform from base to camera
     */
    slamtec_aurora_sdk_pose_se3_t tcb;

    /**
     * @brief The transform from IMU to camera
     */
    slamtec_aurora_sdk_pose_se3_t tc_imu;

    /**
     * @brief The covariance of the noise
     * @details The covariance of the noise is the covariance of the noise.
     */
    double cov_noise[6]; // gyro to accel
    /**
     * @brief The covariance of the random walk
     * @details The covariance of the random walk is the covariance of the random walk.
     */
    double cov_random_walk[6]; // gyro to accel
} slamtec_aurora_sdk_imu_info_t;



// -- Map Objects

/**
 * @brief The global map description structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The global map description structure contains the global map description.
 */
typedef struct _slamtec_aurora_sdk_global_map_desc_t {
    /**
     * @brief The count of the map points to fetch currently in the background thread
     */
    uint64_t lastMPCountToFetch;
    /**
     * @brief The count of the keyframes to fetch currently in the background thread
     */
    uint64_t lastKFCountToFetch;
    /**
     * @brief The count of the maps to fetch currently in the background thread
     */
    uint64_t lastMapCountToFetch;

    /**
     * @brief The count of the map points retrieved currently in the background thread
     */
    uint64_t lastMPRetrieved;
    /**
     * @brief The count of the keyframes retrieved currently in the background thread
     */
    uint64_t lastKFRetrieved;

    /**
     * @brief The total count of the map points
     */
    uint64_t totalMPCount;
    /**
     * @brief The total count of the keyframes
     */
    uint64_t totalKFCount;
    /**
     * @brief The total count of the maps
     */
    uint64_t totalMapCount;

    /**
     * @brief The total count of the map points fetched
     */
    uint64_t totalMPCountFetched;
    /**
     * @brief The total count of the keyframes fetched
     */
    uint64_t totalKFCountFetched;
    /**
     * @brief The total count of the maps fetched
     */
    uint64_t totalMapCountFetched;

    /**
     * @brief The current count of the active map points
     */
    uint64_t currentActiveMPCount;
    /**
     * @brief The current count of the active keyframes
     */
    uint64_t currentActiveKFCount;

    /**
     * @brief The ID of the active map
     */
    uint32_t activeMapID;
} slamtec_aurora_sdk_global_map_desc_t;


/**
 * @brief The map description structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map description structure contains the map description.
 */
typedef struct _slamtec_aurora_sdk_map_desc_t {
    /**
     * @brief The ID of the map
     */
    uint32_t map_id;
    /**
     * @brief The flags of the map
     */
    uint32_t map_flags;
    /**
     * @brief The count of the keyframes in the map
     */
    uint64_t keyframe_count;
    /**
     * @brief The count of the map points in the map
     */
    uint64_t map_point_count;
    
    /**
     * @brief The ID of the first keyframe in the map
     */
    uint64_t keyframe_id_start;
    /**
     * @brief The ID of the last keyframe in the map
     */
    uint64_t keyframe_id_end;

    /**
     * @brief The ID of the first map point in the map
     */
    uint64_t map_point_id_start;
    /**
     * @brief The ID of the last map point in the map
     */
    uint64_t map_point_id_end;
} slamtec_aurora_sdk_map_desc_t;


/**
 * @brief The keyframe description structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The keyframe description structure contains the keyframe description.
 */
typedef struct _slamtec_aurora_sdk_keyframe_desc_t {
    /**
     * @brief The ID of the keyframe
     */
    uint64_t id;
    /**
     * @brief The ID of the parent keyframe
     */
    uint64_t parent_id;
    /**
     * @brief The ID of the map
     */
    uint32_t map_id;

    /**
     * @brief The timestamp of the keyframe
     */
    double timestamp;

    /**
     * @brief The pose of the keyframe (base to world)
     */
    slamtec_aurora_sdk_pose_se3_t pose_se3;
    slamtec_aurora_sdk_pose_t pose;

    /**
     * @brief The count of the looped frames
     */
    size_t looped_frame_count;
    /**
     * @brief The count of the connected frames
     */
    size_t connected_frame_count;
} slamtec_aurora_sdk_keyframe_desc_t;


/**
 * @brief The map point description structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map point description structure contains the map point description.
 */
typedef struct _slamtec_aurora_sdk_map_point_desc_t {
    /**
     * @brief The ID of the map point
     */
    uint64_t id;
    /**
     * @brief The ID of the map
     */
    uint32_t map_id;

    /**
     * @brief The timestamp of the map point
     */
    double timestamp;

    /**
     * @brief The position of the map point
     */
    slamtec_aurora_sdk_position3d_t position;
    /**
     * @brief The flags of the map point
     */
    uint32_t flags;
} slamtec_aurora_sdk_map_point_desc_t;

// callbacks

/**
 * @brief The map storage session result callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The map storage session result callback is the callback for the map storage session result.
 * @param user_data The user data to be passed to the callback
 * @param is_ok Whether the map storage session is ok
 */
typedef void (*slamtec_aurora_sdk_mapstorage_session_result_callback_t)(void* user_data, int is_ok);

/**
 * @brief The image data callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The image data callback to receive the raw image data from the device.
 * @param user_data The user data to be passed to the callback
 * @param timestamp_ns The timestamp of the image
 * @param left_desc The description of the left image
 * @param left_data The data of the left image
 * @param right_desc The description of the right image
 * @param right_data The data of the right image
 */
typedef void (*slamtec_aurora_sdk_on_image_data_callback_t)(void* user_data, uint64_t timestamp_ns, const slamtec_aurora_sdk_image_desc_t* left_desc, const void* left_data, const slamtec_aurora_sdk_image_desc_t* right_desc, const void* right_data);

/**
 * @brief The tracking data callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The tracking data callback to receive the tracking data from the device.
 * @param user_data The user data to be passed to the callback
 * @param tracking_data The tracking data
 * @param provided_buffer_info The provided buffer of the tracking data like image data and keypoints, this buffer is provided by the SDK, the buffer will be invalidated after the callback returns
 */
typedef void (*slamtec_aurora_sdk_on_tracking_data_callback_t)(void* user_data, const slamtec_aurora_sdk_tracking_info_t* tracking_data, const slamtec_aurora_sdk_tracking_data_buffer_t* provided_buffer_info);

/**
 * @brief The IMU data callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The IMU data callback to receive the IMU data from the device.
 * @param user_data The user data to be passed to the callback
 * @param timestamp_ns The timestamp of the IMU data
 * @param status The status of the device
 */
typedef void (*slamtec_aurora_sdk_on_imu_data_callback_t)(void* user_data, const slamtec_aurora_sdk_imu_data_t* imu_data, size_t imu_data_count);


/**
 * @brief The mapping flags callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The mapping flags callback to receive the mapping flags from the device.
 * @param user_data The user data to be passed to the callback
 * @param mapping_flags The mapping flags
 */
typedef void (*slamtec_aurora_sdk_on_mapping_flags_callback_t)(void* user_data, slamtec_aurora_sdk_mapping_flag_t mapping_flags);

/**
 * @brief The device status callback
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The device status callback to receive the device status from the device.
 * @param user_data The user data to be passed to the callback
 * @param timestamp_ns The timestamp of the device status
 * @param status The status of the device
 */
typedef void (*slamtec_aurora_sdk_on_device_status_callback_t)(void* user_data, uint64_t timestamp_ns, slamtec_aurora_sdk_device_status_t status);


/**
 * @brief The listener structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The listener structure contains the listener.
 */
typedef struct _slamtec_aurora_sdk_listener_t {
    /**
     * @brief The user data to be passed to the callback
     */
    void* user_data;

    /**
     * @brief The callback for the raw image data, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_image_data_callback_t on_raw_image_data;
    /**
     * @brief The callback for the tracking data, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_tracking_data_callback_t on_tracking_data;
    /**
     * @brief The callback for the IMU data, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_imu_data_callback_t on_imu_data;
    /**
     * @brief The callback for the mapping flags, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_mapping_flags_callback_t on_mapping_flags;
    /**
     * @brief The callback for the device status, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_device_status_callback_t on_device_status;
    
} slamtec_aurora_sdk_listener_t;


// map visitor

/**
 * @brief The callback for accessing the keyframe data
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The callback to receive the keyframe data locally cached by the SDK
 */
typedef void (*slamtec_aurora_sdk_on_map_keyframe_callback_t)(void* user_data, const slamtec_aurora_sdk_keyframe_desc_t* keyframe_desc, const uint64_t * looped_frame_ids, const uint64_t * connected_frame_ids);
/**
 * @brief The callback for accessing the map point data
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The callback to receive the map point data locally cached by the SDK
 */
typedef void (*slamtec_aurora_sdk_on_map_point_callback_t)(void* user_data, const slamtec_aurora_sdk_map_point_desc_t* map_point_desc);

/**
 * @brief The callback for accessing the map description
 * @ingroup SDK_Callback_Types SDK Callback Types
 * @details The callback to receive the map description locally cached by the SDK
 */
typedef void (*slamtec_aurora_sdk_on_map_desc_callback_t)(void* user_data, const slamtec_aurora_sdk_map_desc_t* map_desc);


/**
 * @brief The map data visitor structure
 * @ingroup SDK_Basic_Data_Types SDK Basic Data Types
 * @details The map data visitor structure contains the map data visitor.
 */
typedef struct _slamtec_aurora_sdk_map_data_visitor_t {
    /**
     * @brief The user data to be passed to the callback
     */
    void* user_data;

    /**
     * @brief The callback for accessing the keyframe data, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_map_keyframe_callback_t on_keyframe;
    /**
     * @brief The callback for accessing the map point data, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_map_point_callback_t on_map_point;
    /**
     * @brief The callback for accessing the map description, set to NULL to ignore this callback
     */
    slamtec_aurora_sdk_on_map_desc_callback_t on_map_desc;
} slamtec_aurora_sdk_map_data_visitor_t;

/** @} */ // end of SDK_Basic_Data_Types
