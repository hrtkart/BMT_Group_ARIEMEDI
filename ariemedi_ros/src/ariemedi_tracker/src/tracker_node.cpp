#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include custom messages
#include "ariemedi_tracker/msg/marker_position.hpp"
#include "ariemedi_tracker/msg/marker_plane.hpp"
#include "ariemedi_tracker/msg/transformation.hpp"
#include "ariemedi_tracker/msg/tool_tracking_data.hpp"

#include "ARMDCombinedAPI.h"
#include "DeviceScan.h"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <cmath>
#include <cctype>

using namespace std::chrono_literals;

class TrackerNode : public rclcpp::Node
{
public:
    TrackerNode() : Node("ariemedi_tracker"), tracker_(new ARMDCombinedAPI()), tracking_started_(false),
                   frame_count_(0), last_frame_time_(0.0), start_time_(0.0),
                   last_tx_(0.0), last_ty_(0.0), last_tz_(0.0), has_last_data_(false)
    {
        this->declare_parameter<std::string>("hostname", "");
        this->declare_parameter<bool>("enable_imaging", false);
        this->declare_parameter<std::string>("base_frame_id", "tracker_base");
        this->declare_parameter<std::string>("default_tool_frame_id", "toolcali0");
        enable_imaging_ = this->get_parameter("enable_imaging").as_bool();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        default_tool_frame_id_ = this->get_parameter("default_tool_frame_id").as_string();
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Topic publisher
        armd_pub_ = this->create_publisher<ariemedi_tracker::msg::ToolTrackingData>("/ARMDpos", 10);

        initDevice();
    }

    ~TrackerNode()
    {
        stopTracking();
        delete tracker_;
    }

private:
    static std::string sanitizeFrameId(const std::string &raw_name, const std::string &fallback)
    {
        const std::string &source = raw_name.empty() ? fallback : raw_name;
        std::string clean;
        clean.reserve(source.size());

        for (const unsigned char c : source) {
            if (std::isalnum(c) || c == '_' || c == '/') {
                clean.push_back(static_cast<char>(c));
            } else {
                clean.push_back('_');
            }
        }

        while (!clean.empty() && clean.front() == '/') {
            clean.erase(clean.begin());
        }

        if (clean.empty()) {
            return fallback;
        }
        return clean;
    }

    void initDevice()
    {
        std::string hostname = this->get_parameter("hostname").as_string();

        if (hostname.empty()) {
            RCLCPP_INFO(this->get_logger(), "Scanning for RT devices...");
            DeviceScan deviceScan;
            
            int scan_attempts = 10;
            while (deviceScan.getDeviceInfo().empty() && scan_attempts > 0)
            {
                deviceScan.updateDeviceInfo();
                std::this_thread::sleep_for(500ms);
                scan_attempts--;
            }

            if (deviceScan.getDeviceInfo().empty()) {
                RCLCPP_ERROR(this->get_logger(), "No RT devices found during scan.");
                rclcpp::shutdown();
                return;
            }

            for (const auto& x : deviceScan.getDeviceInfo()) {
                hostname = x.first;
                RCLCPP_INFO(this->get_logger(), "Scanned RT device hostname: %s, IP: %s", hostname.c_str(), x.second.at(0).c_str());
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Connecting to device: %s...", hostname.c_str());
        RCLCPP_INFO(this->get_logger(), "[DEBUG] Using ARMDCombinedAPI connect() with hostname/IP");
        
        // Attempt connection with detailed error logging
        int errorCode = tracker_->connect(hostname, true, true, true);  // Enable error printing
        
        if (errorCode != 0) {
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "╔════════════════════════════════════════════╗");
            RCLCPP_ERROR(this->get_logger(), "║  CONNECTION FAILED - DIAGNOSTIC INFO        ║");
            RCLCPP_ERROR(this->get_logger(), "╚════════════════════════════════════════════╝");
            RCLCPP_ERROR(this->get_logger(), "Error Code: %d", errorCode);
            RCLCPP_ERROR(this->get_logger(), "Target: %s", hostname.c_str());
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "Possible causes:");
            
            if (errorCode == -1) {
                RCLCPP_ERROR(this->get_logger(), "  [-1] Connection refused");
                RCLCPP_ERROR(this->get_logger(), "      → Check if device IP/hostname is correct");
                RCLCPP_ERROR(this->get_logger(), "      → Verify device is powered on");
                RCLCPP_ERROR(this->get_logger(), "      → Check network connectivity: ping %s", hostname.c_str());
            } else if (errorCode == -2) {
                RCLCPP_ERROR(this->get_logger(), "  [-2] Connection timeout");
                RCLCPP_ERROR(this->get_logger(), "      → Network unreachable");
                RCLCPP_ERROR(this->get_logger(), "      → Check Ethernet cable connection");
                RCLCPP_ERROR(this->get_logger(), "      → Verify device and PC are on same subnet");
            } else if (errorCode == -3 || errorCode == -4) {
                RCLCPP_ERROR(this->get_logger(), "  [%d] Protocol/Communication error", errorCode);
                RCLCPP_ERROR(this->get_logger(), "      → Device firmware may be incompatible");
                RCLCPP_ERROR(this->get_logger(), "      → Try different protocol (TCP/UDP)");
            } else if (errorCode > 0) {
                RCLCPP_ERROR(this->get_logger(), "  [%d] Device-specific error code", errorCode);
                RCLCPP_ERROR(this->get_logger(), "      → Please check device manual for error code");
            }
            
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "Troubleshooting steps:");
            RCLCPP_ERROR(this->get_logger(), "  1. Run: bash diagnose_connection.sh %s", hostname.c_str());
            RCLCPP_ERROR(this->get_logger(), "  2. Check: CONNECTION_TROUBLESHOOTING.md");
            RCLCPP_ERROR(this->get_logger(), "");
            
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected successfully!");

        // Load the specific tool from package share directory
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("ariemedi_tracker");
        std::string tool_path = package_share_directory + "/tool/toolcali0.arom";
        RCLCPP_INFO(this->get_logger(), "Loading tool from: %s", tool_path.c_str());
        std::vector<std::string> tools;
        tools.push_back(tool_path);
        tracker_->loadPassiveToolAROM(tools);

        tracker_->setTrackingDataTransmissionType(TransmissionType::Passive);
        tracker_->startTracking();
        if (enable_imaging_) {
            tracker_->startImaging();
            RCLCPP_INFO(this->get_logger(), "Imaging enabled.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Imaging disabled for max tracking throughput.");
        }

        tracking_started_ = true;
        start_time_ = this->get_clock()->now().seconds();
        last_frame_time_ = start_time_;

        // Drive update/publish at 120Hz.
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 120.0),
            std::bind(&TrackerNode::step, this));

        RCLCPP_INFO(this->get_logger(), "120Hz tracking timer started.");
    }

    void stopTracking()
    {
        tracking_started_ = false;
        if (timer_) {
            timer_->cancel();
        }

        if (tracker_)
        {
            if (ConnectionStatus::Interruption == tracker_->getConnectionStatus())
            {
                tracker_->disconnect();
            }
            else
            {
                tracker_->stopTracking();
                    if (enable_imaging_) {
                        tracker_->stopImaging();
                    }
                tracker_->disconnect();
            }
        }
        RCLCPP_INFO(this->get_logger(), "Disconnected device and stopped tracking.");
    }

    void step()
    {
        if (!tracking_started_) {
            return;
        }

        if (ConnectionStatus::Interruption == tracker_->getConnectionStatus())
        {
            RCLCPP_ERROR(this->get_logger(), "Device connection interrupted!");
            tracking_started_ = false;
            if (timer_) {
                timer_->cancel();
            }
            return;
        }

        tracker_->trackingUpdate();
        processToolData();
    }

    void processToolData()
    {
        std::vector<MarkerPosition> allMarkerData = tracker_->getAllMarkers();
        std::vector<ToolTrackingData> toolData = tracker_->getTrackingData(allMarkerData);
        if (allMarkerData.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Loop updating, 0 markers found");
        }

        // Calculate timing once per timer tick, regardless of number of tools.
        const double current_time = this->get_clock()->now().seconds();
        const double frame_time_diff = current_time - last_frame_time_;
        const double total_time = current_time - start_time_;
        const double fps = (frame_count_ > 0 && total_time > 0.0) ? (frame_count_ / total_time) : 0.0;
        const double instant_fps = (frame_time_diff > 0.0) ? (1.0 / frame_time_diff) : 0.0;

        bool has_valid_tracking_data = false;

        if (!toolData.empty())
        {
            for (const auto& data : toolData)
            {
                if (data.matchStatus)
                {
                    // Deduplicate by SDK frame timestamp first; this is the most reliable way to
                    // avoid publishing cached/repeated frames when timer frequency is higher than
                    // device output frequency.
                    if (!data.timespec.empty() && data.timespec == last_timespec_)
                    {
                        continue;
                    }

                    // Fallback for SDKs that may not provide timespec reliably.
                    if (data.timespec.empty() &&
                        has_last_data_ &&
                        std::abs(data.transform.tx - last_tx_) < 0.01 &&
                        std::abs(data.transform.ty - last_ty_) < 0.01 &&
                        std::abs(data.transform.tz - last_tz_) < 0.01 &&
                        frame_time_diff < 0.005)
                    {
                        continue;
                    }

                    has_valid_tracking_data = true;
                    frame_count_++;

                    // Update last data for deduplication.
                    last_tx_ = data.transform.tx;
                    last_ty_ = data.transform.ty;
                    last_tz_ = data.transform.tz;
                    last_timespec_ = data.timespec;
                    has_last_data_ = true;

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Frame %lu | Position: X=%.5fmm Y=%.5fmm Z=%.5fmm | TimeDiff=%.5fms | InstFPS=%.5f | AvgFPS=%.5f | TS=%s",
                        frame_count_,
                        data.transform.tx,
                        data.transform.ty,
                        data.transform.tz,
                        frame_time_diff * 1000.0,
                        instant_fps,
                        fps,
                        data.timespec.c_str());

                    auto ros_msg = ariemedi_tracker::msg::ToolTrackingData();

                    ros_msg.header.stamp = this->get_clock()->now();
                    ros_msg.header.frame_id = base_frame_id_;

                    ros_msg.name = data.name;
                    ros_msg.timespec = data.timespec;
                    ros_msg.match_status = data.matchStatus;
                    ros_msg.matched_plane = data.matchedPlane;
                    ros_msg.matched_markers_num = data.matchedMarkersNum;
                    ros_msg.error = data.error;
                    ros_msg.fps = data.fps;

                    ros_msg.dir[0] = data.dir[0];
                    ros_msg.dir[1] = data.dir[1];
                    ros_msg.dir[2] = data.dir[2];

                    ros_msg.transform.status = data.transform.status;
                    ros_msg.transform.qw = data.transform.qw;
                    ros_msg.transform.qx = data.transform.qx;
                    ros_msg.transform.qy = data.transform.qy;
                    ros_msg.transform.qz = data.transform.qz;
                    ros_msg.transform.tx = data.transform.tx;
                    ros_msg.transform.ty = data.transform.ty;
                    ros_msg.transform.tz = data.transform.tz;
                    ros_msg.transform.yaw = data.transform.yaw;
                    ros_msg.transform.pitch = data.transform.pitch;
                    ros_msg.transform.roll = data.transform.roll;

                    int m_idx = 0;
                    for (int i = 0; i < 4; i++) {
                        for (int j = 0; j < 4; j++) {
                            ros_msg.transform.matrix[m_idx] = data.transform.matrix[i][j];
                            m_idx++;
                        }
                    }

                    ros_msg.plane.index = data.plane.index;
                    ros_msg.plane.dir[0] = data.plane.dir[0];
                    ros_msg.plane.dir[1] = data.plane.dir[1];
                    ros_msg.plane.dir[2] = data.plane.dir[2];

                    for (const auto& sm : data.plane.markers) {
                        ariemedi_tracker::msg::MarkerPosition m;
                        for (int i = 0; i < 4; i++) m.p[i] = sm.P[i];
                        for (int i = 0; i < 3; i++) m.dir[i] = sm.dir[i];
                        m.left_exposure = sm.leftExposure;
                        m.right_exposure = sm.rightExposure;
                        m.type = sm.type;
                        m.rf = sm.RF;
                        m.biase = sm.biase;
                        ros_msg.plane.markers.push_back(m);
                    }

                    for (const auto& sm : data.points) {
                        ariemedi_tracker::msg::MarkerPosition m;
                        for (int i = 0; i < 4; i++) m.p[i] = sm.P[i];
                        for (int i = 0; i < 3; i++) m.dir[i] = sm.dir[i];
                        m.left_exposure = sm.leftExposure;
                        m.right_exposure = sm.rightExposure;
                        m.type = sm.type;
                        m.rf = sm.RF;
                        m.biase = sm.biase;
                        ros_msg.points.push_back(m);
                    }

                    armd_pub_->publish(ros_msg);

                    // Broadcast TF as well.
                    geometry_msgs::msg::TransformStamped t;
                    t.header = ros_msg.header;
                    t.child_frame_id = sanitizeFrameId(data.name, default_tool_frame_id_);

                    // If ARMD SDK returns mm, divide by 1000 for ROS meters.
                    t.transform.translation.x = data.transform.tx / 1000.0;
                    t.transform.translation.y = data.transform.ty / 1000.0;
                    t.transform.translation.z = data.transform.tz / 1000.0;

                    const double q_norm = std::sqrt(
                        data.transform.qw * data.transform.qw +
                        data.transform.qx * data.transform.qx +
                        data.transform.qy * data.transform.qy +
                        data.transform.qz * data.transform.qz);
                    if (q_norm > 1e-9) {
                        t.transform.rotation.w = data.transform.qw / q_norm;
                        t.transform.rotation.x = data.transform.qx / q_norm;
                        t.transform.rotation.y = data.transform.qy / q_norm;
                        t.transform.rotation.z = data.transform.qz / q_norm;
                    } else {
                        t.transform.rotation.w = 1.0;
                        t.transform.rotation.x = 0.0;
                        t.transform.rotation.y = 0.0;
                        t.transform.rotation.z = 0.0;
                    }

                    tf_broadcaster_->sendTransform(t);
                }
            }
        }

        if (has_valid_tracking_data)
        {
            last_frame_time_ = current_time;
        }
    }

    ARMDCombinedAPI* tracker_;
    bool tracking_started_;
    bool enable_imaging_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<ariemedi_tracker::msg::ToolTrackingData>::SharedPtr armd_pub_;
    std::string base_frame_id_;
    std::string default_tool_frame_id_;
    
    // Frame timing and counting
    uint64_t frame_count_;
    double last_frame_time_;
    double start_time_;
    
    // Data deduplication
    double last_tx_, last_ty_, last_tz_;
    std::string last_timespec_;
    bool has_last_data_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
