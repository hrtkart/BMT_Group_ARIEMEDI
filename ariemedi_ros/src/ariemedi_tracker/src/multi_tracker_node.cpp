#include <rclcpp/rclcpp.hpp>

#include "ariemedi_tracker/msg/tool_tracking_data.hpp"
#include "ARMDCombinedAPI.h"
#include "DeviceScan.h"

#include <chrono>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

class MultiTrackerNode : public rclcpp::Node
{
public:
    MultiTrackerNode()
    : Node("ariemedi_multi_tracker"),
      tracker_(new ARMDCombinedAPI()),
      tracking_started_(false),
      enable_imaging_(false),
      init_ok_(false)
    {
        this->declare_parameter<std::string>("hostname", "");
        this->declare_parameter<bool>("enable_imaging", false);
        this->declare_parameter<std::string>("output_topic", "/ARMDpos");
        this->declare_parameter<std::vector<std::string>>(
            "tool_paths",
            std::vector<std::string>{
                "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliblack2.arom",
                "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliorange.arom"});

        enable_imaging_ = this->get_parameter("enable_imaging").as_bool();
        output_topic_ = this->get_parameter("output_topic").as_string();
        tool_paths_ = this->get_parameter("tool_paths").as_string_array();
        armd_pub_ =
            this->create_publisher<ariemedi_tracker::msg::ToolTrackingData>(output_topic_, 10);

        initDevice();
    }

    ~MultiTrackerNode()
    {
        stopTracking();
        delete tracker_;
    }

    bool is_initialized() const
    {
        return init_ok_;
    }

private:
    struct ToolStats
    {
        uint64_t frame_count{0};
        double start_time{0.0};
        double last_frame_time{0.0};
        double last_frame_dt{0.0};
        double last_tx{0.0};
        double last_ty{0.0};
        double last_tz{0.0};
        std::string last_timespec;
        bool initialized{false};
        bool has_last_pose{false};
    };

    bool parseDeviceTimespec(const std::string &timespec, rclcpp::Time &stamp_out) const
    {
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int minute = 0;
        int second = 0;
        int millisecond = 0;
        const int parsed =
            std::sscanf(timespec.c_str(), "%d-%d-%d %d:%d:%d:%d",
                        &year, &month, &day, &hour, &minute, &second, &millisecond);
        if (parsed != 7) {
            return false;
        }
        if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 ||
            minute < 0 || minute > 59 || second < 0 || second > 60 ||
            millisecond < 0 || millisecond > 999) {
            return false;
        }

        std::tm tm_time{};
        tm_time.tm_year = year - 1900;
        tm_time.tm_mon = month - 1;
        tm_time.tm_mday = day;
        tm_time.tm_hour = hour;
        tm_time.tm_min = minute;
        tm_time.tm_sec = second;
        tm_time.tm_isdst = -1;

        const std::time_t wall_time = std::mktime(&tm_time);
        if (wall_time < 0) {
            return false;
        }
        const int64_t nanoseconds =
            static_cast<int64_t>(wall_time) * 1000000000LL +
            static_cast<int64_t>(millisecond) * 1000000LL;
        stamp_out = rclcpp::Time(nanoseconds, RCL_SYSTEM_TIME);
        return true;
    }

    void initDevice()
    {
        std::string hostname = this->get_parameter("hostname").as_string();

        if (hostname.empty()) {
            RCLCPP_INFO(this->get_logger(), "Scanning for RT devices...");
            DeviceScan device_scan;

            int scan_attempts = 10;
            while (device_scan.getDeviceInfo().empty() && scan_attempts > 0) {
                device_scan.updateDeviceInfo();
                std::this_thread::sleep_for(500ms);
                --scan_attempts;
            }

            if (device_scan.getDeviceInfo().empty()) {
                RCLCPP_ERROR(this->get_logger(), "No RT devices found during scan.");
                return;
            }

            for (const auto &x : device_scan.getDeviceInfo()) {
                hostname = x.first;
                RCLCPP_INFO(
                    this->get_logger(),
                    "Scanned RT device hostname: %s, IP: %s",
                    hostname.c_str(),
                    x.second.at(0).c_str());
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Connecting to device: %s...", hostname.c_str());
        const int error_code = tracker_->connect(hostname, true, true, true);
        if (error_code != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect, error code: %d", error_code);
            return;
        }

        std::vector<std::string> valid_tool_paths;
        valid_tool_paths.reserve(tool_paths_.size());
        for (const auto &path : tool_paths_) {
            if (std::filesystem::exists(path)) {
                valid_tool_paths.push_back(path);
                RCLCPP_INFO(this->get_logger(), "Tool loaded: %s", path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Tool path not found: %s", path.c_str());
            }
        }

        if (valid_tool_paths.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid tool paths were provided.");
            return;
        }

        tracker_->loadPassiveToolAROM(valid_tool_paths);
        tracker_->setTrackingDataTransmissionType(TransmissionType::Passive);
        tracker_->startTracking();

        if (enable_imaging_) {
            tracker_->startImaging();
            RCLCPP_INFO(this->get_logger(), "Imaging enabled.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Imaging disabled for max tracking throughput.");
        }

        tracking_started_ = true;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 120.0),
            std::bind(&MultiTrackerNode::step, this));

        init_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "Multi-tool tracker started at 120Hz.");
    }

    void stopTracking()
    {
        tracking_started_ = false;
        if (timer_) {
            timer_->cancel();
        }

        if (!tracker_) {
            return;
        }

        if (ConnectionStatus::Interruption == tracker_->getConnectionStatus()) {
            tracker_->disconnect();
        } else {
            tracker_->stopTracking();
            if (enable_imaging_) {
                tracker_->stopImaging();
            }
            tracker_->disconnect();
        }
        RCLCPP_INFO(this->get_logger(), "Disconnected device and stopped tracking.");
    }

    void step()
    {
        if (!tracking_started_) {
            return;
        }

        if (ConnectionStatus::Interruption == tracker_->getConnectionStatus()) {
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
        std::vector<MarkerPosition> all_marker_data = tracker_->getAllMarkers();
        std::vector<ToolTrackingData> tool_data = tracker_->getTrackingData(all_marker_data);
        const double current_time = this->get_clock()->now().seconds();

        if (tool_data.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No tracked tools.");
        }

        for (const auto &data : tool_data) {
            if (!data.matchStatus) {
                continue;
            }

            auto &stats = tool_stats_[data.name];
            if (!stats.initialized) {
                stats.start_time = current_time;
                stats.initialized = true;
            }

            if (!data.timespec.empty() && data.timespec == stats.last_timespec) {
                continue;
            }
            if (data.timespec.empty() &&
                stats.has_last_pose &&
                std::abs(data.transform.tx - stats.last_tx) < 0.01 &&
                std::abs(data.transform.ty - stats.last_ty) < 0.01 &&
                std::abs(data.transform.tz - stats.last_tz) < 0.01 &&
                (current_time - stats.last_frame_time) < 0.005) {
                continue;
            }

            stats.frame_count++;
            stats.last_frame_dt = (stats.last_frame_time > 0.0) ? (current_time - stats.last_frame_time) : 0.0;
            stats.last_frame_time = current_time;
            stats.last_tx = data.transform.tx;
            stats.last_ty = data.transform.ty;
            stats.last_tz = data.transform.tz;
            stats.last_timespec = data.timespec;
            stats.has_last_pose = true;

            auto ros_msg = ariemedi_tracker::msg::ToolTrackingData();
            rclcpp::Time stamp = this->get_clock()->now();
            parseDeviceTimespec(data.timespec, stamp);
            ros_msg.header.stamp = stamp;
            ros_msg.header.frame_id = "tracker_base";
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
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    ros_msg.transform.matrix[m_idx] = data.transform.matrix[i][j];
                    ++m_idx;
                }
            }

            ros_msg.plane.index = data.plane.index;
            ros_msg.plane.dir[0] = data.plane.dir[0];
            ros_msg.plane.dir[1] = data.plane.dir[1];
            ros_msg.plane.dir[2] = data.plane.dir[2];

            for (const auto &sm : data.plane.markers) {
                ariemedi_tracker::msg::MarkerPosition marker_msg;
                for (int i = 0; i < 4; ++i) {
                    marker_msg.p[i] = sm.P[i];
                }
                for (int i = 0; i < 3; ++i) {
                    marker_msg.dir[i] = sm.dir[i];
                }
                marker_msg.left_exposure = sm.leftExposure;
                marker_msg.right_exposure = sm.rightExposure;
                marker_msg.type = sm.type;
                marker_msg.rf = sm.RF;
                marker_msg.biase = sm.biase;
                ros_msg.plane.markers.push_back(marker_msg);
            }

            for (const auto &sm : data.points) {
                ariemedi_tracker::msg::MarkerPosition marker_msg;
                for (int i = 0; i < 4; ++i) {
                    marker_msg.p[i] = sm.P[i];
                }
                for (int i = 0; i < 3; ++i) {
                    marker_msg.dir[i] = sm.dir[i];
                }
                marker_msg.left_exposure = sm.leftExposure;
                marker_msg.right_exposure = sm.rightExposure;
                marker_msg.type = sm.type;
                marker_msg.rf = sm.RF;
                marker_msg.biase = sm.biase;
                ros_msg.points.push_back(marker_msg);
            }

            armd_pub_->publish(ros_msg);
        }
    }

    ARMDCombinedAPI *tracker_;
    bool tracking_started_;
    bool enable_imaging_;
    bool init_ok_;
    std::string output_topic_;
    std::vector<std::string> tool_paths_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ariemedi_tracker::msg::ToolTrackingData>::SharedPtr armd_pub_;
    std::unordered_map<std::string, ToolStats> tool_stats_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiTrackerNode>();
    if (!node->is_initialized()) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
