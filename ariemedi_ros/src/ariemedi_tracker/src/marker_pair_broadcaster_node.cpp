#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ariemedi_tracker/msg/tool_tracking_data.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

class MarkerPairBroadcasterNode : public rclcpp::Node
{
public:
    MarkerPairBroadcasterNode()
    : Node("marker_pair_broadcaster_node")
    {
        this->declare_parameter<std::string>("input_topic", "/ARMDpos");
        this->declare_parameter<std::string>("tracker_frame", "tracker_base");
        this->declare_parameter<bool>("use_msg_frame_id", true);
        this->declare_parameter<std::vector<std::string>>(
            "tracked_marker_names",
            std::vector<std::string>{"caliorange", "caliblack2"});
        this->declare_parameter<std::string>("child_frame_prefix", "marker_");

        input_topic_ = this->get_parameter("input_topic").as_string();
        tracker_frame_ = this->get_parameter("tracker_frame").as_string();
        use_msg_frame_id_ = this->get_parameter("use_msg_frame_id").as_bool();
        child_frame_prefix_ = this->get_parameter("child_frame_prefix").as_string();
        const auto marker_names = this->get_parameter("tracked_marker_names").as_string_array();

        for (const auto &name : marker_names) {
            tracked_names_.insert(name);
            child_frame_map_[name] = child_frame_prefix_ + name;
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<ariemedi_tracker::msg::ToolTrackingData>(
            input_topic_,
            100,
            std::bind(&MarkerPairBroadcasterNode::on_tool_msg, this, std::placeholders::_1));

        std::string tracked_summary;
        for (const auto &n : marker_names) {
            if (!tracked_summary.empty()) {
                tracked_summary += ", ";
            }
            tracked_summary += n + "->" + child_frame_map_[n];
        }
        RCLCPP_INFO(this->get_logger(),
                    "TF broadcaster started: topic=%s, parent=%s, use_msg_frame_id=%s, tracked=[%s]",
                    input_topic_.c_str(),
                    tracker_frame_.c_str(),
                    use_msg_frame_id_ ? "true" : "false",
                    tracked_summary.c_str());
    }

private:
    void on_tool_msg(const ariemedi_tracker::msg::ToolTrackingData::SharedPtr msg)
    {
        if (!msg->match_status) {
            return;
        }
        if (tracked_names_.find(msg->name) == tracked_names_.end()) {
            return;
        }

        const auto frame_it = child_frame_map_.find(msg->name);
        if (frame_it == child_frame_map_.end()) {
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id =
            (use_msg_frame_id_ && !msg->header.frame_id.empty()) ? msg->header.frame_id : tracker_frame_;
        tf_msg.child_frame_id = frame_it->second;
        tf_msg.transform.translation.x = msg->transform.tx / 1000.0;
        tf_msg.transform.translation.y = msg->transform.ty / 1000.0;
        tf_msg.transform.translation.z = msg->transform.tz / 1000.0;
        tf_msg.transform.rotation.x = msg->transform.qx;
        tf_msg.transform.rotation.y = msg->transform.qy;
        tf_msg.transform.rotation.z = msg->transform.qz;
        tf_msg.transform.rotation.w = msg->transform.qw;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::string input_topic_;
    std::string tracker_frame_;
    bool use_msg_frame_id_;
    std::string child_frame_prefix_;
    std::unordered_set<std::string> tracked_names_;
    std::unordered_map<std::string, std::string> child_frame_map_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<ariemedi_tracker::msg::ToolTrackingData>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerPairBroadcasterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
