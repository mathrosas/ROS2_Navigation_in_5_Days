#include "project_localization/srv/my_service_message.hpp"
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

using MyServiceMessage = project_localization::srv::MyServiceMessage;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;

class SpotRecorder : public rclcpp::Node {
public:
  SpotRecorder() : Node("spot_recorder") {
    // Subscribe to amcl_pose
    pose_sub_ = create_subscription<PoseMsg>(
        "/amcl_pose", 10, [this](PoseMsg::SharedPtr msg) { last_pose_ = msg; });

    // Create service
    srv_ = create_service<MyServiceMessage>(
        "/save_spot", std::bind(&SpotRecorder::handle_save, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Spot recorder service ready.");
  }

private:
  void handle_save(const std::shared_ptr<MyServiceMessage::Request> req,
                   std::shared_ptr<MyServiceMessage::Response> res) {
    const auto &label = req->label;
    // If not "end", store current pose
    if (label != "end") {
      if (!last_pose_) {
        res->navigation_successfull = false;
        res->message = "No pose received yet.";
        return;
      }
      spots_[label] = last_pose_->pose.pose;
      res->navigation_successfull = true;
      res->message = "Saved spot: " + label;
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());

    } else {
      // Write all saved spots to file
      std::ofstream ofs("config/spots.txt");
      if (!ofs) {
        res->navigation_successfull = false;
        res->message = "Failed to open spots.txt for writing.";
        return;
      }
      for (auto &entry : spots_) {
        auto &p = entry.second;
        ofs << entry.first << " " << p.position.x << " " << p.position.y << " "
            << p.position.z << " " << p.orientation.x << " " << p.orientation.y
            << " " << p.orientation.z << " " << p.orientation.w << "\n";
      }
      ofs.close();

      res->navigation_successfull = true;
      res->message =
          "Wrote " + std::to_string(spots_.size()) + " spots to spots.txt";
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    }
  }

  rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub_;
  rclcpp::Service<MyServiceMessage>::SharedPtr srv_;
  PoseMsg::SharedPtr last_pose_;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> spots_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpotRecorder>());
  rclcpp::shutdown();
  return 0;
}