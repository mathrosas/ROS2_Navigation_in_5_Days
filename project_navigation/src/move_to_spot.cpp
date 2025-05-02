// src/move_to_spot.cpp

#include <algorithm>
#include <cctype>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <string>
#include <unordered_map>

struct PoseYAML {
  double px = 0, py = 0, pz = 0;
  double ox = 0, oy = 0, oz = 0, ow = 1;
};

// trim whitespace from both ends
static std::string trim(const std::string &s) {
  auto l = s.find_first_not_of(" \t\r\n");
  if (l == std::string::npos)
    return "";
  auto r = s.find_last_not_of(" \t\r\n");
  return s.substr(l, r - l + 1);
}

// load spot_file into a map<label,PoseYAML>
static std::unordered_map<std::string, PoseYAML>
loadSpots(const std::string &file) {
  std::unordered_map<std::string, PoseYAML> m;
  std::ifstream ifs(file);
  if (!ifs)
    return m;

  std::string line, label, block;
  PoseYAML cur;
  while (std::getline(ifs, line)) {
    auto t = trim(line);
    if (t.empty())
      continue;
    int indent = (int)line.find_first_not_of(' ');
    if (indent == 0 && t.back() == ':') {
      if (!label.empty())
        m[label] = cur;
      label = t.substr(0, t.size() - 1);
      cur = PoseYAML{};
    } else if (indent == 2) {
      block = (t == "position:") ? "pos" : (t == "orientation:") ? "ori" : "";
    } else if (indent == 4 && !label.empty() && !block.empty()) {
      auto p = t.find(':');
      if (p == std::string::npos)
        continue;
      auto key = trim(t.substr(0, p));
      double v = std::stod(trim(t.substr(p + 1)));
      if (block == "pos") {
        if (key == "x")
          cur.px = v;
        else if (key == "y")
          cur.py = v;
        else if (key == "z")
          cur.pz = v;
      } else { // ori
        if (key == "x")
          cur.ox = v;
        else if (key == "y")
          cur.oy = v;
        else if (key == "z")
          cur.oz = v;
        else if (key == "w")
          cur.ow = v;
      }
    }
  }
  if (!label.empty())
    m[label] = cur;
  return m;
}

class MoveToSpot : public rclcpp::Node {
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  MoveToSpot() : Node("move_to_spot") {
    declare_parameter<std::string>("spot_name", "");
    declare_parameter<std::string>("spot_file", "src/spot-list.yaml");
    spot_name_ = get_parameter("spot_name").as_string();
    spot_file_ = get_parameter("spot_file").as_string();

    if (spot_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'spot_name' must be set");
      rclcpp::shutdown();
      return;
    }

    auto spots = loadSpots(spot_file_);
    if (!spots.count(spot_name_)) {
      RCLCPP_ERROR(get_logger(), "Spot '%s' not found in '%s'",
                   spot_name_.c_str(), spot_file_.c_str());
      rclcpp::shutdown();
      return;
    }

    // === BUILD THE GOAL ===
    Navigate::Goal goal_msg;
    auto &s = spots[spot_name_];
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = s.px;
    goal_msg.pose.pose.position.y = s.py;
    goal_msg.pose.pose.position.z = s.pz;
    goal_msg.pose.pose.orientation.x = s.ox;
    goal_msg.pose.pose.orientation.y = s.oy;
    goal_msg.pose.pose.orientation.z = s.oz;
    goal_msg.pose.pose.orientation.w = s.ow;

    // === CREATE AND WAIT FOR THE ACTION SERVER ===
    client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Sending robot to '%s' → (%.2f, %.2f)",
                spot_name_.c_str(), s.px, s.py);

    // === SEND GOAL ===
    auto options = typename rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.result_callback = [this](const GoalHandle::WrappedResult &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Reached '%s'!", spot_name_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Failed to reach '%s' (code %d)",
                    spot_name_.c_str(), int(result.code));
      }
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal_msg, options);
  }

private:
  std::string spot_name_, spot_file_;
  rclcpp_action::Client<Navigate>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToSpot>());
  rclcpp::shutdown();
  return 0;
}
