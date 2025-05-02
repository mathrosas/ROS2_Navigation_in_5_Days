// src/spot_recorder.cpp

#include <algorithm>
#include <cctype>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <unordered_map>

struct PoseYAML {
  double px = 0, py = 0, pz = 0;
  double ox = 0, oy = 0, oz = 0, ow = 1;
};

// trim whitespace
static std::string trim(const std::string &s) {
  auto l = s.find_first_not_of(" \t\r\n");
  if (l == std::string::npos)
    return "";
  auto r = s.find_last_not_of(" \t\r\n");
  return s.substr(l, r - l + 1);
}

// load existing YAML into map<label,PoseYAML>
static std::unordered_map<std::string, PoseYAML>
loadSpots(const std::string &file) {
  std::unordered_map<std::string, PoseYAML> m;
  std::ifstream ifs(file);
  if (!ifs)
    return m;

  std::string line, label, block;
  PoseYAML current;
  while (std::getline(ifs, line)) {
    auto t = trim(line);
    if (t.empty())
      continue;
    int indent = (int)line.find_first_not_of(' ');
    if (indent == 0 && t.back() == ':') {
      if (!label.empty())
        m[label] = current;
      label = t.substr(0, t.size() - 1);
      current = PoseYAML{};
    } else if (indent == 2) {
      if (t == "position:")
        block = "pos";
      else if (t == "orientation:")
        block = "ori";
    } else if (indent == 4 && !label.empty()) {
      auto p = t.find(':');
      if (p == std::string::npos)
        continue;
      auto key = trim(t.substr(0, p));
      double v = std::stod(trim(t.substr(p + 1)));
      if (block == "pos") {
        if (key == "x")
          current.px = v;
        else if (key == "y")
          current.py = v;
        else if (key == "z")
          current.pz = v;
      } else if (block == "ori") {
        if (key == "x")
          current.ox = v;
        else if (key == "y")
          current.oy = v;
        else if (key == "z")
          current.oz = v;
        else if (key == "w")
          current.ow = v;
      }
    }
  }
  if (!label.empty())
    m[label] = current;
  return m;
}

// write the full map back to YAML
static void writeSpots(const std::string &file,
                       const std::unordered_map<std::string, PoseYAML> &m) {
  std::ofstream ofs(file);
  for (auto &e : m) {
    ofs << e.first << ":\n";
    ofs << "  position:\n";
    ofs << "    x: " << e.second.px << "\n";
    ofs << "    y: " << e.second.py << "\n";
    ofs << "    z: " << e.second.pz << "\n";
    ofs << "  orientation:\n";
    ofs << "    x: " << e.second.ox << "\n";
    ofs << "    y: " << e.second.oy << "\n";
    ofs << "    z: " << e.second.oz << "\n";
    ofs << "    w: " << e.second.ow << "\n\n";
  }
}

class SpotRecorder : public rclcpp::Node {
public:
  SpotRecorder() : Node("spot_recorder") {
    declare_parameter<std::string>("spot_name", "");
    declare_parameter<std::string>("spot_file", "src/spot-list.yaml");
    spot_name_ = get_parameter("spot_name").as_string();
    spot_file_ = get_parameter("spot_file").as_string();

    if (spot_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "Must set --ros-args -p spot_name:=<label>");
      rclcpp::shutdown();
      return;
    }

    sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&SpotRecorder::on_pose_, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Waiting for /initialpose → recording '%s' in '%s'",
                spot_name_.c_str(), spot_file_.c_str());
  }

private:
  void on_pose_(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (done_)
      return;
    done_ = true;

    auto spots = loadSpots(spot_file_);
    PoseYAML p;
    p.px = msg->pose.pose.position.x;
    p.py = msg->pose.pose.position.y;
    p.pz = msg->pose.pose.position.z;
    p.ox = msg->pose.pose.orientation.x;
    p.oy = msg->pose.pose.orientation.y;
    p.oz = msg->pose.pose.orientation.z;
    p.ow = msg->pose.pose.orientation.w;
    spots[spot_name_] = p;

    writeSpots(spot_file_, spots);

    RCLCPP_INFO(get_logger(), "Recorded '%s': (%.2f, %.2f, %.2f)",
                spot_name_.c_str(), p.px, p.py, p.pz);

    rclcpp::shutdown();
  }

  std::string spot_name_, spot_file_;
  bool done_{false};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpotRecorder>());
  rclcpp::shutdown();
  return 0;
}
