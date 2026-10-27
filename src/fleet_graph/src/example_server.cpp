// 서버 쪽 C++ 예시 노드: 경로를 계산해 Move / Display를 보내고, 로봇의 event / state를 받는다.
//
//   ros2 run fleet_graph example_server --ros-args -p robot_id:=pinky1 -p from:=N13 -p to:=D3
//
// 메인 서비스 코드에 그대로 옮겨 쓸 수 있는 부분: 토픽 이름, QoS, 메시지 채우는 법, 이벤트 받는 법.
// 주의: 예시라서 경로 전체를 한 번에 보낸다. 실제 서버는 lock을 잡은 노드까지만 보내고
//       NODE_REACHED를 받을 때마다 APPEND한다 (fleet_msgs README 4.1).
#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fleet_graph/graph.hpp"
#include "fleet_msgs/msg/display.hpp"
#include "fleet_msgs/msg/move.hpp"
#include "fleet_msgs/msg/robot_event.hpp"
#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/waypoint.hpp"

using namespace std::chrono_literals;
using fleet_msgs::msg::Display;
using fleet_msgs::msg::Move;
using fleet_msgs::msg::RobotEvent;
using fleet_msgs::msg::RobotState;

class ExampleServer : public rclcpp::Node
{
public:
  ExampleServer()
  : rclcpp::Node("fleet_example_server")
  {
    robot_id_ = declare_parameter<std::string>("robot_id", "pinky1");
    from_ = declare_parameter<std::string>("from", "N13");
    to_ = declare_parameter<std::string>("to", "D3");
    std::string graph_file = declare_parameter<std::string>("graph_file", "");
    if (graph_file.empty()) {
      graph_file = ament_index_cpp::get_package_share_directory("fleet_msgs") + "/docs/node_graph.yaml";
    }
    graph_ = fleet_graph::Graph::load(graph_file);
    for (const auto & p : graph_.validate()) {
      RCLCPP_WARN(get_logger(), "노드 그래프 문제: %s", p.c_str());
    }

    // 서버 쪽 토픽 = /<robot_id>/fleet/... (README 2.2), QoS (README 2.3)
    const std::string ns = "/" + robot_id_ + "/fleet/";
    move_pub_ = create_publisher<Move>(ns + "move", rclcpp::QoS(10).reliable());
    display_pub_ = create_publisher<Display>(ns + "display", rclcpp::QoS(1).reliable().transient_local());
    event_sub_ = create_subscription<RobotEvent>(
      ns + "event", rclcpp::QoS(10).reliable(),
      [this](const RobotEvent & e) {on_event(e);});
    state_sub_ = create_subscription<RobotState>(
      ns + "state", rclcpp::QoS(1).best_effort(),
      [this](const RobotState & s) {last_state_ = s;});

    // 구독자가 연결될 시간을 조금 준 뒤 한 번 보낸다
    timer_ = create_wall_timer(2s, [this]() {timer_->cancel(); send_route();});
  }

private:
  void send_route()
  {
    const auto path = graph_.shortest_path(from_, to_);
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "%s -> %s 경로 없음", from_.c_str(), to_.c_str());
      return;
    }
    const uint32_t cmd_id = ++cmd_counter_;
    bool first = true;
    for (const auto & seg : graph_.to_segments(path)) {
      Move m;
      m.cmd_id = cmd_id;
      m.mode = first ? Move::REPLACE : Move::APPEND;   // 같은 명령이면 같은 cmd_id로 이어 붙임
      m.drive = static_cast<uint8_t>(seg.drive);       // Move::FORWARD(0) / Move::REVERSE(1)
      for (const auto & w : seg.waypoints) {
        fleet_msgs::msg::Waypoint wp;
        wp.node_id = w.node_id;
        wp.x = w.x;
        wp.y = w.y;
        wp.yaw = w.yaw;
        m.waypoints.push_back(wp);
      }
      move_pub_->publish(m);
      RCLCPP_INFO(
        get_logger(), "Move cmd=%u %s %s, %zu개 (마지막 %s)", cmd_id,
        first ? "REPLACE" : "APPEND", fleet_graph::to_string(seg.drive),
        m.waypoints.size(), m.waypoints.back().node_id.c_str());
      first = false;
    }

    Display d;
    d.text = to_ + " 이동 중";
    d.countdown_sec = 0;
    display_pub_->publish(d);
  }

  void on_event(const RobotEvent & e)
  {
    const char * type = "?";
    switch (e.type) {
      case RobotEvent::NODE_REACHED: type = "NODE_REACHED"; break;
      case RobotEvent::CMD_DONE: type = "CMD_DONE"; break;
      case RobotEvent::CMD_FAILED: type = "CMD_FAILED"; break;
      case RobotEvent::BLOCKED: type = "BLOCKED"; break;
      default: break;
    }
    RCLCPP_INFO(
      get_logger(), "[%s] %s cmd=%u node=%s %s", e.robot_id.c_str(), type, e.cmd_id,
      e.node_id.c_str(), e.reason.c_str());
  }

  std::string robot_id_, from_, to_;
  fleet_graph::Graph graph_;
  uint32_t cmd_counter_ = 0;
  RobotState last_state_;
  rclcpp::Publisher<Move>::SharedPtr move_pub_;
  rclcpp::Publisher<Display>::SharedPtr display_pub_;
  rclcpp::Subscription<RobotEvent>::SharedPtr event_sub_;
  rclcpp::Subscription<RobotState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleServer>());
  rclcpp::shutdown();
  return 0;
}
