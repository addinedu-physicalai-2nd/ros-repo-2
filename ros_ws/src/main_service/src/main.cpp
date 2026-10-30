// 핑키토랑 Main Service (ROS 2 Jazzy, 서버 도메인)
// 실행: ros2 run pinkytorang_main main_service <설정 파일 경로> [--localize pinky1=true pinky2=true pinky3=false]
//   --localize: 시작 전 위치 맞추기 할 로봇 (true만). 설정의 localize_robots를 덮어씀. 보통 launch/system.launch.xml이 넘김
//
// 스레드
//   main     : 0.1초마다 tasks.tick()
//   ROS spin : 로봇 메시지가 오면 Task Manager 함수 호출
//   Crow     : HTTP 요청이 오면 Task Manager 함수 호출
//   Task Manager 안의 mutex 하나가 이 셋이 동시에 들어오는 것을 막는다

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "pinkytorang/client_link_manager.hpp"
#include "pinkytorang/config.hpp"
#include "pinkytorang/db_manager.hpp"
#include "pinkytorang/dispatch_manager.hpp"
#include "pinkytorang/fleet_link_manager.hpp"
#include "pinkytorang/log.hpp"
#include "pinkytorang/map_manager.hpp"
#include "pinkytorang/task_manager.hpp"
#include "pinkytorang/traffic_manager.hpp"

using namespace pinky;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    std::cerr << "사용법: ros2 run pinkytorang_main main_service <main_service.yaml 경로>" << std::endl;
    return 1;
  }

  // 1. 설정 · 맵 · DB
  Config cfg;
  std::string error;
  if (!loadConfig(argv[1], cfg, error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  // --localize pinky1=true pinky2=false ... (다음 '--' 옵션 전까지)
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) != "--localize") continue;
    cfg.localize_robots.clear();
    bool enable = true;   // enable=false 면 위치 맞추기 안 함 (launch localize:=false)
    for (int j = i + 1; j < argc && std::string(argv[j]).rfind("--", 0) != 0; ++j) {
      std::string a = argv[j];
      size_t eq = a.find('=');
      if (eq == std::string::npos) continue;
      std::string id = a.substr(0, eq), v = a.substr(eq + 1);
      bool on = (v == "true" || v == "True" || v == "1");
      if (id == "enable") enable = on;
      else if (on) cfg.localize_robots.push_back(id);
    }
    if (!enable) cfg.localize_robots.clear();
  }
  MapManager map;
  if (!map.load(cfg.node_graph, error)) {
    std::cerr << error << std::endl;
    return 1;
  }
  DBManager db;
  if (!db.open(cfg.db_path, error)) {
    std::cerr << error << std::endl;
    return 1;
  }

  // 2. 판단 모듈
  TrafficManager traffic(cfg, map);
  for (const std::string& id : cfg.robots) traffic.addRobot(id);
  DispatchManager dispatch(cfg, map, traffic);
  TaskManager tasks(cfg, map, traffic, dispatch, db);

  // 3. 입구 · 출구 연결
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pinkytorang_main_service");
  FleetLinkManager fleet(node, cfg);
  ClientLinkManager client(cfg, tasks);
  fleet.setTaskManager(&tasks);
  traffic.setFleetOut(&fleet);
  tasks.setFleetOut(&fleet);
  tasks.setClientOut(&client);

  // 4. 시작
  tasks.start();
  client.start();
  std::thread spin_thread([node]() { rclcpp::spin(node); });  // 로봇 메시지 받기
  logLine("MAIN", "실행 중 (Ctrl+C 종료)");

  // 5. 0.1초마다 주기 처리. Ctrl+C를 누르면 rclcpp::ok()가 false가 된다
  while (rclcpp::ok()) {
    tasks.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  spin_thread.join();
  client.stop();
  return 0;
}
