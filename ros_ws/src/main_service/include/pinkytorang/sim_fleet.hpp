// SimFleet: ROS 없이 로봇 3대를 흉내 내는 가짜 Fleet Link (UI · 관제 로직 확인용)
// - 받은 Waypoint 쪽으로 직선으로 움직이며 NODE_REACHED / CMD_DONE / 상태를 Task Manager에 알린다
// - 도킹 노드에서는 1초 기다렸다가 CMD_DONE (정밀 도킹 흉내)
// - 배터리: 움직이면 줄고 도킹에 서 있으면 찬다 (충전 규칙 확인용)
// - 장애물 시험: SIM_BLOCK=<노드>[:<비키는 초>] 면 그 노드로 가다 앞에서 멈춘다 (로봇 실제 동작과 같게)
//   block_wait_sec(10초) 기다려도 안 비키면 BLOCKED -> 서버가 다른 길(REPLACE)을 주면 그 길로.
//   비키는 초를 주면 그만큼 뒤 장애물이 없어져 그대로 이어 감. 로봇마다 한 번만 막힘
#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pinkytorang/config.hpp"
#include "pinkytorang/map_manager.hpp"
#include "pinkytorang/ports.hpp"
#include "pinkytorang/task_manager.hpp"

namespace pinky {

class SimFleet : public IFleetOut {
 public:
  SimFleet(const Config& cfg, const MapManager& map, double speed_mps, double time_scale);
  ~SimFleet();

  void setTaskManager(TaskManager* tasks) { tasks_ = tasks; }
  void start();
  void stop();

  // IFleetOut (Task Manager가 부름)
  void sendMove(const std::string& robot_id, const MoveCmd& cmd) override;
  void sendControl(const std::string& robot_id, ControlCmd cmd) override;
  void sendDisplay(const std::string& robot_id, const DisplayCmd& cmd) override;

 private:
  struct PathPoint {
    WaypointCmd w;
    bool reverse = false;  // 이 노드까지 후진 (받은 Move의 drive)
  };

  struct Bot {
    std::string id;
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool guide = false;  // 지금 명령이 안내 (Move.guide)
    std::string last_node;
    uint32_t cmd_id = 0;
    bool running = false;            // 명령 수행 중
    std::vector<PathPoint> path;     // 남은 Waypoint (+ 그 칸을 후진으로 가는지)
    bool paused = false;
    double dock_wait = 0.0;          // 남은 정밀 도킹 시간 (초)
    double loc_wait = 0.0;           // 남은 위치 맞추기 시간 (초, Control LOCALIZE)
    double battery = 90.0;           // %
    double block_t = -1.0;           // 장애물 앞에서 멈춘 지 몇 초 (-1 = 안 막힘)
    bool block_reported = false;     // BLOCKED 보냄
    bool block_done = false;         // 이미 한 번 막혔음 (SIM_BLOCK은 로봇마다 한 번)
  };

  // 바깥(Task Manager)에 알릴 이벤트. 자물쇠를 푼 뒤에 한꺼번에 보낸다
  struct PendingEvent {
    std::string robot_id;
    uint8_t type;
    uint32_t cmd_id;
    std::string node;
    std::string reason;
  };

  void loop();
  void step(Bot& b, double dt, std::vector<PendingEvent>& out);

  const Config& cfg_;
  const MapManager& map_;
  double speed_;
  double scale_;
  TaskManager* tasks_ = nullptr;

  std::mutex mutex_;
  std::map<std::string, Bot> bots_;
  std::vector<PendingEvent> pending_;  // sendMove에서 바로 생긴 이벤트 (다음 loop에서 보냄)
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace pinky
