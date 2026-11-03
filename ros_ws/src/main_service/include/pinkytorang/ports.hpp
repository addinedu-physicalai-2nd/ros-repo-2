// Task Manager가 바깥으로 내보내는 출구 (인터페이스)
// - IFleetOut : FleetLinkManager(ROS) 또는 SimFleet(가상 로봇)이 구현
// - IClientOut: ClientLinkManager(HTTP / WebSocket)가 구현
#pragma once

#include <string>

#include "pinkytorang/types.hpp"

namespace pinky {

class IFleetOut {
 public:
  virtual ~IFleetOut() {}
  virtual void sendMove(const std::string& robot_id, const MoveCmd& cmd) = 0;
  virtual void sendControl(const std::string& robot_id, ControlCmd cmd) = 0;
  virtual void sendDisplay(const std::string& robot_id, const DisplayCmd& cmd) = 0;
};

class IClientOut {
 public:
  virtual ~IClientOut() {}
  virtual void pushSnapshot(const SnapshotView& snap) = 0;          // 매장 UI
  virtual void pushTaskUpdate(const TaskView& task) = 0;            // 그 테이블 고객 UI
  virtual void pushConfirmNeeded(const ConfirmView& c) = 0;         // 확인 버튼 띄우기
  virtual void pushConfirmClosed(const ConfirmView& c, const std::string& by) = 0;  // by = BUTTON / TIMEOUT
  virtual void pushAlert(const std::string& level, const std::string& robot_id, const std::string& message) = 0;
  // 로봇 카메라(YOLO)가 앞 장애물로 멈춰 세운 순간 (+ 막힘 보고 때): 관제 화면 팝업 (사진은 GET /api/robots/<id>/obstacle.jpg)
  virtual void pushObstacleSnap(const std::string& robot_id, const std::string& node, const std::string& kind,
                                const std::string& label, long long stamp_ms) = 0;
};

}  // namespace pinky
