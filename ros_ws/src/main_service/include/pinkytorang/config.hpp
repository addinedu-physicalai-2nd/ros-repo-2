// 설정 파일 (config/main_service.yaml)
#pragma once

#include <map>
#include <string>
#include <vector>

#include "pinkytorang/types.hpp"

namespace pinky {

// LCD 문구. {table} 은 테이블 이름으로 바뀐다
struct DisplayTexts {
  std::string to_pickup = "";
  std::string at_pickup = "음식을 올려 주세요";
  std::string to_table_serve = "{table} 배달 중";
  std::string at_table_serve = "음식이 도착했습니다. 가져가 주세요";
  std::string to_table_guide = "{table}(으)로 안내할게요. 따라오세요";
  std::string at_table_guide = "{table}입니다. 자리에 앉아 주세요";
  std::string to_dock = "";
  std::string idle = "";
};

struct TableConfig {
  std::string id;   // 노드 그래프의 table 값과 같아야 함
  double x = 0.0;   // 키오스크 배치도에 그릴 테이블 중심 (m, 맵 좌표)
  double y = 0.0;
  double w = 0.165; // 테이블 크기 (m)
  double h = 0.110;
};

struct Config {
  int http_port = 8080;
  std::string web_root = "../web";
  std::string db_path = "pinkytorang.db";
  std::string node_graph = "config/node_graph.yaml";
  double board_x = 0.0;   // 판(UI 배치도) 왼쪽 아래의 map 좌표 (m). SLAM 맵 yaml의 origin
  double board_y = 0.0;
  double board_w = 2.28;  // 판 크기 (m)
  double board_h = 1.82;
  bool board_flip = false; // true = 배치도를 180도 돌려 그림 (맵이 매장 그림과 거꾸로 만들어졌을 때)
  std::vector<std::string> robots = {"pinky1", "pinky2", "pinky3"};

  int table_timeout_sec = 30;    // 서빙: 손님 수령 카운트다운
  int guide_timeout_sec = 15;    // 안내: 착석 확인 카운트다운
  int pickup_timeout_sec = 0;    // 픽업: 0 = 주방 적재 버튼으로만, 0보다 크면 이만큼 기다렸다 자동 출발 (시험용)
  bool auto_ready = false;       // true = 주문하면 조리 완료로 바로 (주방 버튼 없이, 시험용)
  int depart_grace_sec = 2;      // 카운트다운 0 이후 실제 출발까지
  int max_active_serve = 2;      // 빈 테이블이 있을 때 동시에 서빙하는 로봇 최대 수 (만석이면 로봇 수 전부)
  int max_active_guide = 1;      // 동시에 안내하는 로봇 최대 수
  std::map<std::string, std::string> guide_spots;  // 안내: 테이블 -> 서는 노드 (그 노드에서 테이블 쪽을 봄). 없으면 테이블 정차점
  bool table_pass_through = false;  // 테이블은 지나가며 서고 반대 노드로 나감 (들어갈 때 나갈 칸까지 같이 잡음)
  int serve_batch_max = 3;       // 같은 테이블 주문을 한 번에 싣는 최대 건수
  int reassign_after_sec = 10;   // 적재 전 로봇이 이만큼 고장 · 통신 끊김이면 다른 로봇에 다시 배정

  // 도킹 구역
  std::vector<std::string> entrance_docks = {"D1"};        // 입구 쪽 (앞칸부터). 안내는 여기 서 있는 로봇만
  std::vector<std::string> pickup_docks = {"D2", "D3"};    // 픽업 쪽
  std::map<std::string, std::string> dock_via;              // 뒤칸 -> 앞칸 (예전 맵 D1: D2 = D1은 D2를 지나 드나듦). 최종 맵은 없음
  double entrance_keep_penalty_m = 1.5; // 서빙 배정 때 입구 로봇에 더하는 비용 (입구 로봇을 아끼려고)

  // 대기 배치: 찬 테이블(사용 중 + 안내 중) 수에 따라 입구에 세워 둘 로봇 수. 만석이면 0
  int busy_tables = 4;           // 이 수 이상 차면 바쁨
  int entrance_standby_quiet = 2;
  int entrance_standby_busy = 1;

  // 배터리 (%). min_battery_pct = 0 이면 검사 안 함
  double min_battery_pct = 30.0;   // 이보다 낮으면 작업 배정 안 하고 충전
  double ready_battery_pct = 80.0; // 충전 중이면 이 이상이 돼야 다시 배정

  bool require_dock_start = true;  // 처음 위치는 도킹 노드에서만 인정
  // 시작 전 위치 맞추기 (Control LOCALIZE): 이 로봇들에게 이름 순서대로 한 대씩. 비우면 안 함
  //   system.launch.xml의 pinky1:=true ... 가 main_service --localize 로 넘겨 덮어씀
  std::vector<std::string> localize_robots;
  int localize_wait_sec = 60;      // 로봇이 연결 · 위치 확인될 때까지 기다리는 최대 시간 (넘으면 건너뜀)
  int localize_timeout_sec = 90;   // LOCALIZE 하나가 끝나기를 기다리는 최대 시간 (넘으면 알림 뒤 다음 로봇)
  bool stay_at_start = false;      // 시험용: 도킹 대신 켤 때 노드를 집으로 (쉬면 거기서 대기, 작업 끝나면 거기로 돌아감)
  double comm_timeout_sec = 3.0; // state가 이만큼 안 오면 통신 끊김
  int lock_lookahead = 2;        // 로봇 앞으로 미리 잡아 두는 lock 노드 수
  double detour_max_m = 1.0;     // 앞 로봇이 움직이는 중이면, 우회가 이만큼(m) 이하로 길어질 때만 돌아간다
  double detour_wait_sec = 8.0;  // 이만큼 기다려도 안 풀리면 길이 상관없이 우회
  double avoid_sec = 6.0;        // 우회를 정하면 막힌 노드를 이 시간 동안 경로 계산에서 뺀다
  double localize_radius_m = 0.15;
  int done_keep_sec = 600;       // 끝난 작업을 매장 화면에 남기는 시간

  std::vector<TableConfig> tables;
  std::vector<MenuItem> menu;
  DisplayTexts display;
};

// 파일을 읽어 cfg에 채운다. 실패하면 false + error에 이유
bool loadConfig(const std::string& path, Config& cfg, std::string& error);

}  // namespace pinky
