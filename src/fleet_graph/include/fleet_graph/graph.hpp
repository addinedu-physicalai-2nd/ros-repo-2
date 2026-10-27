// fleet_graph: 노드 그래프(fleet_msgs/docs/node_graph.yaml)를 C++에서 읽고 쓰는 라이브러리
//
// 메인 서비스(서버)가 쓰는 기능만 담았다.
//   - yaml 읽기 / 검사
//   - 최단 경로 (방향 있는 엣지, 일방통행 반영)
//   - 경로를 전진 / 후진 구간으로 나누기 (Move 하나 = 방향 하나, README 4.9)
//   - 각 웨이포인트의 x, y, yaw 계산 (yaw = 로봇 "앞"이 보는 방향)
// lock 관리, 작업 배정 같은 서버 로직은 여기 없다. 메인 서비스에서 이 라이브러리를 가져다 쓴다.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace fleet_graph
{

// fleet_msgs::msg::Move::FORWARD / REVERSE 와 같은 값
enum class Drive : uint8_t { FORWARD = 0, REVERSE = 1 };

struct Node
{
  std::string id;
  std::string type;        // "PASS" / "STOP"
  std::string stop_kind;   // STOP일 때: QUEUE / PICKUP / SERVE / DOCK
  std::string table;       // SERVE일 때
  bool pocket = false;
  Drive entry = Drive::FORWARD;   // pocket 진입 방향
  bool lock = true;
  bool no_wait = false;
  double x = 0.0;
  double y = 0.0;
  std::optional<double> yaw;      // STOP 노드만
  std::vector<std::string> next;  // 갈 수 있는 다음 노드 (방향 있음)

  bool is_stop() const {return type == "STOP";}
};

// 방향 하나짜리 구간 = Move 메시지 하나
struct Waypoint
{
  std::string node_id;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Segment
{
  Drive drive = Drive::FORWARD;
  std::vector<Waypoint> waypoints;
};

class Graph
{
public:
  // yaml 파일에서 읽기 (graph_loader.cpp, yaml-cpp 사용). 실패하면 std::runtime_error
  static Graph load(const std::string & yaml_path);

  // 코드로 직접 만들 때 (테스트용)
  void add_node(const Node & n);
  void add_reverse_chain(const std::vector<std::string> & chain);

  bool has(const std::string & id) const;
  const Node & node(const std::string & id) const;   // 없으면 std::out_of_range
  const std::map<std::string, Node> & nodes() const {return nodes_;}
  const std::vector<std::vector<std::string>> & reverse_chains() const {return reverse_chains_;}

  bool has_edge(const std::string & a, const std::string & b) const;

  // 경로 path에서 path[i-1] -> path[i] 엣지를 전진으로 가는지 후진으로 가는지
  //   - reverse 구간에 적힌 엣지: 후진
  //   - 경로의 도착지가 pocket이면 마지막 엣지는 그 pocket의 entry 방향 (D1~D4는 REVERSE = 바구니부터)
  //   - 경로의 출발지가 pocket이면 첫 엣지는 entry의 반대 (REVERSE로 들어간 도킹은 FORWARD로 나옴)
  //   - 나머지(지나가기만 하는 pocket 포함): 전진
  Drive drive_on(const std::vector<std::string> & path, size_t i) const;

  // 문제 목록. 비어 있으면 정상
  std::vector<std::string> validate() const;

  // 최단 경로 (거리 기준, 방향 있는 엣지만). [from, ..., to], 길이 없으면 빈 벡터
  std::vector<std::string> shortest_path(const std::string & from, const std::string & to) const;

  // 경로 -> 방향별 구간. path[0]은 로봇이 지금 있는 노드라 웨이포인트에 넣지 않는다.
  // 방향이 바뀌는 곳, 그리고 pocket에 드나드는 곳에서 구간(Move)을 끊는다 (README 4.5, 4.9).
  // yaw 규칙: 그 노드로 들어오는 방향(후진 구간이면 +180°). 경로의 마지막이 STOP 노드면 그 노드의 yaw.
  std::vector<Segment> to_segments(const std::vector<std::string> & path) const;

private:
  std::map<std::string, Node> nodes_;
  std::set<std::pair<std::string, std::string>> reverse_edges_;
  std::vector<std::vector<std::string>> reverse_chains_;
};

const char * to_string(Drive d);

}  // namespace fleet_graph
