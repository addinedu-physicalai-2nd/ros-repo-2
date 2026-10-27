// 노드 그래프 점검 도구 (로봇 없이 실행)
//   ros2 run fleet_graph graph_check <node_graph.yaml>               # 읽기 + 검사
//   ros2 run fleet_graph graph_check <node_graph.yaml> N13 D3        # 경로와 Move 구간까지 출력
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

#include "fleet_graph/graph.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf("사용: graph_check <node_graph.yaml> [출발 노드] [도착 노드]\n");
    return 1;
  }
  try {
    const auto g = fleet_graph::Graph::load(argv[1]);
    std::printf("노드 %zu개, 후진 구간 %zu개\n", g.nodes().size(), g.reverse_chains().size());

    const auto problems = g.validate();
    if (problems.empty()) {
      std::printf("검사: 문제 없음\n");
    } else {
      for (const auto & p : problems) {
        std::printf("문제: %s\n", p.c_str());
      }
    }

    if (argc >= 4) {
      const auto path = g.shortest_path(argv[2], argv[3]);
      if (path.empty()) {
        std::printf("%s -> %s 경로 없음\n", argv[2], argv[3]);
        return 2;
      }
      std::printf("경로:");
      for (const auto & id : path) {
        std::printf(" %s", id.c_str());
      }
      std::printf("\n");
      int k = 0;
      for (const auto & s : g.to_segments(path)) {
        std::printf("Move %d  %s\n", ++k, fleet_graph::to_string(s.drive));
        for (const auto & w : s.waypoints) {
          std::printf("  %-4s x=%.3f y=%.3f yaw=%+.0f°\n", w.node_id.c_str(), w.x, w.y, w.yaw * 180.0 / M_PI);
        }
      }
    }
    return problems.empty() ? 0 : 3;
  } catch (const std::exception & e) {
    std::printf("오류: %s\n", e.what());
    return 1;
  }
}
