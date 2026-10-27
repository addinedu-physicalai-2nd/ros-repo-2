// fleet_graph 핵심 로직 (yaml-cpp, ROS 없이 컴파일된다)
#include "fleet_graph/graph.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace fleet_graph
{

namespace
{
double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

const char * to_string(Drive d)
{
  return d == Drive::REVERSE ? "REVERSE" : "FORWARD";
}

void Graph::add_node(const Node & n)
{
  if (nodes_.count(n.id)) {
    throw std::runtime_error("노드 ID 중복: " + n.id);
  }
  nodes_[n.id] = n;
}

void Graph::add_reverse_chain(const std::vector<std::string> & chain)
{
  reverse_chains_.push_back(chain);
  for (size_t i = 1; i < chain.size(); ++i) {
    reverse_edges_.insert({chain[i - 1], chain[i]});
  }
}

bool Graph::has(const std::string & id) const
{
  return nodes_.count(id) > 0;
}

const Node & Graph::node(const std::string & id) const
{
  return nodes_.at(id);
}

bool Graph::has_edge(const std::string & a, const std::string & b) const
{
  auto it = nodes_.find(a);
  if (it == nodes_.end()) {
    return false;
  }
  for (const auto & n : it->second.next) {
    if (n == b) {
      return true;
    }
  }
  return false;
}

Drive Graph::drive_on(const std::vector<std::string> & path, size_t i) const
{
  const Node & a = node(path[i - 1]);
  const Node & b = node(path[i]);
  if (reverse_edges_.count({a.id, b.id})) {
    return Drive::REVERSE;
  }
  if (i + 1 == path.size() && b.pocket) {
    return b.entry;                      // 도착 pocket으로 들어가는 방향
  }
  if (i == 1 && a.pocket) {              // 출발 pocket에서 나오는 방향 = 들어간 방향의 반대
    return a.entry == Drive::FORWARD ? Drive::REVERSE : Drive::FORWARD;
  }
  return Drive::FORWARD;
}

std::vector<std::string> Graph::validate() const
{
  std::vector<std::string> p;
  for (const auto & [id, n] : nodes_) {
    for (const auto & nx : n.next) {
      if (!has(nx)) {
        p.push_back(id + ": next에 없는 노드 " + nx);
      }
    }
    if (n.type != "PASS" && n.type != "STOP") {
      p.push_back(id + ": type은 PASS / STOP 이어야 함 (" + n.type + ")");
    }
    if (n.is_stop() && !n.yaw) {
      p.push_back(id + ": STOP 노드인데 yaw가 없음");
    }
    if (n.entry == Drive::REVERSE && !n.pocket) {
      p.push_back(id + ": entry REVERSE는 pocket 노드에만 쓴다");
    }
  }
  for (const auto & chain : reverse_chains_) {
    if (chain.size() < 2) {
      p.push_back("reverse 구간은 노드가 2개 이상이어야 함");
      continue;
    }
    for (size_t i = 1; i < chain.size(); ++i) {
      if (!has_edge(chain[i - 1], chain[i])) {
        p.push_back("reverse 구간의 엣지가 그래프에 없음: " + chain[i - 1] + " -> " + chain[i]);
      }
    }
    for (const auto & end : {chain.front(), chain.back()}) {
      if (!has(end)) {
        p.push_back("reverse 구간에 없는 노드: " + end);
      } else if (!node(end).lock) {
        p.push_back("reverse 구간 끝 노드는 lock 노드여야 함: " + end);
      }
    }
  }
  return p;
}

std::vector<std::string> Graph::shortest_path(const std::string & from, const std::string & to) const
{
  if (!has(from) || !has(to)) {
    return {};
  }
  std::map<std::string, double> dist;
  std::map<std::string, std::string> prev;
  using Item = std::pair<double, std::string>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  dist[from] = 0.0;
  pq.push({0.0, from});
  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) {
      continue;
    }
    if (u == to) {
      break;
    }
    const Node & nu = node(u);
    for (const auto & v : nu.next) {
      if (!has(v)) {
        continue;
      }
      const Node & nv = node(v);
      double nd = d + std::hypot(nv.x - nu.x, nv.y - nu.y);
      auto it = dist.find(v);
      if (it == dist.end() || nd < it->second) {
        dist[v] = nd;
        prev[v] = u;
        pq.push({nd, v});
      }
    }
  }
  if (!dist.count(to)) {
    return {};
  }
  std::vector<std::string> path{to};
  while (path.back() != from) {
    path.push_back(prev.at(path.back()));
  }
  return {path.rbegin(), path.rend()};
}

std::vector<Segment> Graph::to_segments(const std::vector<std::string> & path) const
{
  std::vector<Segment> segs;
  double last_heading = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const Node & a = node(path[i - 1]);
    const Node & b = node(path[i]);
    const Drive d = drive_on(path, i);
    const bool pocket_edge = (i == 1 && a.pocket) || (i + 1 == path.size() && b.pocket);
    if (segs.empty() || segs.back().drive != d || pocket_edge) {
      segs.push_back(Segment{d, {}});
    }
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) > 1e-6) {
      last_heading = std::atan2(dy, dx);   // 진행 방향
    }
    double yaw = (d == Drive::REVERSE) ? wrap(last_heading + M_PI) : last_heading;
    if (i + 1 == path.size() && b.is_stop() && b.yaw) {
      yaw = *b.yaw;                        // 목적지 STOP 노드는 정해진 방향
    }
    segs.back().waypoints.push_back(Waypoint{b.id, b.x, b.y, yaw});
  }
  return segs;
}

}  // namespace fleet_graph
