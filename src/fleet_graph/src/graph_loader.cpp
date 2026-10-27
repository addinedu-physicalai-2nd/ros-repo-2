// node_graph.yaml 읽기 (yaml-cpp)
#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "fleet_graph/graph.hpp"

namespace fleet_graph
{

namespace
{
template<typename T>
T get_or(const YAML::Node & n, const char * key, const T & def)
{
  return n[key] ? n[key].as<T>() : def;
}

Drive parse_drive(const std::string & s, const std::string & id)
{
  if (s == "FORWARD") {
    return Drive::FORWARD;
  }
  if (s == "REVERSE") {
    return Drive::REVERSE;
  }
  throw std::runtime_error(id + ": entry는 FORWARD / REVERSE 중 하나 (" + s + ")");
}
}  // namespace

Graph Graph::load(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("노드 그래프를 읽을 수 없음: " + yaml_path + " (" + e.what() + ")");
  }
  if (!root["nodes"] || !root["nodes"].IsSequence()) {
    throw std::runtime_error("nodes 목록이 없음: " + yaml_path);
  }

  Graph g;
  for (const auto & y : root["nodes"]) {
    Node n;
    n.id = y["id"].as<std::string>();
    n.type = y["type"].as<std::string>();
    n.stop_kind = get_or<std::string>(y, "stop_kind", "");
    n.table = get_or<std::string>(y, "table", "");
    n.pocket = get_or<bool>(y, "pocket", false);
    n.entry = parse_drive(get_or<std::string>(y, "entry", "FORWARD"), n.id);
    n.lock = get_or<bool>(y, "lock", true);
    n.no_wait = get_or<bool>(y, "no_wait", false);
    n.x = y["x"].as<double>();
    n.y = y["y"].as<double>();
    if (y["yaw"]) {
      n.yaw = y["yaw"].as<double>();
    }
    if (y["next"]) {
      n.next = y["next"].as<std::vector<std::string>>();
    }
    g.add_node(n);
  }

  if (root["reverse"] && root["reverse"].IsSequence()) {
    for (const auto & chain : root["reverse"]) {
      g.add_reverse_chain(chain.as<std::vector<std::string>>());
    }
  }
  return g;
}

}  // namespace fleet_graph
