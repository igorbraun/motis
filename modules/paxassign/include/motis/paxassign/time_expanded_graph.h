#pragma once

namespace motis::paxassign {

struct eg_edge;
struct time_expanded_graph;

struct eg_event_node {
  time time_{INVALID_TIME};
  event_type type_{event_type::ARR};
  std::uint32_t station_{0};
  std::vector<std::unique_ptr<eg_edge>> out_edges_;
  std::vector<eg_edge*> in_edges_;
  size_t id_;
};

enum class eg_edge_type : std::uint8_t { TRIP, INTERCHANGE, WAIT, NO_ROUTE };

inline std::ostream& operator<<(std::ostream& out, eg_edge_type const et) {
  switch (et) {
    case eg_edge_type::TRIP: return out << "TRIP";
    case eg_edge_type::INTERCHANGE: return out << "INTERCHANGE";
    case eg_edge_type::WAIT: return out << "WAIT";
    case eg_edge_type::NO_ROUTE: return out << "NO_ROUTE";
  }
  return out;
}

struct eg_edge {
  eg_event_node* from_{};
  eg_event_node* to_{};
  eg_edge_type type_{};
  uint32_t transfer_time_{};
  std::uint16_t capacity_{};
  struct trip const* trip_{};
};

struct eg_trip_data {
  std::vector<eg_edge*> edges_;
};

struct time_expanded_graph {
  std::vector<std::unique_ptr<eg_event_node>> nodes_;
  mcd::hash_map<extern_trip, std::unique_ptr<eg_trip_data>> trip_data_;
  std::vector<eg_edge*> interchange_edges_;
  std::vector<eg_edge*> no_route_edges_;
};

}  // namespace motis::paxassign