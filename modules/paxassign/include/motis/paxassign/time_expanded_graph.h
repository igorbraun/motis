#pragma once

namespace motis::paxassign {

struct eg_edge;
struct time_expanded_graph;

enum class eg_event_type : uint8_t { DEP, ARR, WAIT };

inline std::ostream& operator<<(std::ostream& o, eg_event_type const t) {
  switch (t) {
    case eg_event_type::ARR: return o << "ARR";
    case eg_event_type::DEP: return o << "DEP";
    case eg_event_type::WAIT: return o << "WAIT";
  }
  return o;
}

struct eg_event_node {
  time time_{INVALID_TIME};
  eg_event_type type_{eg_event_type::ARR};
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
  uint32_t cost_{0};
  std::uint16_t capacity_{};
  struct trip const* trip_{};
};

inline std::string eg_edge_type_to_string(eg_edge const* e) {
  switch (e->type_) {
    case eg_edge_type::TRIP: return "TRIP";
    case eg_edge_type::INTERCHANGE: return "INTERCHANGE";
    case eg_edge_type::WAIT: return "WAIT";
    case eg_edge_type::NO_ROUTE: return "NO_ROUTE";
  }
  return "";
}

struct eg_trip_data {
  std::vector<eg_edge*> edges_;
};

struct time_expanded_graph {
  std::vector<std::unique_ptr<eg_event_node>> nodes_;
  mcd::hash_map<extern_trip, std::unique_ptr<eg_trip_data>> trip_data_;
  std::vector<eg_edge*> not_trip_edges_;
};

}  // namespace motis::paxassign