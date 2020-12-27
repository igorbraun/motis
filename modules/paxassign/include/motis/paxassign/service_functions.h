#pragma once

#include "motis/paxmon/compact_journey.h"
#include "motis/paxmon/graph.h"
#include "motis/paxmon/paxmon_data.h"

namespace motis::paxassign {

template <typename T>
std::size_t get_vec_size_bytes(std::vector<T> const& vec) {
  return sizeof(std::vector<T>) + (sizeof(T) * vec.size());
}

template <typename T>
double get_vec_of_vec_mb_size(std::vector<std::vector<T>> const& vec_of_vec) {
  if (vec_of_vec.size() == 0) return 0;
  return vec_of_vec.size() * get_vec_size_bytes(vec_of_vec[0]) / 1024 / 1024;
}

inline uint32_t find_edge_idx(motis::paxmon::trip_data const* td,
                              motis::paxmon::journey_leg const& leg,
                              bool const from) {
  auto result_edge =
      std::find_if(begin(td->edges_), end(td->edges_), [&](auto const& e) {
        if (from) {
          return e->from_->station_ == leg.enter_station_id_;
        } else {
          return e->to_->station_ == leg.exit_station_id_;
        }
      });
  if (result_edge != end(td->edges_)) {
    return distance(begin(td->edges_), result_edge);
  } else {
    return std::numeric_limits<uint32_t>::max();
  }
}

inline uint32_t find_edge_idx(eg_trip_data const* td,
                              motis::paxmon::journey_leg const& leg,
                              bool const from) {
  auto result_edge =
      std::find_if(begin(td->edges_), end(td->edges_), [&](auto const& e) {
        if (from) {
          return e->from_->station_ == leg.enter_station_id_;
        } else {
          return e->to_->station_ == leg.exit_station_id_;
        }
      });
  if (result_edge != end(td->edges_)) {
    return distance(begin(td->edges_), result_edge);
  } else {
    return std::numeric_limits<uint32_t>::max();
  }
}

}  // namespace motis::paxassign