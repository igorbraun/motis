#include "motis/paxassign/get_edges_from_solutions.h"

#include "motis/paxassign/service_functions.h"

#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"

#include "motis/paxmon/graph.h"
#include "utl/enumerate.h"

#include <iostream>
#include <map>

namespace motis::paxassign {

/*
std::map<motis::paxmon::edge*, uint32_t> get_edges_load_from_solutions(
    std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>> const&
        assignments,
    motis::paxmon::paxmon_data const& data) {
  std::map<motis::paxmon::edge*, uint32_t> edges_load;
  for (auto const& asg : assignments) {
    for (auto const [leg_idx, leg] : utl::enumerate(asg.second.legs_)) {

      auto const td = data.graph_.trip_data_.at(leg.trip_).get();

      auto start_edge_idx = find_edge_idx(td, leg, true);
      auto last_edge_idx = find_edge_idx(td, leg, false);

      for (auto i = start_edge_idx; i <= last_edge_idx; ++i) {
        if (edges_load.find(td->edges_[i]) == edges_load.end()) {
          edges_load[td->edges_[i]] = asg.first.passengers_;
        } else {
          edges_load[td->edges_[i]] += asg.first.passengers_;
        }
      }
    }
  }
  return edges_load;
}
*/

std::map<eg_edge*, uint32_t> get_edges_load_from_solutions(
    std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>> const&
        assignments,
    time_expanded_graph const& te_graph, schedule const& sched) {
  std::map<eg_edge*, uint32_t> edges_load;
  for (auto const& asg : assignments) {
    for (auto const [leg_idx, leg] : utl::enumerate(asg.second.legs_)) {

      auto const td =
          te_graph.trip_data_.at(to_extern_trip(sched, leg.trip_)).get();

      auto start_edge_idx = find_edge_idx(td, leg, true);
      auto last_edge_idx = find_edge_idx(td, leg, false);

      for (auto i = start_edge_idx; i <= last_edge_idx; ++i) {
        if (edges_load.find(td->edges_[i]) == edges_load.end()) {
          edges_load[td->edges_[i]] = asg.first.passengers_;
        } else {
          edges_load[td->edges_[i]] += asg.first.passengers_;
        }
      }
    }
  }
  return edges_load;
}

void add_affected_edges_from_sol(
    std::map<eg_edge*, uint32_t> const& affected_edges,
    std::set<eg_edge*>& result) {
  for (auto const& p : affected_edges) {
    result.insert(p.first);
  }
}

void print_edges_load(std::map<eg_edge*, uint32_t> const& edges_load,
                      schedule const& sched) {
  for (auto const& p : edges_load) {
    std::cout << p.first->trip_->id_.primary_.train_nr_ << ": "
              << sched.stations_[p.first->from_->station_]->name_ << " to "
              << sched.stations_[p.first->to_->station_]->name_ << " at "
              << p.first->from_->time_ << " load: " << p.second << std::endl;
  }
}

void print_affected_edges(std::set<eg_edge*> const& affected_edges,
                          schedule const& sched) {
  for (auto const* e : affected_edges) {
    std::cout << e->trip_->id_.primary_.train_nr_ << ": "
              << sched.stations_[e->from_->station_]->name_ << " to "
              << sched.stations_[e->to_->station_]->name_ << " at "
              << e->from_->time_ << std::endl;
  }
}

}  // namespace motis::paxassign