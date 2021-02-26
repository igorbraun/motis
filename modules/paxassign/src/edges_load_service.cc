#include "motis/paxassign/edges_load_service.h"

#include "motis/paxassign/service_functions.h"

#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"

#include "motis/paxmon/graph.h"
#include "utl/enumerate.h"

#include <iostream>
#include <map>
#include <numeric>

namespace motis::paxassign {

std::map<eg_edge*, uint32_t> get_edges_load_from_solutions(
    std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>> const&
        assignments,
    time_expanded_graph const& te_graph, schedule const& sched) {
  std::map<eg_edge*, uint32_t> edges_load;
  for (auto const& asg : assignments) {
    for (auto const [leg_idx, leg] : utl::enumerate(asg.second.legs_)) {

      if (te_graph.trip_data_.find(to_extern_trip(sched, leg.trip_)) ==
          te_graph.trip_data_.end()) {
        continue;
      }
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

std::map<eg_edge*, uint32_t> get_final_edges_load_for_solution(
    std::set<eg_edge*> const& all_affected_edges,
    std::map<eg_edge*, uint32_t> const& loads_from_sol) {
  std::map<eg_edge*, uint32_t> resulting_load;
  for (auto* e : all_affected_edges) {
    resulting_load[e] = e->passengers_;
    if (loads_from_sol.find(e) != loads_from_sol.end()) {
      resulting_load[e] += loads_from_sol.at(e);
    }
  }
  return resulting_load;
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
              << e->from_->time_
              << ", load without new assignment: " << e->passengers_ << " / "
              << e->soft_cap_boundary_ << std::endl;
  }
}

std::vector<double> get_relative_loads(
    std::map<eg_edge*, uint32_t> const& loads) {
  std::vector<double> distr;
  for (auto const& e : loads) {
    distr.push_back((double)e.second / e.first->soft_cap_boundary_);
  }
  std::sort(distr.begin(), distr.end());
  return distr;
}

std::vector<double> get_load_histogram(
    std::map<eg_edge*, uint32_t> const& loads,
    std::vector<double> const& ranges) {
  auto relative_loads = get_relative_loads(loads);

  std::vector<std::uint16_t> values_in_ranges;
  std::vector<double> hist;

  double lower = -1.0;
  double upper = -1.0;
  for (auto i = 0u; i < ranges.size(); ++i) {
    upper = ranges[i];
    values_in_ranges.push_back(std::count_if(
        relative_loads.begin(), relative_loads.end(),
        [&](double const val) { return val > lower && val <= upper; }));
    lower = upper;
  }

  for (auto const val_in_r : values_in_ranges) {
    hist.push_back((double)val_in_r / relative_loads.size());
  }

  return hist;
}

}  // namespace motis::paxassign