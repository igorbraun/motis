#include "motis/paxassign/get_edges_from_solutions.h"

#include "motis/paxassign/service_functions.h"

#include "motis/paxmon/graph.h"
#include "utl/enumerate.h"

#include <iostream>

namespace motis::paxassign {

std::set<motis::paxmon::edge*> get_edges_from_solutions(
    std::vector<std::pair<std::uint16_t, motis::paxmon::compact_journey>> const&
        assignments,
    motis::paxmon::paxmon_data const& data) {
  std::cout << "get edges from solutions" << std::endl;
  std::set<motis::paxmon::edge*> affected_edges;

  for (auto const& asg : assignments) {
    for (auto const [leg_idx, leg] : utl::enumerate(asg.second.legs_)) {
      auto const td = data.graph_.trip_data_.at(leg.trip_).get();

      auto start_edge_idx = find_edge_idx(td, leg, true);
      auto last_edge_idx = find_edge_idx(td, leg, false);

      for (auto i = start_edge_idx; i <= last_edge_idx; ++i) {
        affected_edges.insert(td->edges_[i]);
      }
    }
  }

  return affected_edges;
}

}  // namespace motis::paxassign