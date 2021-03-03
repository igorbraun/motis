#include "motis/paxassign/solution_to_compact_journey.h"

#include <iostream>

#include "utl/get_or_create.h"
#include "utl/pipes/all.h"
#include "utl/pipes/remove_if.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

#include "motis/paxmon/util/interchange_time.h"

namespace motis::paxassign {
std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
node_arc_solution_to_compact_j(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& na_solution,
    schedule const& sched) {
  std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>> result;
  for (auto i = 0u; i < na_solution.size(); ++i) {
    auto no_route_edge = std::find_if(
        na_solution[i].begin(), na_solution[i].end(),
        [](eg_edge* e) { return e->type_ == eg_edge_type::NO_ROUTE; });
    if (no_route_edge != na_solution[i].end()) {
      result.push_back(
          {eg_psg_groups[i].cpg_, motis::paxmon::compact_journey{}});
      continue;
    }

    std::vector<eg_edge*> trip_edges = utl::all(na_solution[i]) |
                                       utl::remove_if([&](auto const& e) {
                                         return e->type_ != eg_edge_type::TRIP;
                                       }) |
                                       utl::vec();
    motis::paxmon::compact_journey cj{};

    bool first_trip = true;
    trip const* curr_trip = nullptr;
    eg_edge* from_e = nullptr;
    eg_edge* to_e = nullptr;
    for (auto const& e : trip_edges) {
      if (curr_trip != e->trip_) {
        if (curr_trip != nullptr) {
          // add leg
          if (first_trip) {
            first_trip = false;
            cj.legs_.emplace_back(motis::paxmon::journey_leg{
                curr_trip, from_e->from_->station_, to_e->to_->station_,
                from_e->from_->time_, to_e->to_->time_,
                motis::paxmon::transfer_info{}});
          } else {
            cj.legs_.emplace_back(motis::paxmon::journey_leg{
                curr_trip,
                from_e->from_->station_,
                to_e->to_->station_,
                from_e->from_->time_,
                to_e->to_->time_,
                motis::paxmon::util::get_transfer_info(
                    sched, from_e->from_->station_, to_e->to_->station_),
            });
          }
        }
        curr_trip = e->trip_;
        from_e = e;
        to_e = e;
      } else {
        to_e = e;
      }
    }

    if (curr_trip == nullptr || from_e == nullptr || to_e == nullptr) {
      result.push_back(
          {eg_psg_groups[i].cpg_, motis::paxmon::compact_journey{}});
      continue;
    }
    if (first_trip) {
      first_trip = false;
      cj.legs_.emplace_back(motis::paxmon::journey_leg{
          curr_trip, from_e->from_->station_, to_e->to_->station_,
          from_e->from_->time_, to_e->to_->time_,
          motis::paxmon::transfer_info{}});
    } else {
      cj.legs_.emplace_back(motis::paxmon::journey_leg{
          curr_trip,
          from_e->from_->station_,
          to_e->to_->station_,
          from_e->from_->time_,
          to_e->to_->time_,
          motis::paxmon::util::get_transfer_info(sched, from_e->from_->station_,
                                                 to_e->to_->station_),
      });
    }

    result.push_back({eg_psg_groups[i].cpg_, cj});
  }
  return result;
}
}  // namespace motis::paxassign