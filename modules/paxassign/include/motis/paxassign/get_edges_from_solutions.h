#pragma once

#include "motis/paxassign/combined_pg.h"
#include "motis/paxassign/time_expanded_graph.h"
#include "motis/paxmon/compact_journey.h"
#include "motis/paxmon/paxmon_data.h"

#include "motis/core/schedule/schedule.h"

namespace motis::paxassign {

std::map<eg_edge*, uint32_t> get_edges_load_from_solutions(
    std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>> const&
        assignments,
    time_expanded_graph const& te_graph, schedule const& sched);
void print_edges_load(std::map<eg_edge*, uint32_t> const& edges_load,
                      schedule const& sched);
void add_affected_edges_from_sol(
    std::map<eg_edge*, uint32_t> const& affected_edges,
    std::set<eg_edge*>& result);
void print_affected_edges(std::set<eg_edge*> const& affected_edges,
                          schedule const& sched);
}  // namespace motis::paxassign