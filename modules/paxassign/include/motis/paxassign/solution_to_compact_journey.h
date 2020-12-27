#pragma once

#include "motis/core/schedule/schedule.h"

#include "motis/paxmon/compact_journey.h"

#include "motis/paxassign/time_expanded_graph.h"

namespace motis::paxassign {
std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
node_arc_solution_to_compact_j(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& na_solution,
    schedule const& sched);
}