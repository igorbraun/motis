#pragma once

#include "motis/paxmon/compact_journey.h"
#include "motis/paxmon/paxmon_data.h"

namespace motis::paxassign {

std::set<motis::paxmon::edge*> get_edges_from_solutions(
    std::vector<std::pair<std::uint16_t, motis::paxmon::compact_journey>> const&
        assignments,
    motis::paxmon::paxmon_data const& data);
}