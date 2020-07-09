#pragma once

#include <cstddef>
#include <cstdint>

#include "motis/core/schedule/schedule.h"

#include "motis/paxmon/paxmon_data.h"

namespace motis::paxmon::loader::journeys {

std::size_t load_journeys(schedule const& sched, paxmon_data& data,
                          std::string const& journey_file);

}  // namespace motis::paxmon::loader::journeys
