#pragma once

#include <vector>

#include "motis/core/journey/journey.h"

#include "motis/paxmon/localization.h"
#include "motis/paxmon/passenger_group.h"

#include "motis/paxforecast/alternatives.h"

namespace motis::paxassign {

struct combined_pg {
  std::uint16_t id_;
  unsigned destination_station_id_{};
  std::uint16_t passengers_{};
  motis::paxmon::passenger_localization localization_;
  std::vector<motis::paxmon::passenger_group const*> groups_;
  std::vector<motis::paxforecast::alternative> alternatives_;
};

}  // namespace motis::paxassign