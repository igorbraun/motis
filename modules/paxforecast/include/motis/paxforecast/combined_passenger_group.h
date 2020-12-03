#pragma once

#include <vector>

#include "motis/core/journey/journey.h"

#include "motis/paxmon/localization.h"
#include "motis/paxmon/passenger_group.h"

#include "motis/paxforecast/alternatives.h"

namespace motis::paxforecast {

struct combined_passenger_group {
  unsigned destination_station_id_{};
  std::uint16_t passengers_{};
  motis::paxmon::passenger_localization localization_;
  std::vector<motis::paxmon::passenger_group const*> groups_;
  // TODO: move it to paxassign, paxforecast should be deactivated at all
  std::vector<alternative> alternatives_;
};

}  // namespace motis::paxforecast
