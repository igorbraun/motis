#pragma once

#include <tuple>

#include "motis/core/schedule/schedule.h"

#include "motis/rsl/reachability.h"

namespace motis::rsl {

struct passenger_localization {
  trip const* in_trip_{};
  station const* at_station_{};
  time arrival_time_{INVALID_TIME};

  inline friend bool operator==(passenger_localization const& lhs,
                                passenger_localization const& rhs) {
    return std::tie(lhs.in_trip_, lhs.at_station_, lhs.arrival_time_) ==
           std::tie(rhs.in_trip_, rhs.at_station_, rhs.arrival_time_);
  }

  inline friend bool operator!=(passenger_localization const& lhs,
                                passenger_localization const& rhs) {
    return std::tie(lhs.in_trip_, lhs.at_station_, lhs.arrival_time_) !=
           std::tie(rhs.in_trip_, rhs.at_station_, rhs.arrival_time_);
  }

  bool in_trip() const { return in_trip_ != nullptr; }
};

passenger_localization localize(schedule const& sched,
                                reachability_info const& reachability,
                                time localization_time);

}  // namespace motis::rsl