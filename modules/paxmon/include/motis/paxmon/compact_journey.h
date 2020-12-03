#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "cista/reflection/comparable.h"

#include "motis/core/schedule/time.h"
#include "motis/core/schedule/trip.h"

#include "motis/paxmon/transfer_info.h"

namespace motis::paxmon {

struct journey_leg {
  CISTA_COMPARABLE()

  trip const* trip_;
  unsigned enter_station_id_;
  unsigned exit_station_id_;
  motis::time enter_time_;
  motis::time exit_time_;
  std::optional<transfer_info> enter_transfer_;
};

inline bool operator==(journey_leg const& lhs, journey_leg const& rhs) {
  return lhs.trip_ == rhs.trip_ &&
         lhs.enter_station_id_ == rhs.enter_station_id_ &&
         lhs.exit_station_id_ == rhs.exit_station_id_ &&
         lhs.enter_time_ == rhs.enter_time_ &&
         lhs.exit_time_ == rhs.exit_time_;
}

struct compact_journey {
  CISTA_COMPARABLE()

  std::vector<journey_leg> legs_;

  inline unsigned destination_station_id() const {
    return legs_.back().exit_station_id_;
  }

  bool operator==(compact_journey const& another_cj) {
    return legs_ == another_cj.legs_;
  }
};

}  // namespace motis::paxmon
