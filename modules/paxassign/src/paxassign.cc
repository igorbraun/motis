#include "motis/paxassign/paxassign.h"

#include <iostream>

#include "motis/core/common/logging.h"

#include "motis/paxassign/build_toy_scenario.h"
#include "motis/paxforecast/messages.h"
#include "motis/paxmon/data_key.h"
#include "motis/paxmon/graph_access.h"
#include "motis/paxmon/messages.h"

using namespace motis::module;
using namespace motis::logging;
using namespace motis::paxmon;
using namespace motis::paxforecast;

namespace motis::paxassign {

paxassign::paxassign() : module("Passenger Assignment", "paxassign") {}

paxassign::~paxassign() = default;

void paxassign::init(motis::module::registry& reg) {
  reg.subscribe("/paxforecast/passenger_forecast", [&](msg_ptr const& msg) {
    on_forecast(msg);
    return nullptr;
  });

  reg.subscribe("/paxforecast/toy_scenario", [&](msg_ptr const& msg) {
    toy_scenario(msg);
    return nullptr;
  });
}

void paxassign::toy_scenario(const motis::module::msg_ptr& msg) {
  std::cout << "paxassign toyscenario" << std::endl;
  build_toy_scenario();
}

void paxassign::on_forecast(const motis::module::msg_ptr& msg) {
  auto const& sched = get_sched();
  auto& data = *get_shared_data<paxmon_data*>(motis::paxmon::DATA_KEY);

  auto const forecast = motis_content(PassengerForecast, msg);

  LOG(info) << "received passenger forecast: over capacity="
            << forecast->sim_result()->over_capacity();

  for (auto const& group_forecast : *forecast->groups()) {
    auto const& group = data.get_passenger_group(group_forecast->group()->id());
    auto const localization =
        from_fbs(sched, group_forecast->localization_type(),
                 group_forecast->localization());
    auto const forecast_journey =
        from_fbs(sched, group_forecast->forecast_journey());

    LOG(info) << "  group " << group_forecast->group()->id()
              << ": at_station=" << localization.at_station_->eva_nr_
              << ", in_trip=" << localization.in_trip() << ", destination="
              << sched
                     .stations_[group.compact_planned_journey_
                                    .destination_station_id()]
                     ->eva_nr_;

    auto edges_over_capacity = 0;
    for_each_edge(sched, data, forecast_journey,
                  [&](journey_leg const&, motis::paxmon::edge* e) {
                    if (e->passengers() > e->capacity()) {
                      ++edges_over_capacity;
                    }
                  });
    LOG(info) << "    edges over capacity in forecast journey: "
              << edges_over_capacity;
  }
}

}  // namespace motis::paxassign
