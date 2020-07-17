#include "motis/paxassign/paxassign.h"

#include <iostream>

#include "motis/core/common/date_time_util.h"
#include "motis/core/common/logging.h"
#include "motis/core/access/service_access.h"
#include "motis/module/context/get_schedule.h"
#include "motis/module/context/motis_call.h"
#include "motis/module/context/motis_publish.h"
#include "motis/module/context/motis_spawn.h"
#include "motis/module/message.h"

#include "motis/paxforecast/messages.h"

#include "motis/paxmon/data_key.h"
#include "motis/paxmon/graph_access.h"
#include "motis/paxmon/messages.h"

#include "motis/paxassign/build_toy_scenario.h"

using namespace motis::module;
using namespace motis::logging;
using namespace motis::routing;
using namespace motis::paxmon;
using namespace motis::rt;
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

  reg.subscribe("/paxmon/monitoring_update", [&](msg_ptr const& msg) {
    on_monitoring(msg);
    return nullptr;
  });
}

void paxassign::toy_scenario(const motis::module::msg_ptr&) {
  std::cout << "paxassign toyscenario" << std::endl;
  build_toy_scenario();
}

void paxassign::on_monitoring(const motis::module::msg_ptr& msg) {
  std::cout << "IN PAXASSIGN::ON_MONITORING" << std::endl;
  auto const& sched = get_schedule();
  auto& data = *get_shared_data<paxmon_data*>(motis::paxmon::DATA_KEY);

  auto const mon_update = motis_content(MonitoringUpdate, msg);

  auto const current_time =
      unix_to_motistime(sched.schedule_begin_, sched.system_time_);
  utl::verify(current_time != INVALID_TIME, "invalid current system time");

  std::map<unsigned, std::vector<combined_passenger_group>> combined_groups;

  for (auto const& event : *mon_update->events()) {
    if (event->type() == MonitoringEventType_NO_PROBLEM) {
      continue;
    }
    auto const& pg = data.get_passenger_group(event->group()->id());
    auto const localization =
        from_fbs(sched, event->localization_type(), event->localization());
    auto const destination_station_id =
        pg.compact_planned_journey_.destination_station_id();

    auto& destination_groups = combined_groups[destination_station_id];
    auto cpg = std::find_if(
        begin(destination_groups), end(destination_groups),
        [&](auto const& g) { return g.localization_ == localization; });
    if (cpg == end(destination_groups)) {
      destination_groups.emplace_back(combined_passenger_group{
          destination_station_id, pg.passengers_, localization, {&pg}, {}});
    } else {
      cpg->passengers_ += pg.passengers_;
      cpg->groups_.push_back(&pg);
    }
  }

  if (combined_groups.empty()) {
    return;
  }

  auto routing_requests = 0ULL;
  auto alternatives_found = 0ULL;

  {
    scoped_timer alt_timer{"find alternatives"};
    std::vector<ctx::future_ptr<ctx_data, void>> futures;
    for (auto& cgs : combined_groups) {
      auto const destination_station_id = cgs.first;
      for (auto& cpg : cgs.second) {
        ++routing_requests;
        futures.emplace_back(
            spawn_job_void([&sched, destination_station_id, &cpg] {
              cpg.alternatives_ = find_alternatives(
                  sched, destination_station_id, cpg.localization_);
            }));
      }
    }
    ctx::await_all(futures);
  }

  {
    scoped_timer alt_trips_timer{"add alternatives to graph"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        alternatives_found += cpg.alternatives_.size();
        for (auto const& alt : cpg.alternatives_) {
          for (auto const& leg : alt.compact_journey_.legs_) {
            get_or_add_trip(sched, data, leg.trip_);
          }
        }
      }
    }
  }

  LOG(info) << "alternatives: " << routing_requests << " routing requests => "
            << alternatives_found << " alternatives";

  // cap_ILP_edge, cap_ILP_connection, cap_ILP_psg_group aus allen Alternativen
  // von allen Passagieren bauen
  std::map<motis::paxmon::edge*, uint32_t> paxmon_to_cap_ILP_edge;
  {
    scoped_timer alt_timer{"build capacitated model"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        std::cout << "psg is at the station "
                  << cpg.localization_.at_station_->name_ << std::endl;
        for (auto const& alt : cpg.alternatives_) {
          motis::paxmon::event_node* last_node = nullptr;
          for (auto const& [leg_idx, leg] :
               utl::enumerate(alt.compact_journey_.legs_)) {
            auto const td = data.graph_.trip_data_.at(leg.trip_).get();
            auto in_trip = false;
            for (auto [edge_idx, e] : utl::enumerate(td->edges_)) {
              if (e->from_->station_ == leg.enter_station_id_ &&
                  e->from_->time_ == leg.enter_time_) {
                in_trip = true;
                if (last_node != nullptr) {
                  for (auto& oe : last_node->outgoing_edges(data.graph_)) {
                    if (oe->type_ == motis::paxmon::edge_type::INTERCHANGE &&
                        oe->to(data.graph_) == e->from_) {
                      // TODO: add interchange node to the ilp model with
                      // duration:
                      e->transfer_time();
                    }
                  }
                }
              }
              if (in_trip) {
                // TODO: add trip edge to the ilp model using following data
                e->capacity();
                e->passengers_;  // first do grp.edges_ - passengers from
                                 // combined group
                e->type();  // check, that it is trip edge
                e->to_->current_time() -
                    e->from_->current_time();  // duration / cost?
              }
              if (e->to_->station_idx() ==
                  leg.exit_station_id_) {  // is that true? how is it with
                                           // interchange edges? from == to?
                // add also interchange edge to the model:
                last_node = e->to_;
                break;
              }
              // TODO: till now no wait edges considered
            }
          }

          // TODO: check that journey_.stops_ math the build ILP connections
          std::cout << "alternative has " << alt.journey_.trips_.size()
                    << " trips" << std::endl;
          std::cout << "alternative starts at "
                    << alt.journey_.stops_.begin()->name_ << std::endl;
          alt.journey_.trips_[0];
          alt.journey_.attributes_[0].from_;

          // std::cout << "from: " << alt.journey_.attributes_[0].from_
          //          << " to " << alt.journey_.attributes_[0].to_ << std::endl;
        }
      }
    }
  }
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

  // auto psg_assignment = build_ILP_from_scenario_API(psg_groups, config, "1");
}

}  // namespace motis::paxassign
