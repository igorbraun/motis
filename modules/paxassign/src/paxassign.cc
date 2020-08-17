#include "motis/paxassign/paxassign.h"

#include <iosfwd>
#include <iostream>

#include "motis/core/common/date_time_util.h"
#include "motis/core/common/logging.h"
#include "motis/core/access/service_access.h"
#include "motis/core/access/station_access.h"
#include "motis/module/context/get_schedule.h"
#include "motis/module/context/motis_call.h"
#include "motis/module/context/motis_publish.h"
#include "motis/module/context/motis_spawn.h"
#include "motis/module/message.h"
#include "motis/paxmon/messages.h"

#include "motis/paxforecast/messages.h"

#include "motis/paxmon/build_graph.h"
#include "motis/paxmon/data_key.h"
#include "motis/paxmon/graph_access.h"
#include "motis/paxmon/loader/journeys/to_compact_journey.h"
#include "motis/paxmon/messages.h"

#include "motis/paxassign/build_cap_ILP.h"
#include "motis/paxassign/build_time_exp_graph.h"
#include "motis/paxassign/build_whole_graph_ilp.h"
#include "motis/paxassign/time_expanded_graph.h"

#include "motis/paxassign/build_toy_scenario.h"
#include "../../../build/rel/generated/motis/protocol/Message_generated.h"

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
    // whole_graph_ilp_assignment(msg);
    on_monitor(msg);
    return nullptr;
  });
}

void paxassign::toy_scenario(const motis::module::msg_ptr&) {
  std::cout << "paxassign toyscenario" << std::endl;
  build_toy_scenario();
}

uint32_t find_edge_idx(trip_data const* td,
                       motis::paxmon::journey_leg const& leg, bool const from) {
  auto result_edge =
      std::find_if(begin(td->edges_), end(td->edges_), [&](auto const& e) {
        if (from) {
          return e->from_->station_ == leg.enter_station_id_;
        } else {
          return e->to_->station_ == leg.exit_station_id_;
        }
      });
  if (result_edge != end(td->edges_)) {
    return distance(begin(td->edges_), result_edge);
  } else {
    return std::numeric_limits<uint32_t>::max();
  }
}

inline duration get_transfer_duration(std::optional<transfer_info> const& ti) {
  return ti.has_value() ? ti.value().duration_ : 0;
}

void paxassign::on_monitor(const motis::module::msg_ptr& msg) {
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

  // cap_ilp_assignment(combined_groups, data, sched);
  whole_graph_ilp_assignment(combined_groups, data, sched);
}

void paxassign::cap_ilp_assignment(
    std::map<unsigned, std::vector<combined_passenger_group>>& combined_groups,
    paxmon_data& data, schedule const& sched) {

  {
    scoped_timer withdraw_pg_assignments{"withdraw pg assignments"};
    for (auto const& same_dest_gr : combined_groups) {
      for (auto const& cpg : same_dest_gr.second) {
        for (auto const& g : cpg.groups_) {
          for (auto& e : g->edges_) {
            if (e->passengers_ < g->passengers_) {
              throw std::runtime_error("edge psgs less than group psgs");
            }
            e->passengers_ -= g->passengers_;
          }
        }
      }
    }
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
    scoped_timer no_alt_timer{"remove cpg with no alternatives"};
    for (auto& cgs : combined_groups) {
      uint32_t cpg_idx = 0;
      while (cpg_idx < cgs.second.size()) {
        if (cgs.second[cpg_idx].alternatives_.empty()) {
          cgs.second.erase(cgs.second.begin() + cpg_idx);
        } else {
          ++cpg_idx;
        }
      }
    }
  }

  {
    scoped_timer alt_trips_timer{"add alternatives to graph"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        size_t curr_alt_ind = 0;
        while (curr_alt_ind < cpg.alternatives_.size()) {
          bool remove_alt = false;
          for (auto const [leg_idx, leg] : utl::enumerate(
                   cpg.alternatives_[curr_alt_ind].compact_journey_.legs_)) {
            auto td = get_or_add_trip(sched, data, leg.trip_);
            auto const leg_first_edge_idx = find_edge_idx(td, leg, true);
            auto const leg_last_edge_idx = find_edge_idx(td, leg, false);
            if (leg_first_edge_idx == std::numeric_limits<uint32_t>::max() ||
                leg_last_edge_idx == std::numeric_limits<uint32_t>::max()) {
              remove_alt = true;
              break;
            }
          }
          if (remove_alt) {
            cpg.alternatives_.erase(cpg.alternatives_.begin() + curr_alt_ind);
          } else {
            ++curr_alt_ind;
          }
        }
      }
    }
  }

  {
    scoped_timer alt_inchs_timer{"add interchanges to graph"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        alternatives_found += cpg.alternatives_.size();
        for (auto const& alt : cpg.alternatives_) {
          event_node* last_node = nullptr;
          for (auto const& leg : alt.compact_journey_.legs_) {
            auto td = get_or_add_trip(sched, data, leg.trip_);
            if (last_node != nullptr) {
              auto const transfer_time =
                  get_transfer_duration(leg.enter_transfer_);
              auto const inch_e_start_idx = find_edge_idx(td, leg, true);
              add_interchange_edge(last_node,
                                   td->edges_[inch_e_start_idx]->from_,
                                   transfer_time, data.graph_);
            }
            auto const leg_last_st_idx = find_edge_idx(td, leg, false);
            last_node = td->edges_[leg_last_st_idx]->to_;
          }
        }
      }
    }
  }

  LOG(info) << "alternatives: " << routing_requests << " routing requests => "
            << alternatives_found << " alternatives";

  std::map<uint32_t, combined_passenger_group*> cap_ilp_psg_to_cpg;
  uint32_t curr_cpg_id = 1;
  uint32_t curr_e_id = 1;
  uint32_t curr_alt_id = 1;
  std::map<motis::paxmon::edge*, motis::paxassign::cap_ILP_edge> cap_edges;
  cap_ILP_edge no_route_edge{curr_e_id++, 100000, 100000, edge_type::NOROUTE};

  std::vector<cap_ILP_psg_group> cap_ILP_scenario;
  cap_ILP_config ILP_config{};
  {
    scoped_timer alt_timer{"build data for capacitated model"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        std::vector<cap_ILP_connection> cpg_ILP_connections;
        for (auto const& alt : cpg.alternatives_) {
          cap_ILP_connection curr_connection{curr_alt_id++, 0,
                                             std::vector<cap_ILP_edge*>{}};
          uint32_t associated_waiting_time = 0;
          for (auto const [leg_idx, leg] :
               utl::enumerate(alt.compact_journey_.legs_)) {
            auto const td = data.graph_.trip_data_.at(leg.trip_).get();

            auto start_edge_idx = find_edge_idx(td, leg, true);
            auto last_edge_idx = find_edge_idx(td, leg, false);

            // ILP TRIP EDGES
            for (auto i = start_edge_idx; i <= last_edge_idx; ++i) {
              if (leg_idx == 0 && i == start_edge_idx) {
                associated_waiting_time +=
                    (td->edges_[i]->from_->current_time() -
                     cpg.localization_.arrival_time_);
              }

              if (cap_edges.find(td->edges_[i]) == cap_edges.end()) {
                uint32_t remaining_cap =
                    (td->edges_[i]->capacity() < td->edges_[i]->passengers_)
                        ? 0
                        : td->edges_[i]->capacity() -
                              td->edges_[i]->passengers_;
                cap_edges[td->edges_[i]] = cap_ILP_edge{
                    curr_e_id++,
                    static_cast<uint32_t>(td->edges_[i]->to_->current_time() -
                                          td->edges_[i]->from_->current_time()),
                    remaining_cap, edge_type::TRIP};
              }
              curr_connection.edges_.push_back(&cap_edges[td->edges_[i]]);

              // ILP WAIT EDGES
              if (i < last_edge_idx) {
                for (auto const& oe :
                     td->edges_[i]->to_->outgoing_edges(data.graph_)) {
                  if (oe->type_ == motis::paxmon::edge_type::WAIT &&
                      oe->to_ == td->edges_[i + 1]->from_) {
                    if (cap_edges.find(oe.get()) == cap_edges.end()) {
                      uint32_t remaining_cap =
                          (oe->capacity() < oe->passengers_)
                              ? 0
                              : oe->capacity() - oe->passengers_;
                      cap_edges[oe.get()] = cap_ILP_edge{
                          curr_e_id++,
                          static_cast<uint32_t>(
                              td->edges_[i + 1]->from_->current_time() -
                              td->edges_[i]->to_->current_time()),
                          remaining_cap, edge_type::WAIT};
                    }
                    curr_connection.edges_.push_back(&cap_edges.at(oe.get()));
                  }
                }
              }
            }

            // ILP INTERCHANGE EDGES
            if (leg_idx < alt.compact_journey_.legs_.size() - 1) {
              auto const td_next =
                  data.graph_.trip_data_
                      .at(alt.compact_journey_.legs_[leg_idx + 1].trip_)
                      .get();
              auto inch_target_st_idx = find_edge_idx(
                  td_next, alt.compact_journey_.legs_[leg_idx + 1], true);
              for (auto const& oe :
                   td->edges_[last_edge_idx]->to_->outgoing_edges(
                       data.graph_)) {
                if (oe->type_ == motis::paxmon::edge_type::INTERCHANGE &&
                    oe->to_ == td_next->edges_[inch_target_st_idx]->from_) {
                  if (cap_edges.find(oe.get()) == cap_edges.end()) {
                    cap_edges[oe.get()] =
                        cap_ILP_edge{curr_e_id++, oe->transfer_time(), 100000,
                                     edge_type::INTERCHANGE};
                  }
                  curr_connection.edges_.push_back(&cap_edges.at(oe.get()));
                  associated_waiting_time +=
                      (td_next->edges_[inch_target_st_idx]
                           ->from_->current_time() -
                       td->edges_[last_edge_idx]->to_->current_time() -
                       oe->transfer_time());
                }
              }
            }
          }
          curr_connection.associated_waiting_time_ = associated_waiting_time;
          cpg_ILP_connections.push_back(curr_connection);
        }
        cpg_ILP_connections.push_back(cap_ILP_connection{
            curr_alt_id++, ILP_config.no_route_cost_, {&no_route_edge}});
        cap_ilp_psg_to_cpg[curr_cpg_id] = &cpg;
        cap_ILP_scenario.push_back(cap_ILP_psg_group{
            curr_cpg_id++, cpg_ILP_connections, cpg.passengers_});
      }
    }
  }

  cap_ILP_solution sol;
  {
    scoped_timer alt_timer{"solve capacitated model"};
    std::srand(std::time(nullptr));
    int random_variable = std::rand();
    sol =
        build_ILP_from_scenario_API(cap_ILP_scenario, ILP_config,
                                    std::to_string(cap_ILP_scenario.size()) +
                                        "_" + std::to_string(random_variable));
  }

  std::ofstream stats_file;
  stats_file.open("motis/build/rel/ilp_files/ILP_stats.csv",
                  std::ios_base::app);
  stats_file << sol.stats_.num_groups_ << "," << sol.stats_.run_time_ << ","
             << sol.stats_.num_vars_ << "," << sol.stats_.num_constraints_
             << "," << sol.stats_.no_alt_found_ << "," << sol.stats_.obj_
             << std::endl;
  stats_file.close();

  {
    scoped_timer pg_assignments{"assign psgs back to edges"};
    for (auto const& same_dest_gr : combined_groups) {
      for (auto const& cpg : same_dest_gr.second) {
        for (auto const& g : cpg.groups_) {
          for (auto& e : g->edges_) {
            e->passengers_ += g->passengers_;
          }
        }
      }
    }
  }

  message_creator mc;
  std::vector<flatbuffers::Offset<ConnAssignment>> fbs_assignments;

  for (auto& assignment : sol.alt_to_use_) {
    if (cap_ilp_psg_to_cpg[assignment.first]->alternatives_.size() <=
        assignment.second) {
      fbs_assignments.emplace_back(CreateConnAssignment(
          mc, assignment.first, to_fbs(sched, mc, compact_journey{})));
    } else {
      auto cj = motis::paxmon::to_compact_journey(
          cap_ilp_psg_to_cpg[assignment.first]
              ->alternatives_[assignment.second]
              .journey_,
          sched);
      fbs_assignments.emplace_back(
          CreateConnAssignment(mc, assignment.first, to_fbs(sched, mc, cj)));
    }
  }

  using namespace motis;
  mc.create_and_finish(
      MsgContent_ConnAssignments,
      CreateConnAssignments(mc, mc.CreateVector(fbs_assignments)).Union(),
      "/paxassign/ilp_result");
  ctx::await_all(motis_publish(make_msg(mc)));
}

void paxassign::whole_graph_ilp_assignment(
    std::map<unsigned, std::vector<combined_passenger_group>>& combined_groups,
    paxmon_data& data, schedule const& sched) {
  auto te_graph = build_time_expanded_graph(data, sched);

  std::for_each(te_graph.nodes_.begin(), te_graph.nodes_.end(),
                [](std::unique_ptr<eg_event_node>& n) {
                  for (auto const e : n->in_edges_) {
                    if (e->type_ == eg_edge_type::INTERCHANGE) {
                      std::cout << "inch edge from " << e->from_->type_
                                << " to " << e->to_->type_ << ", from time "
                                << e->from_->time_ << " to time "
                                << e->to_->time_
                                << ", transfer time: " << e->transfer_time_
                                << std::endl;
                    }
                  }
                });

  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      // TODO: handle also passengers, which are not in trip yet
      if (cpg.localization_.in_trip()) {
        eg_event_node* at_ev_node = nullptr;
        auto tr_data = te_graph.trip_data_.find(
            to_extern_trip(sched, cpg.localization_.in_trip_));
        if (tr_data != te_graph.trip_data_.end()) {
        }
        auto at_edge = std::find_if(
            std::begin(tr_data->second->edges_),
            std::end(tr_data->second->edges_), [&cpg](eg_edge* e_ptr) {
              return e_ptr->to_->station_ ==
                         cpg.localization_.at_station_->index_ &&
                     cpg.localization_.arrival_time_ == e_ptr->to_->time_;
            });
        if (at_edge != tr_data->second->edges_.end()) {
          at_ev_node = (*at_edge)->to_;
        }

        if (at_ev_node != nullptr) {
          auto target_node = te_graph.nodes_
                                 .emplace_back(std::make_unique<eg_event_node>(
                                     eg_event_node{INVALID_TIME,
                                                   event_type::ARR,
                                                   cpg.destination_station_id_,
                                                   {},
                                                   {},
                                                   te_graph.nodes_.size()}))
                                 .get();

          for (auto& n : te_graph.nodes_) {
            if (n.get() != target_node &&
                n->station_ == cpg.destination_station_id_ &&
                n->type_ == event_type::ARR) {
              motis::paxassign::add_interchange(n.get(), target_node, 0,
                                                te_graph);
            }
          }

          build_whole_graph_ilp(at_ev_node, target_node, te_graph);
        }
      }
    }
  }

  std::cout << "Edges after interchanges to dummies: " << std::endl;
  std::cout << std::accumulate(te_graph.nodes_.begin(), te_graph.nodes_.end(),
                               0.0,
                               [](double sum, auto& n) {
                                 return sum + n->in_edges_.size();
                               })
            << std::endl;

  throw std::runtime_error("time expanded graph is built");
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
