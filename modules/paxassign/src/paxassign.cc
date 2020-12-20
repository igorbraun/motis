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

#include "motis/paxassign/algorithms_configs.h"
#include "motis/paxassign/build_cap_ILP.h"
#include "motis/paxassign/build_time_exp_graph.h"
#include "motis/paxassign/build_whole_graph_ilp.h"
#include "motis/paxassign/heuristic_algo/greedy.h"
#include "motis/paxassign/heuristic_algo/local_search.h"
#include "motis/paxassign/perceived_tt.h"
#include "motis/paxassign/print_solution.h"
#include "motis/paxassign/service_functions.h"
#include "motis/paxassign/service_time_exp_graph.h"
#include "motis/paxassign/time_expanded_graph.h"

#include "motis/paxassign/build_toy_scenario.h"
#include "../../../build/rel/generated/motis/protocol/Message_generated.h"

using namespace motis::module;
using namespace motis::logging;
using namespace motis::routing;
using namespace motis::paxmon;
using namespace motis::rt;

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
    on_monitor(msg);
    return nullptr;
  });
}

void paxassign::toy_scenario(const motis::module::msg_ptr&) {
  std::cout << "paxassign toyscenario" << std::endl;
  // build_toy_scenario();
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

void paxassign::on_monitor(const motis::module::msg_ptr& msg) {
  auto const& sched = get_schedule();
  auto& data = *get_shared_data<paxmon_data*>(motis::paxmon::DATA_KEY);

  auto const mon_update = motis_content(MonitoringUpdate, msg);

  auto const current_time =
      unix_to_motistime(sched.schedule_begin_, sched.system_time_);
  utl::verify(current_time != INVALID_TIME, "invalid current system time");

  std::map<unsigned, std::vector<combined_pg>> combined_groups;
  std::uint16_t curr_id = 0;

  for (auto const& event : *mon_update->events()) {
    if (event->type() == MonitoringEventType_NO_PROBLEM ||
        event->type() == MonitoringEventType_MAJOR_DELAY_EXPECTED) {
      continue;
    }
    auto const pg = data.get_passenger_group(event->group()->id());
    auto const localization =
        from_fbs(sched, event->localization_type(), event->localization());
    auto const destination_station_id =
        pg->compact_planned_journey_.destination_station_id();

    auto& destination_groups = combined_groups[destination_station_id];
    auto cpg =
        std::find_if(std::begin(destination_groups),
                     std::end(destination_groups), [&](combined_pg const& g) {
                       return g.localization_ == localization &&
                              g.groups_[0]->planned_arrival_time_ ==
                                  pg->planned_arrival_time_;
                     });
    if (cpg == end(destination_groups)) {
      destination_groups.emplace_back(combined_pg{curr_id++,
                                                  destination_station_id,
                                                  pg->passengers_,
                                                  localization,
                                                  {pg},
                                                  {}});
    } else {
      cpg->passengers_ += pg->passengers_;
      cpg->groups_.push_back(pg);
    }
  }

  std::cout << "size of comb groups: " << combined_groups.size() << std::endl;
  std::ofstream results_file("comparison.csv");
  results_file << "ID,Halle_obj,NA_obj\n";
  uint16_t curr_scenario_id = 0;
  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      /*
      bool contains_needed_group = false;
      for (auto const& grp : cpg.groups_) {
        if (grp->id_ == 35042) {  // grp->id_ == 83364 || grp->id_ == 125658 ||
          contains_needed_group = true;
        }
      }

      if (contains_needed_group) {
        std::cout
            << sched.stations_[cgs.first]->name_ << " to "
            << sched.stations_[cpg.localization_.at_station_->index_]->name_
            << ", psgrs: " << cpg.passengers_ << std::endl;

        std::map<unsigned, std::vector<combined_pg>> selected_combined_groups;
        auto& g = selected_combined_groups[cgs.first];
        g.push_back(combined_pg{cpg});
        node_arc_ilp_assignment(selected_combined_groups, data, sched);
      }
      */
      std::map<unsigned, std::vector<combined_pg>> selected_combined_groups;
      auto& g = selected_combined_groups[cgs.first];
      g.push_back(combined_pg{cpg});
      results_file << curr_scenario_id++ << ",";
      node_arc_ilp_assignment(selected_combined_groups, data, sched,
                              results_file);
    }
  }
  results_file.close();

  // throw std::runtime_error("the end");

  /*
  std::uint16_t needed_group_idx = 0;
  for (auto j = 0u; j < combined_groups[65019].size(); ++j) {
    if (combined_groups[65019][j].groups_[0]->planned_arrival_time_ == 7662) {
      needed_group_idx = j;
    }
  }

  std::map<unsigned, std::vector<combined_pg>> selected_combined_groups;
  auto& g = selected_combined_groups[65019];
  g.push_back(combined_pg{combined_groups[65019][needed_group_idx]});
  */

  if (combined_groups.empty()) {
    return;
  }

  std::map<std::string, std::tuple<double, double, double, double>>
      variables_with_values;
  // cap_ilp_assignment(combined_groups, data, sched, variables_with_values);
  // node_arc_ilp_assignment(combined_groups, data, sched);
  // heuristic_assignments(combined_groups, data, sched);
}

std::vector<std::pair<ilp_psg_id, alt_idx>> paxassign::cap_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched,
    std::map<std::string, std::tuple<double, double, double, double>>&
        variables_with_values,
    std::ofstream& results_file) {
  uint16_t psgs_in_sc = 0;
  for (auto& cgs : combined_groups) {
    psgs_in_sc += cgs.second.size();
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
              cpg.alternatives_ = motis::paxforecast::find_alternatives(
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
        for (auto const& grp : cpg.groups_) {
          std::cout << "paxmon::passenger_group.id_ : " << grp->id_
                    << ", source primary ref: " << grp->source_.primary_ref_
                    << ", source secondary ref: " << grp->source_.secondary_ref_
                    << ", planned arrival time: " << grp->planned_arrival_time_
                    << std::endl;
        }
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
            std::cout << "cpg.id_: " << cpg.id_ << ", planned arrival time: "
                      << cpg.groups_.back()->planned_arrival_time_ << " vs "
                      << cpg.alternatives_[curr_alt_ind]
                             .compact_journey_.legs_.back()
                             .exit_time_
                      << ", difference: "
                      << cpg.alternatives_[curr_alt_ind]
                                 .compact_journey_.legs_.back()
                                 .exit_time_ -
                             cpg.groups_.back()->planned_arrival_time_
                      << std::endl;
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
              // TODO. interchange edge cost here and in the node-arc graph
              // should be equivalent. Check it
              // auto const transfer_time =
              // get_transfer_duration(leg.enter_transfer_);
              auto const inch_e_start_idx = find_edge_idx(td, leg, true);
              add_interchange_edge(last_node,
                                   td->edges_[inch_e_start_idx]->from_, 0,
                                   data.graph_);
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

  std::map<std::uint16_t, combined_pg*> cpg_id_to_group;
  uint32_t curr_e_id = 1;
  uint32_t curr_alt_id = 1;
  std::map<motis::paxmon::edge*, motis::paxassign::cap_ILP_edge> cap_edges;
  std::map<uint32_t, motis::paxassign::cap_ILP_edge> train_entry_cap_edges;

  perceived_tt_config perc_tt_config{};
  cap_ILP_edge no_route_edge{nullptr,
                             nullptr,
                             curr_e_id++,
                             perc_tt_config.no_route_cost_,
                             std::numeric_limits<std::uint64_t>::max(),
                             std::numeric_limits<std::uint64_t>::max(),
                             0,
                             edge_type::NOROUTE,
                             nullptr};

  std::vector<cap_ILP_psg_group> cap_ILP_scenario;
  {
    scoped_timer alt_timer{"build data for capacitated model"};
    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        std::vector<cap_ILP_connection> cpg_ILP_connections;
        for (auto const& alt : cpg.alternatives_) {
          cap_ILP_connection curr_connection{curr_alt_id++, 0,
                                             std::vector<cap_ILP_edge*>{}};
          uint32_t associated_waiting_time = 0;

          if (!cpg.localization_.in_trip() ||
              (cpg.localization_.in_trip_ !=
               alt.compact_journey_.legs_.begin()->trip_)) {
            auto const td = data.graph_.trip_data_
                                .at(alt.compact_journey_.legs_.begin()->trip_)
                                .get();
            auto first_node_idx =
                find_edge_idx(td, *alt.compact_journey_.legs_.begin(), true);
            train_entry_cap_edges[curr_e_id] =
                cap_ILP_edge{td->edges_[first_node_idx]->from_,
                             td->edges_[first_node_idx]->from_,
                             curr_e_id,
                             0,
                             std::numeric_limits<std::uint64_t>::max(),
                             std::numeric_limits<std::uint64_t>::max(),
                             0,
                             edge_type::INTERCHANGE,
                             nullptr};
            curr_connection.edges_.push_back(
                &train_entry_cap_edges[curr_e_id++]);
          }

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
                     cpg.localization_.current_arrival_time_);
              }

              std::uint64_t last_trip_edge_cap =
                  td->edges_[i]->has_capacity()
                      ? td->edges_[i]->capacity()
                      : std::numeric_limits<std::uint64_t>::max();

              if (cap_edges.find(td->edges_[i]) == cap_edges.end()) {
                cap_edges[td->edges_[i]] = cap_ILP_edge{
                    td->edges_[i]->from_,
                    td->edges_[i]->to_,
                    curr_e_id++,
                    static_cast<uint32_t>(td->edges_[i]->to_->current_time() -
                                          td->edges_[i]->from_->current_time()),
                    td->edges_[i]->has_capacity()
                        ? td->edges_[i]->capacity()
                        : std::numeric_limits<std::uint64_t>::max(),
                    td->edges_[i]->has_capacity()
                        ? static_cast<uint64_t>(
                              td->edges_[i]->capacity() *
                              perc_tt_config.cost_function_capacity_steps_
                                  .back())
                        : std::numeric_limits<std::uint64_t>::max(),
                    td->edges_[i]->passengers(),
                    edge_type::TRIP,
                    td->edges_[i]->get_trip(sched)};
              }
              curr_connection.edges_.push_back(&cap_edges[td->edges_[i]]);

              // ILP WAIT EDGES
              if (i < last_edge_idx) {
                for (auto const& oe :
                     td->edges_[i]->to_->outgoing_edges(data.graph_)) {
                  if (oe->type_ == motis::paxmon::edge_type::WAIT &&
                      oe->to_ == td->edges_[i + 1]->from_) {
                    if (cap_edges.find(oe.get()) == cap_edges.end()) {
                      cap_edges[oe.get()] = cap_ILP_edge{
                          oe->from_,
                          oe->to_,
                          curr_e_id++,
                          static_cast<uint32_t>(oe->to_->time_ -
                                                oe->from_->time_),
                          last_trip_edge_cap,
                          (last_trip_edge_cap ==
                           std::numeric_limits<std::uint64_t>::max())
                              ? std::numeric_limits<std::uint64_t>::max()
                              : static_cast<uint64_t>(
                                    last_trip_edge_cap *
                                    perc_tt_config.cost_function_capacity_steps_
                                        .back()),
                          oe->passengers(),
                          edge_type::WAIT,
                          oe->get_trip(sched)};
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
                        cap_ILP_edge{oe->from_,
                                     oe->to_,
                                     curr_e_id++,
                                     0,
                                     std::numeric_limits<std::uint64_t>::max(),
                                     std::numeric_limits<std::uint64_t>::max(),
                                     0,
                                     edge_type::INTERCHANGE,
                                     nullptr};
                  }
                  curr_connection.edges_.push_back(&cap_edges.at(oe.get()));
                  associated_waiting_time += oe->to_->time_ - oe->from_->time_;
                }
              }
            }
          }
          curr_connection.associated_waiting_time_ = associated_waiting_time;
          cpg_ILP_connections.push_back(curr_connection);
        }
        cpg_ILP_connections.push_back(
            cap_ILP_connection{curr_alt_id++, 0, {&no_route_edge}});
        cap_ILP_scenario.push_back(
            cap_ILP_psg_group{cpg.id_, cpg_ILP_connections, cpg.passengers_});
        cpg_id_to_group[cpg.id_] = &cpg;
      }
    }
  }

  cap_ILP_solution sol;
  {
    scoped_timer alt_timer{"solve capacitated model"};
    std::srand(std::time(nullptr));
    int random_variable = std::rand();
    sol = build_ILP_from_scenario_API(cap_ILP_scenario, perc_tt_config,
                                      std::to_string(cap_ILP_scenario.size()) +
                                          "_" + std::to_string(random_variable),
                                      variables_with_values, results_file);
  }

  auto final_obj = piecewise_linear_convex_perceived_tt_halle(
      cap_ILP_scenario, sol.alt_to_use_, perc_tt_config, variables_with_values);
  std::cout << "manually calculated perc_tt of halle ILP : " << final_obj
            << std::endl;

  // print_solution_routes_mini_halle(cpg_id_to_group, sol.alt_to_use_, sched);

  print_solution_routes_halle(cap_ILP_scenario, sol.alt_to_use_, sched);

  // std::cout << "HALLE APPROACH. Passengers in scenario INPUT : " <<
  // psgs_in_sc
  //          << ", OUTPUT assignments : " << sol.alt_to_use_.size() <<
  //          std::endl;

  /*
  std::cout << " ------------------------------ HALLE EDGE ANALYSIS: "
               "------------------------------ "
            << std::endl;
  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      std::cout << "SELECTED ALTERNATIVE FOR " << cpg.id_
                << " with psgs: " << cpg.passengers_ << std::endl;
      auto asg =
          std::find_if(sol.alt_to_use_.begin(), sol.alt_to_use_.end(),
                       [&cpg](std::pair<std::uint16_t, std::uint16_t> p) {
                         return p.first == cpg.id_;
                       });
      if (cpg.alternatives_.size() == asg->second) {
        std::cout << "NO ROUTE" << std::endl;
        continue;
      }
      for (auto const& l :
           cpg.alternatives_[asg->second].compact_journey_.legs_) {
        auto tr_data = data.graph_.trip_data_.find(l.trip_);
        auto entry_edge = std::find_if(
            tr_data->second->edges_.begin(), tr_data->second->edges_.end(),
            [&l](motis::paxmon::edge const* e) {
              return e->from_->station_ == l.enter_station_id_;
            });
        auto exit_edge = std::find_if(
            tr_data->second->edges_.begin(), tr_data->second->edges_.end(),
            [&l](motis::paxmon::edge const* e) {
              return e->to_->station_ == l.exit_station_id_;
            });
        std::cout << "train: " << l.trip_->id_.primary_.train_nr_ << std::endl;
        for (auto it = entry_edge;; ++it) {
          auto cap = (*it)->has_capacity()
                         ? (*it)->capacity()
                         : std::numeric_limits<std::uint64_t>::max();
          std::cout << "from " << sched.stations_[(*it)->from_->station_]->name_
                    << " to " << sched.stations_[(*it)->to_->station_]->name_
                    << ", " << (*it)->passengers() << " / " << cap << std::endl;
          if (it == exit_edge) break;
        }
      }
    }
  }
  */
  return sol.alt_to_use_;

  // throw std::runtime_error("time expanded graph is built");

  /* TODO: for future evaluation
  std::ofstream stats_file;
  stats_file.open("motis/build/rel/ilp_files/ILP_stats.csv",
                  std::ios_base::app);
  stats_file << sol.stats_.num_groups_ << "," << sol.stats_.run_time_ << ","
             << sol.stats_.num_vars_ << "," << sol.stats_.num_constraints_
             << "," << sol.stats_.no_alt_found_ << "," << sol.stats_.obj_
             << std::endl;
  stats_file.close();
   */

  /* TODO: now it returns a compact journey as response, but since it will be
  done differently in other algorithms, I suppose it will also be changed
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
*/
}

void paxassign::node_arc_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched, std::ofstream& results_file) {
  std::map<std::string, std::tuple<double, double, double, double>>
      variables_with_values_halle;

  auto alts_to_use = cap_ilp_assignment(
      combined_groups, data, sched, variables_with_values_halle, results_file);

  uint16_t psgs_in_sc = 0;
  for (auto& cgs : combined_groups) {
    psgs_in_sc += cgs.second.size();
  }

  node_arc_config na_config{1.2, 30, 6, 10000};
  perceived_tt_config perc_tt_config;
  auto te_graph = build_time_expanded_graph(data, sched, na_config);
  std::cout << "NODES IN GRAPH : " << te_graph.nodes_.size() << std::endl;
  uint32_t nodes_count = 0;
  for (auto const& n : te_graph.nodes_) {
    nodes_count += n->out_edges_.size();
  }
  std::cout << "EDGES IN GRAPH : " << nodes_count << std::endl;

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);

  std::map<std::string, std::tuple<double, double, double, double>>
      variables_with_values_node_arc;
  auto solution =
      node_arc_ilp(eg_psg_groups, te_graph, na_config, perc_tt_config, sched,
                   variables_with_values_node_arc, results_file);

  double final_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, solution, perc_tt_config);
  std::cout << "manually NODE-ARC ILP CUMULATIVE: " << final_obj << std::endl;
  print_solution_routes_node_arc(solution, eg_psg_groups, sched, te_graph);

  std::cout << " ------------------------------ NODE ARC EDGE ANALYSIS: "
               "------------------------------ "
            << std::endl;
  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      // if (cpg.id_ != 3) continue;
      std::cout << "SELECTED ALTERNATIVE FOR " << cpg.id_
                << " with psgs: " << cpg.passengers_ << ", planned arr time: "
                << cpg.groups_.back()->planned_arrival_time_ << std::endl;
      auto asg =
          std::find_if(alts_to_use.begin(), alts_to_use.end(),
                       [&cpg](std::pair<std::uint16_t, std::uint16_t> p) {
                         return p.first == cpg.id_;
                       });

      if (cpg.alternatives_.size() == asg->second) {
        std::cout << "NO ROUTE" << std::endl;
        continue;
      }

      auto eg_psg_g = std::find_if(
          eg_psg_groups.begin(), eg_psg_groups.end(),
          [&](eg_psg_group const& pg) { return pg.cpg_.id_ == cpg.id_; });
      config_graph_reduction reduction_config;
      std::vector<bool> nodes_validity =
          reduce_te_graph((*eg_psg_g), te_graph, reduction_config, sched);

      for (auto const& alt : cpg.alternatives_) {
        std::cout << "NEW ALTERNATIVE" << std::endl;
        eg_event_node* from;
        eg_event_node* to;
        for (auto const& l : alt.compact_journey_.legs_) {
          auto tr_data =
              te_graph.trip_data_.find(to_extern_trip(sched, l.trip_));
          auto entry_edge =
              std::find_if(tr_data->second->edges_.begin(),
                           tr_data->second->edges_.end(), [&l](eg_edge* e) {
                             return e->from_->station_ == l.enter_station_id_;
                           });
          if (from == nullptr) {
            from = (*entry_edge)->from_;
          }
          auto exit_edge =
              std::find_if(tr_data->second->edges_.begin(),
                           tr_data->second->edges_.end(), [&l](eg_edge* e) {
                             return e->to_->station_ == l.exit_station_id_;
                           });
          to = (*exit_edge)->to_;
          std::cout << "train: " << l.trip_->id_.primary_.train_nr_
                    << std::endl;
          for (auto it = entry_edge;; ++it) {
            std::cout << "from " << (*it)->from_->station_ << " == "
                      << sched.stations_[(*it)->from_->station_]->name_ << " ("
                      << nodes_validity[(*it)->from_->id_]
                      << ") time: " << (*it)->from_->time_ << " to "
                      << (*it)->to_->station_
                      << " == " << sched.stations_[(*it)->to_->station_]->name_
                      << " (" << nodes_validity[(*it)->to_->id_]
                      << ") time: " << (*it)->to_->time_
                      << ", psgrs: " << (*it)->passengers_ << " / "
                      << (*it)->soft_cap_boundary_ << std::endl;
            if (it == exit_edge) break;
          }
        }
      }
      /*
            std::cout << "Before dij" << std::endl;
            auto solut = sssd_dijkstra<double>(
                (*eg_psg_g).from_, (*eg_psg_g).to_, (*eg_psg_g).psg_count_, 0.0,
                std::numeric_limits<double>::max(), te_graph, nodes_validity, 6,
                calc_perc_tt_dist);
            std::cout << "GREEDY SOLUTION" << std::endl;
            print_solution_routes_node_arc(std::vector<std::vector<eg_edge*>>{solut},
                                           eg_psg_groups, sched, te_graph);
            */
    }
  }

  /*
  std::cout << "NODE-ARC APPROACH. Passengers in scenario INPUT : "
            << psgs_in_sc << ", OUTPUT assignments : " << solution.size()
            << std::endl;

    std::cout << "HALLE APPROACH: " << std::endl;
    for (auto const& var_halle : variables_with_values_halle) {
      std::cout << var_halle.first << ": val: " << std::get<0>(var_halle.second)
                << ", obj coef: " << std::get<1>(var_halle.second)
                << ", obj: " << std::get<2>(var_halle.second)
                << ", ub: " << std::get<3>(var_halle.second) << std::endl;
    }
    std::cout << "NODE-ARC APPROACH: " << std::endl;
    for (auto const& var_halle : variables_with_values_node_arc) {
      std::cout << var_halle.first << ": val: " << std::get<0>(var_halle.second)
                << ", obj coef: " << std::get<1>(var_halle.second)
                << ", obj: " << std::get<2>(var_halle.second)
                << ", ub: " << std::get<3>(var_halle.second) << std::endl;
    }
    */

  // throw std::runtime_error("time expanded graph is built");
}

void paxassign::heuristic_assignments(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {

  perceived_tt_config perc_tt_config;
  node_arc_config eg_config{1.2, 30, 6, 10000};

  auto te_graph = build_time_expanded_graph(data, sched, eg_config);
  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, eg_config, te_graph);

  // TODO: for subset-scenario (only long dist stations) nodes validity has the
  // size of 3,53 mb per psg. If during evaluation it will be too big, remove
  // nodes validity at all and do all the stuff without it
  config_graph_reduction reduction_config;
  std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    nodes_validity[i] =
        reduce_te_graph(eg_psg_groups[i], te_graph, reduction_config, sched);
  }

  // perceived tt for start solution
  auto calc_perc_tt_dist = [&](eg_edge* e, double curr_dist) {
    if (e->capacity_utilization_ >
        perc_tt_config.cost_function_capacity_steps_.back()) {
      return (double)perc_tt_config.no_route_cost_ + curr_dist;
    }
    double transfer_penalty = (e->type_ == eg_edge_type::TRAIN_ENTRY)
                                  ? perc_tt_config.transfer_penalty_
                                  : 0.0;
    auto const it =
        std::lower_bound(perc_tt_config.cost_function_capacity_steps_.begin(),
                         perc_tt_config.cost_function_capacity_steps_.end(),
                         e->capacity_utilization_);
    auto idx =
        std::distance(perc_tt_config.cost_function_capacity_steps_.begin(), it);
    return perc_tt_config.tt_and_waiting_penalties_[idx] * e->cost_ +
           transfer_penalty + curr_dist;
  };

  // shortest tt as in the Halle-paper
  auto calc_tt_dist = [&](eg_edge* e, double curr_dist) {
    if (e->capacity_utilization_ >
        perc_tt_config.cost_function_capacity_steps_.back()) {
      return (double)perc_tt_config.no_route_cost_ + curr_dist;
    }
    return e->cost_ + curr_dist;
  };

  auto rng = std::mt19937{};

  // Start solution for local search. calc_perc_tt_dist is used
  auto greedy_solution = greedy_assignment(
      te_graph, nodes_validity, eg_config.max_allowed_interchanges_,
      eg_psg_groups, rng, calc_perc_tt_dist);

  /*
  for (auto const& sol : greedy_solution) {
    std::cout << calc_perc_tt(sol, perc_tt_config) << std::endl;
  }
  */
  std::cout << std::fixed << "GREEDY CUMULATIVE "
            << get_obj_after_assign(eg_psg_groups, greedy_solution,
                                    perc_tt_config)
            << std::endl;
  print_solution_statistics(greedy_solution, eg_psg_groups, sched);

  {
    scoped_timer alt_timer{"LOCAL SEARCH"};
    auto ls_solution = local_search(
        eg_psg_groups, greedy_solution, perc_tt_config, 3, rng, te_graph,
        nodes_validity, eg_config.max_allowed_interchanges_, calc_perc_tt_dist);
    /*
        for (auto const& sol : ls_solution) {
          std::cout << calc_perc_tt(sol, perc_tt_config) << std::endl;
        }
    */
    std::cout << std::fixed << "LS CUMULATIVE "
              << get_obj_after_assign(eg_psg_groups, ls_solution,
                                      perc_tt_config)
              << std::endl;
  }

  throw std::runtime_error("heuristic algorithms finished");
}

void paxassign::on_forecast(const motis::module::msg_ptr&) {

  /*
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
*/
}
}  // namespace motis::paxassign
