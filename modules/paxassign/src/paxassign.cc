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
#include "motis/paxassign/edges_load_service.h"
#include "motis/paxassign/heuristic_algo/greedy.h"
#include "motis/paxassign/heuristic_algo/local_search.h"
#include "motis/paxassign/perceived_tt.h"
#include "motis/paxassign/print_solution.h"
#include "motis/paxassign/service_functions.h"
#include "motis/paxassign/service_time_exp_graph.h"
#include "motis/paxassign/solution_to_compact_journey.h"
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
  reg.subscribe("/paxforecast/toy_scenario", [&](msg_ptr const& msg) {
    toy_scenario(msg);
    return nullptr;
  });

  reg.register_op("/paxassign/monitoring_update", [&](msg_ptr const& msg) {
    on_monitor(msg);
    return nullptr;
  });
}

void paxassign::toy_scenario(const motis::module::msg_ptr&) {
  std::cout << "paxassign toyscenario" << std::endl;
  // build_toy_scenario();
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

  int group_size = 0;
  for (auto const& cg : combined_groups) {
    group_size += cg.second.size();
  }
  std::ofstream group_sizes("group_sizes.txt", std::ios_base::app);
  group_sizes << group_size << "\n";

  std::ofstream results_file("comparison.csv");
  results_file << "ID,Halle_obj,NA_obj\n";
  uint16_t curr_scenario_id = 0;
  /*
  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      // first block
      bool contains_needed_group = false;
      for (auto const& grp : cpg.groups_) {
        if (grp->id_ == 155721) {  // grp->id_ == 83364 || grp->id_ == 125658 ||
          // 35042 - one psg with 215 min
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
        node_arc_ilp_assignment(selected_combined_groups, data, sched,
                                results_file);
      }

      // second block
      std::map<unsigned, std::vector<combined_pg>> selected_combined_groups;
      auto& g = selected_combined_groups[cgs.first];
      g.push_back(combined_pg{cpg});
      results_file << curr_scenario_id++ << ",";
      node_arc_ilp_assignment(selected_combined_groups, data, sched,
                              results_file);

    }
  }
  */

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
  node_arc_ilp_assignment(combined_groups, data, sched, results_file);
  // heuristic_assignments(combined_groups, data, sched);
  results_file.close();
  group_sizes.close();
}

std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
paxassign::cap_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, uint16_t const allowed_delay, schedule const& sched,
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
        /*
        for (auto const& grp : cpg.groups_) {
          std::cout << "paxmon::passenger_group.id_ : " << grp->id_
                    << ", source primary ref: " << grp->source_.primary_ref_
                    << ", source secondary ref: " << grp->source_.secondary_ref_
                    << ", planned arrival time: " << grp->planned_arrival_time_
                    << std::endl;
        }
        */
        size_t curr_alt_ind = 0;
        while (curr_alt_ind < cpg.alternatives_.size()) {
          bool remove_alt = false;
          if ((cpg.alternatives_[curr_alt_ind]
                   .compact_journey_.legs_.back()
                   .exit_time_ >= cpg.groups_.back()->planned_arrival_time_) &&
              (cpg.alternatives_[curr_alt_ind]
                       .compact_journey_.legs_.back()
                       .exit_time_ -
                   cpg.groups_.back()->planned_arrival_time_ >
               allowed_delay)) {
            remove_alt = true;
          }
          if (!remove_alt) {
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

  // print_solution_routes_halle(cap_ILP_scenario, sol.alt_to_use_, sched);

  std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
      cpg_id_to_comp_jrn;
  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      auto asg = std::find_if(
          sol.alt_to_use_.begin(), sol.alt_to_use_.end(),
          [&cpg](std::pair<std::uint16_t, std::uint16_t> const& p) {
            return p.first == cpg.id_;
          });
      assert(asg != sol.alt_to_use_.end());
      if (cpg.alternatives_.size() == asg->second) {
        // NO ROUTE found
        cpg_id_to_comp_jrn.push_back({cpg, motis::paxmon::compact_journey{}});
        continue;
      } else {
        cpg_id_to_comp_jrn.push_back(
            {cpg, cpg.alternatives_[asg->second].compact_journey_});
      }
    }
  }

  return cpg_id_to_comp_jrn;

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
}

void paxassign::node_arc_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched, std::ofstream& results_file) {
  std::map<std::string, std::tuple<double, double, double, double>>
      variables_with_values_halle;

  config_graph_reduction graph_red_config{};

  auto cpg_to_cj_halle =
      cap_ilp_assignment(combined_groups, data, graph_red_config.allowed_delay_,
                         sched, variables_with_values_halle, results_file);

  node_arc_config na_config{1.2, 30, 6, 10000};
  auto te_graph = build_time_expanded_graph(data, sched, na_config);

  std::cout << "NODES IN GRAPH : " << te_graph.nodes_.size() << std::endl;
  uint32_t edges_count = 0;
  for (auto const& n : te_graph.nodes_) {
    edges_count += n->out_edges_.size();
  }
  std::cout << "EDGES IN GRAPH : " << edges_count << std::endl;

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);

  std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());
  {
    logging::scoped_timer reduce_graph_timer{"reduce te graph for passengers"};
    config_graph_reduction reduction_config;
    for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
      nodes_validity[i] =
          reduce_te_graph(eg_psg_groups[i], te_graph, reduction_config, sched);
    }
  }

  std::map<std::string, std::tuple<double, double, double, double>>
      variables_with_values_node_arc;
  perceived_tt_config perc_tt_config;
  auto solution = node_arc_ilp(eg_psg_groups, nodes_validity, te_graph,
                               na_config, perc_tt_config, sched,
                               variables_with_values_node_arc, results_file);

  double final_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, solution, perc_tt_config);
  std::cout << "manually NODE-ARC ILP CUMULATIVE: " << final_obj << std::endl;
  //print_solution_routes_node_arc(solution, eg_psg_groups, sched, te_graph);

  /*
  auto cpg_to_cj_node_arc =
      node_arc_solution_to_compact_j(eg_psg_groups, solution, sched);

  auto halle_affected_edges =
      get_edges_load_from_solutions(cpg_to_cj_halle, te_graph, sched);
  auto node_arc_affected_edges =
      get_edges_load_from_solutions(cpg_to_cj_node_arc, te_graph, sched);

  std::set<eg_edge*> all_affected_edges;
  add_affected_edges_from_sol(halle_affected_edges, all_affected_edges);
  add_affected_edges_from_sol(node_arc_affected_edges, all_affected_edges);

  auto node_arc_resulting_load = get_final_edges_load_for_solution(
      all_affected_edges, node_arc_affected_edges);
  std::cout << "FINAL LOAD NODE ARC" << std::endl;
  print_edges_load(node_arc_resulting_load, sched);

  auto halle_resulting_load = get_final_edges_load_for_solution(
      all_affected_edges, halle_affected_edges);
  std::cout << "FINAL LOAD HALLE" << std::endl;
  print_edges_load(halle_resulting_load, sched);

  auto hist_of_halle = get_load_histogram(
      halle_resulting_load, perc_tt_config.cost_function_capacity_steps_);
  auto hist_of_node_arc = get_load_histogram(
      node_arc_resulting_load, perc_tt_config.cost_function_capacity_steps_);
  */

  // TODO: heuristics: obj funktion ändern, damit cumulative perc tt optimiert
  // wird
  // TODO: heuristics: aktuellen Ansatz evaluieren
  // TODO: heuristics: akt. Ans. verbessern. Konzentration auf Problemstellen
  // TODO: ggf. nur IC/ICE im Fahrplan lassen und schauen, was mit dem Graph
  // passiert

  //throw std::runtime_error("time expanded graph is built");
}

void paxassign::heuristic_assignments(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {

  perceived_tt_config perc_tt_config;
  node_arc_config eg_config{1.2, 30, 6, 10000};

  auto te_graph = build_time_expanded_graph(data, sched, eg_config);
  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, eg_config, te_graph);

  // for subset-scenario (only long dist stations) nodes validity has the size
  // of 3,53 mb per psg. If during evaluation it will be too big, remove nodes
  // validity at all and do all the stuff without it
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

}  // namespace motis::paxassign
