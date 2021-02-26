#include "motis/paxassign/paxassign.h"

#include <chrono>
#include <ctime>
#include <filesystem>
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

  // std::ofstream group_stats("groups_stat_all.csv", std::ios_base::app);
  // group_stats << group_size << "\n";
  // group_stats.close();

  /*
   for (auto& cgs : combined_groups) {
     for (auto& cpg : cgs.second) {
       // first block
       bool contains_needed_group = false;
       for (auto const& grp : cpg.groups_) {
         if (grp->id_ == 155721) {  // grp->id_ == 83364 || grp->id_ == 125658
   ||
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

  if (combined_groups.empty()) {
    return;
  }

  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      time loc_time = cgs.second[cpg_ind].localization_.current_arrival_time_;
      time planner_arr_time =
          cgs.second[cpg_ind].groups_[0]->planned_arrival_time_;
      if (planner_arr_time > loc_time + 1440) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }

  // count_scenarios(combined_groups, data, sched);
  // filter_evaluation(combined_groups, data, sched);
  // filter_and_opt_evaluation(combined_groups, data, sched);

  // find_suspicious_groups(combined_groups, data, sched);

  // std::map<std::string, std::tuple<double, double, double, double>>
  //    variables_with_values;
  // cap_ilp_assignment(combined_groups, data, sched, variables_with_values);
  // node_arc_ilp_assignment(combined_groups, data, sched);
  heuristic_assignments(combined_groups, data, sched);
}

std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
paxassign::cap_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, uint16_t const allowed_delay, schedule const& sched,
    time_expanded_graph const& te_graph, double& obj_value,
    std::ofstream& results_file) {
  uint16_t psgs_in_sc = 0;
  for (auto& cgs : combined_groups) {
    psgs_in_sc += cgs.second.size();
  }

  auto routing_requests = 0ULL;
  auto alternatives_found = 0ULL;

  {
    scoped_timer alt_timer{"find alternatives (paxassign)"};

    auto start = std::chrono::steady_clock::now();

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

    auto end = std::chrono::steady_clock::now();
    auto time_finding_alternatives =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    // results_file << time_finding_alternatives << ",";
  }

  auto start = std::chrono::steady_clock::now();
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
          if (cpg.alternatives_[curr_alt_ind].compact_journey_.legs_.empty()) {
            remove_alt = true;
          }
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
            // CHECK IF TRIPS FROM HALLE ARE IN TE-GRAPH
            for (auto const& l :
                 cpg.alternatives_[curr_alt_ind].compact_journey_.legs_) {
              auto tr_data =
                  te_graph.trip_data_.find(to_extern_trip(sched, l.trip_));
              if (tr_data == te_graph.trip_data_.end()) {
                remove_alt = true;
              }
            }
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
  // results_file << routing_requests << "," << alternatives_found << ",";

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

  auto end = std::chrono::steady_clock::now();
  auto time_preprocessing_halle_ILP =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  // results_file << time_preprocessing_halle_ILP << ",";

  cap_ILP_solution sol;
  {
    scoped_timer alt_timer{"solve capacitated model"};
    std::srand(std::time(nullptr));
    int random_variable = std::rand();
    sol = build_ILP_from_scenario_API(cap_ILP_scenario, perc_tt_config,
                                      std::to_string(cap_ILP_scenario.size()) +
                                          "_" + std::to_string(random_variable),
                                      obj_value, results_file);
  }

  auto final_obj = piecewise_linear_convex_perceived_tt_halle(
      cap_ILP_scenario, sol.alt_to_use_, perc_tt_config);
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
}

void paxassign::count_scenarios(std::map<unsigned, std::vector<combined_pg>>&,
                                paxmon_data&, schedule const&) {
  std::ofstream count_scenarios("Count_scenarios.txt", std::ios_base::app);
  count_scenarios << "1" << std::endl;
  count_scenarios.close();
}

void paxassign::filter_and_opt_evaluation(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {
  std::time_t unique_key = std::time(nullptr);

  std::string scenario_stats_f_name = "filter_eval_scenario_stats.csv";
  bool scenario_stats_f_existed =
      std::filesystem::exists(scenario_stats_f_name);
  std::ofstream scenario_stats(scenario_stats_f_name, std::ios_base::app);
  if (!scenario_stats_f_existed) {
    scenario_stats << "ts,conf_delay,conf_inches,filter_time,build_ILP_time,"
                      "ILP_solving_time,obj\n";
  }

  std::string solution_compar_f_name = "filter_eval_solutions_comp.csv";
  bool solution_compar_f_existed =
      std::filesystem::exists(solution_compar_f_name);
  std::ofstream solutions_compar(solution_compar_f_name, std::ios_base::app);
  if (!solution_compar_f_existed) {
    solutions_compar << "ts,conf_delay,conf_inches,delay,inches\n";
  }

  std::string no_alts = "no_alts_210_6.csv";
  bool no_alts_existed = std::filesystem::exists(no_alts);
  std::ofstream no_alts_file(no_alts, std::ios_base::app);
  if (!no_alts_existed) {
    no_alts_file << "delay,inches\n";
  }

  std::string loads_f_name = "loads.csv";
  std::ofstream loads(loads_f_name, std::ios_base::app);

  node_arc_config na_config{1.2, 30, 6, 10000};
  auto te_graph = build_time_expanded_graph(data, sched, na_config);

  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      if (!cgs.second[cpg_ind].localization_.in_trip()) {
        ++cpg_ind;
        continue;
      }
      auto tr_data = te_graph.trip_data_.find(
          to_extern_trip(sched, cgs.second[cpg_ind].localization_.in_trip_));
      if (tr_data == te_graph.trip_data_.end()) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);

  std::vector<std::pair<int, int>> del_inch_conf{
      {210, 6}
      //    {120, 3}, {150, 4}, {180, 5}, {210, 6}
  };

  std::vector<
      std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>>
      all_solutions_cj(4);

  int config_index = 0;
  for (auto const& curr_conf : del_inch_conf) {
    scenario_stats << unique_key << "," << curr_conf.first << ","
                   << curr_conf.second << ",";
    std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());

    config_graph_reduction config;
    config.allowed_delay_ = curr_conf.first;
    config.max_interchanges_ = curr_conf.second;

    auto start = std::chrono::steady_clock::now();
    for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
      nodes_validity[i] =
          reduce_te_graph(eg_psg_groups[i], te_graph, config, sched);
    }
    auto end = std::chrono::steady_clock::now();
    auto time_graph_reduction_all =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    scenario_stats << time_graph_reduction_all << ",";

    double na_gurobi_obj;
    perceived_tt_config perc_tt_config;
    auto solution =
        node_arc_ilp(eg_psg_groups, nodes_validity, te_graph, na_config,
                     perc_tt_config, sched, na_gurobi_obj, scenario_stats);

    double final_obj = piecewise_linear_convex_perceived_tt_node_arc(
        eg_psg_groups, solution, perc_tt_config);
    std::cout << "manually NODE-ARC ILP CUMULATIVE: " << final_obj << std::endl;

    all_solutions_cj[config_index] =
        node_arc_solution_to_compact_j(eg_psg_groups, solution, sched);

    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        solutions_compar << unique_key << "," << curr_conf.first << ","
                         << curr_conf.second << ",";
        // PLANNED
        auto planned_exit = (*cpg.groups_.begin())
                                ->compact_planned_journey_.legs_.back()
                                .exit_time_;
        // NODE-ARC
        auto na_sol = std::find_if(
            all_solutions_cj[config_index].begin(),
            all_solutions_cj[config_index].end(),
            [&](std::pair<combined_pg&, motis::paxmon::compact_journey> const&
                    p) { return p.first.id_ == cpg.id_; });
        if (na_sol == all_solutions_cj[config_index].end()) {
          throw std::runtime_error("didn't find node-arc solution");
        }
        if (na_sol->second.legs_.empty()) {
          solutions_compar << "-,-\n";

          // CHECK REASON OF NO ROUTES WITH ROUTING
          if (curr_conf.first == 210) {
            auto alternatives = motis::paxforecast::find_alternatives(
                sched, cpg.destination_station_id_, cpg.localization_);
            if (alternatives.empty()) {
              no_alts_file << "-,-\n";
            }
            for (auto const& a : alternatives) {
              no_alts_file << a.arrival_time_ -
                                  cpg.groups_[0]->planned_arrival_time_
                           << "," << a.transfers_ << "\n";
            }
          }
        } else {
          auto na_exit = na_sol->second.legs_.back().exit_time_;
          solutions_compar << (int)na_exit - planned_exit << ",";
          auto na_interchanges = na_sol->second.legs_.size() - 1;
          solutions_compar << na_interchanges << "\n";
        }
      }
    }
    ++config_index;
  }

  std::set<eg_edge*> all_affected_edges;
  std::vector<std::map<eg_edge*, uint32_t>> affected_edges_per_solution(4);

  for (auto config_i = 0u; config_i < del_inch_conf.size(); ++config_i) {
    affected_edges_per_solution[config_i] = get_edges_load_from_solutions(
        all_solutions_cj[config_i], te_graph, sched);
    add_affected_edges_from_sol(affected_edges_per_solution[config_i],
                                all_affected_edges);
  }

  for (auto config_i = 0u; config_i < del_inch_conf.size(); ++config_i) {
    auto curr_resulting_load = get_final_edges_load_for_solution(
        all_affected_edges, affected_edges_per_solution[config_i]);
    auto rel_node_arc_loads = get_relative_loads(curr_resulting_load);
    loads << unique_key << "," << del_inch_conf[config_i].first << ","
          << del_inch_conf[config_i].second;
    for (auto const l : rel_node_arc_loads) {
      loads << "," << l;
    }
    loads << "\n";
  }

  scenario_stats.close();
  solutions_compar.close();
  loads.close();
  no_alts_file.close();
}

void paxassign::filter_evaluation(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {

  node_arc_config na_config{1.2, 30, 6, 10000};
  auto te_graph = build_time_expanded_graph(data, sched, na_config);

  std::cout << "NODES IN GRAPH : " << te_graph.nodes_.size() << std::endl;
  uint32_t edges_count = 0;
  for (auto const& n : te_graph.nodes_) {
    edges_count += n->out_edges_.size();
  }
  std::cout << "EDGES IN GRAPH : " << edges_count << std::endl;

  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      if (!cgs.second[cpg_ind].localization_.in_trip()) {
        ++cpg_ind;
        continue;
      }
      auto tr_data = te_graph.trip_data_.find(
          to_extern_trip(sched, cgs.second[cpg_ind].localization_.in_trip_));
      if (tr_data == te_graph.trip_data_.end()) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);

  /*
  std::string reducing_stats_f_name_time = "reducing_eval_time.csv";
  std::string reducing_stats_f_name_inch = "reducing_eval_inch.csv";

  bool reducing_stats_f_existed_time =
      std::filesystem::exists(reducing_stats_f_name_time);
  bool reducing_stats_f_existed_inch =
      std::filesystem::exists(reducing_stats_f_name_inch);

  std::ofstream reducing_stats_time(reducing_stats_f_name_time,
                                    std::ios_base::app);
  std::ofstream reducing_stats_inch(reducing_stats_f_name_inch,
                                    std::ios_base::app);

  if (!reducing_stats_f_existed_time) {
    reducing_stats_time << "all_nodes,120,150,180,210\n";
  }
  if (!reducing_stats_f_existed_inch) {
    reducing_stats_inch << "all_nodes,3,4,5,6\n";
  }

  {
    logging::scoped_timer reduce_graph_timer{"reduce te graph for passengers"};

    // config_graph_reduction reduction_config;

    for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
      reduce_te_graph_time(eg_psg_groups[i], te_graph, sched,
                           reducing_stats_time);
      reduce_te_graph_inch(eg_psg_groups[i], te_graph, sched,
                           reducing_stats_inch);

      // nodes_validity[i] = reduce_te_graph(
      //    eg_psg_groups[i], te_graph, reduction_config, sched,
      //    reducing_stats);
    }
  }

  reducing_stats_time.close();
  reducing_stats_inch.close();

  */

  std::string reducing_stats_f_name_combinations =
      "reducing_eval_combinations.csv";
  bool reducing_stats_f_existed_combinations =
      std::filesystem::exists(reducing_stats_f_name_combinations);
  std::ofstream reducing_stats_combinations(reducing_stats_f_name_combinations,
                                            std::ios_base::app);
  if (!reducing_stats_f_existed_combinations) {
    reducing_stats_combinations << "delay,inch,result\n";
  }

  std::vector<int> delays{120, 150, 180, 210};
  std::vector<uint16_t> inches{3, 4, 5, 6};
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    for (auto const del : delays) {
      for (auto const inch : inches) {
        config_graph_reduction config;
        config.allowed_delay_ = del;
        config.max_interchanges_ = inch;
        reducing_stats_combinations << del << "," << inch << ",";
        reduce_te_graph(eg_psg_groups[i], te_graph, config, sched,
                        reducing_stats_combinations);
      }
    }
  }
  reducing_stats_combinations.close();
}

void paxassign::find_suspicious_groups(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {

  node_arc_config na_config{1.2, 30, 6, 10000};
  auto te_graph = build_time_expanded_graph(data, sched, na_config);

  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      if (!cgs.second[cpg_ind].localization_.in_trip()) {
        ++cpg_ind;
        continue;
      }
      auto tr_data = te_graph.trip_data_.find(
          to_extern_trip(sched, cgs.second[cpg_ind].localization_.in_trip_));
      if (tr_data == te_graph.trip_data_.end()) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }
  int group_size = 0;
  for (auto const& cg : combined_groups) {
    group_size += cg.second.size();
  }
  if (group_size != 30 && group_size != 31) {
    return;
  }

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);

  std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());
  {
    logging::scoped_timer reduce_graph_timer{"reduce te graph for passengers"};
    config_graph_reduction reduction_config;
    for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
      std::cout << "FROM "
                << sched.stations_[eg_psg_groups[i].from_->station_]->name_
                << " AT " << format_time(eg_psg_groups[i].from_->time_)
                << " TO "
                << sched.stations_[eg_psg_groups[i].to_->station_]->name_
                << " AT "
                << format_time(
                       eg_psg_groups[i].cpg_.groups_[0]->planned_arrival_time_)
                << std::endl;
      std::cout << "legs enter/exit: " << std::endl;
      for (auto const& l :
           eg_psg_groups[i].cpg_.groups_[0]->compact_planned_journey_.legs_) {
        std::cout << "enter time: " << format_time(l.enter_time_)
                  << " exit time: " << format_time(l.exit_time_) << std::endl;
      }
      nodes_validity[i] =
          reduce_te_graph(eg_psg_groups[i], te_graph, reduction_config, sched);
    }
  }

  std::ofstream scenario_stats("dummy_scenario_stats.csv", std::ios_base::app);

  double na_gurobi_ojb;
  perceived_tt_config perc_tt_config;
  auto solution =
      node_arc_ilp(eg_psg_groups, nodes_validity, te_graph, na_config,
                   perc_tt_config, sched, na_gurobi_ojb, scenario_stats);
}

void paxassign::node_arc_ilp_assignment(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {
  std::time_t unique_key = std::time(nullptr);

  std::string scenario_stats_f_name = "halle_vs_na/20_scenario_stats.csv";
  std::string reducing_stats_f_name = "halle_vs_na/20_reducing_stats.csv";
  std::string solution_compar_f_name = "halle_vs_na/20_solutions_comp.csv";
  std::string loads_f_name = "halle_vs_na/20_loads.csv";

  bool scenario_stats_f_existed =
      std::filesystem::exists(scenario_stats_f_name);
  bool reducing_stats_f_existed =
      std::filesystem::exists(reducing_stats_f_name);
  bool solution_compar_f_existed =
      std::filesystem::exists(solution_compar_f_name);

  std::ofstream scenario_stats(scenario_stats_f_name, std::ios_base::app);
  std::ofstream reducing_stats(reducing_stats_f_name, std::ios_base::app);
  std::ofstream solutions_compar(solution_compar_f_name, std::ios_base::app);
  std::ofstream loads(loads_f_name, std::ios_base::app);

  // COLUMNS SCENARIO STATS
  // ts,groups,buid_te_gr,adding_psg_te_gr,graph_red_all,na_build_ILP,
  // na_obj,na_runtime,na_vars,na_constrs,halle_alt_search,halle_rout_reqs,halle_alts_found,
  // halle_prepr,halle_build_ilp,halle_obj,halle_runtime,halle_vars,halle_constr
  if (!scenario_stats_f_existed) {
    scenario_stats
        << "ts,groups,buid_te_gr,adding_psg_te_gr,graph_red_all,na_build_ILP,"
           "na_obj,na_runtime,na_vars,na_constrs,halle_alt_search,halle_rout_"
           "reqs,halle_alts_found,halle_prepr,halle_build_ilp,halle_obj,halle_"
           "runtime,halle_vars,halle_constr\n";
  }

  // COLUMNS SCENARIO STATS
  // ts,all_nodes,time_fil_time,time_fil_res,inch_fil_time,inch_fil_res,util_fil_time,util_fil_res
  if (!reducing_stats_f_existed) {
    reducing_stats << "ts,all_nodes,time_fil_time,time_fil_res,inch_fil_time,"
                      "inch_fil_res,util_fil_time,util_fil_res\n";
  }

  // COLUMNS SOLUTION COMPARISON
  // ts,na_exit_diff,na_inchs,halle_exit_diff,halle_inchs
  if (!solution_compar_f_existed) {
    solutions_compar
        << "ts,na_exit_diff,na_inchs,halle_exit_diff,halle_inchs\n";
  }

  node_arc_config na_config{1.2, 30, 6, 10000};

  scenario_stats << unique_key << ",";
  auto start = std::chrono::steady_clock::now();
  auto te_graph = build_time_expanded_graph(data, sched, na_config);
  auto end = std::chrono::steady_clock::now();
  auto time_building_te_graph =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  std::cout << "NODES IN GRAPH : " << te_graph.nodes_.size() << std::endl;
  uint32_t edges_count = 0;
  for (auto const& n : te_graph.nodes_) {
    edges_count += n->out_edges_.size();
  }
  std::cout << "EDGES IN GRAPH : " << edges_count << std::endl;

  // CHECK IN TRIP PASSENGERS
  int group_size = 0;
  for (auto const& cg : combined_groups) {
    group_size += cg.second.size();
  }
  std::cout << "GROUPS BEFORE CHECK: " << group_size << std::endl;
  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      if (!cgs.second[cpg_ind].localization_.in_trip()) {
        ++cpg_ind;
        continue;
      }
      auto tr_data = te_graph.trip_data_.find(
          to_extern_trip(sched, cgs.second[cpg_ind].localization_.in_trip_));
      if (tr_data == te_graph.trip_data_.end()) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }
  group_size = 0;
  for (auto const& cg : combined_groups) {
    group_size += cg.second.size();
  }

  std::cout << "GROUPS AFTER CHECK: " << group_size << std::endl;
  scenario_stats << group_size << ",";
  scenario_stats << time_building_te_graph << ",";

  start = std::chrono::steady_clock::now();
  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, na_config, te_graph);
  end = std::chrono::steady_clock::now();
  auto time_adding_psgs_to_graph =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  scenario_stats << time_adding_psgs_to_graph << ",";

  std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());
  start = std::chrono::steady_clock::now();
  {
    logging::scoped_timer reduce_graph_timer{"reduce te graph for passengers"};
    config_graph_reduction reduction_config;
    for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
      reducing_stats << unique_key << ",";
      nodes_validity[i] = reduce_te_graph(
          eg_psg_groups[i], te_graph, reduction_config, sched, reducing_stats);
    }
  }
  end = std::chrono::steady_clock::now();
  auto time_graph_reduction_all =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  scenario_stats << time_graph_reduction_all << ",";

  double na_gurobi_obj;
  perceived_tt_config perc_tt_config;
  auto solution =
      node_arc_ilp(eg_psg_groups, nodes_validity, te_graph, na_config,
                   perc_tt_config, sched, na_gurobi_obj, scenario_stats);

  double final_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, solution, perc_tt_config);
  std::cout << "manually NODE-ARC ILP CUMULATIVE: " << final_obj << std::endl;
  // print_solution_routes_node_arc(solution, eg_psg_groups, sched, te_graph);

  auto cpg_to_cj_node_arc =
      node_arc_solution_to_compact_j(eg_psg_groups, solution, sched);

  // HALLE #----------------------------#
  config_graph_reduction graph_red_config{};
  double halle_obj;
  auto cpg_to_cj_halle =
      cap_ilp_assignment(combined_groups, data, graph_red_config.allowed_delay_,
                         sched, te_graph, halle_obj, scenario_stats);
  // END HALLE #------------------------#

  for (auto& cgs : combined_groups) {
    for (auto& cpg : cgs.second) {
      solutions_compar << unique_key << ",";
      // PLANNED
      auto planned_exit = (*cpg.groups_.begin())
                              ->compact_planned_journey_.legs_.back()
                              .exit_time_;
      // NODE-ARC
      auto na_sol = std::find_if(
          cpg_to_cj_node_arc.begin(), cpg_to_cj_node_arc.end(),
          [&](std::pair<combined_pg&, motis::paxmon::compact_journey> const&
                  p) { return p.first.id_ == cpg.id_; });
      if (na_sol == cpg_to_cj_node_arc.end()) {
        throw std::runtime_error("didn't find node-arc solution");
      }
      if (na_sol->second.legs_.empty()) {
        solutions_compar << "-,-,";
      } else {
        auto na_exit = na_sol->second.legs_.back().exit_time_;
        solutions_compar << (int)na_exit - planned_exit << ",";
        auto na_interchanges = na_sol->second.legs_.size() - 1;
        solutions_compar << na_interchanges << ",";
      }

      // HALLE
      auto halle_sol = std::find_if(
          cpg_to_cj_halle.begin(), cpg_to_cj_halle.end(),
          [&](std::pair<combined_pg&, motis::paxmon::compact_journey> const&
                  p) { return p.first.id_ == cpg.id_; });
      if (halle_sol == cpg_to_cj_halle.end()) {
        throw std::runtime_error("didn't find halle solution");
      }
      if (halle_sol->second.legs_.empty()) {
        solutions_compar << "-,-\n";
      } else {
        auto halle_exit = halle_sol->second.legs_.back().exit_time_;
        solutions_compar << (int)halle_exit - planned_exit << ",";
        auto halle_interchanges = halle_sol->second.legs_.size() - 1;
        solutions_compar << halle_interchanges << "\n";
      }
    }
  }

  auto halle_affected_edges =
      get_edges_load_from_solutions(cpg_to_cj_halle, te_graph, sched);
  auto node_arc_affected_edges =
      get_edges_load_from_solutions(cpg_to_cj_node_arc, te_graph, sched);

  std::set<eg_edge*> all_affected_edges;
  add_affected_edges_from_sol(halle_affected_edges, all_affected_edges);
  add_affected_edges_from_sol(node_arc_affected_edges, all_affected_edges);

  auto node_arc_resulting_load = get_final_edges_load_for_solution(
      all_affected_edges, node_arc_affected_edges);
  // std::cout << "FINAL LOAD NODE ARC" << std::endl;
  // print_edges_load(node_arc_resulting_load, sched);

  auto halle_resulting_load = get_final_edges_load_for_solution(
      all_affected_edges, halle_affected_edges);
  // std::cout << "FINAL LOAD HALLE" << std::endl;
  // print_edges_load(halle_resulting_load, sched);

  auto rel_node_arc_loads = get_relative_loads(node_arc_resulting_load);
  loads << unique_key << ",na";
  for (auto const l : rel_node_arc_loads) {
    loads << "," << l;
  }
  loads << "\n";

  auto rel_halle_loads = get_relative_loads(halle_resulting_load);
  loads << unique_key << ",halle";
  for (auto const l : rel_halle_loads) {
    loads << "," << l;
  }
  loads << "\n";

  auto hist_of_halle = get_load_histogram(
      halle_resulting_load, perc_tt_config.cost_function_capacity_steps_);
  auto hist_of_node_arc = get_load_histogram(
      node_arc_resulting_load, perc_tt_config.cost_function_capacity_steps_);

  scenario_stats.close();
  reducing_stats.close();
  solutions_compar.close();
  loads.close();

  // throw std::runtime_error("time expanded graph is built");
}

void paxassign::heuristic_assignments(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    paxmon_data& data, schedule const& sched) {
  node_arc_config eg_config{1.2, 30, 6, 10000};
  auto te_graph = build_time_expanded_graph(data, sched, eg_config);

  for (auto& cgs : combined_groups) {
    size_t cpg_ind = 0;
    while (cpg_ind < cgs.second.size()) {
      if (!cgs.second[cpg_ind].localization_.in_trip()) {
        ++cpg_ind;
        continue;
      }
      auto tr_data = te_graph.trip_data_.find(
          to_extern_trip(sched, cgs.second[cpg_ind].localization_.in_trip_));
      if (tr_data == te_graph.trip_data_.end()) {
        cgs.second.erase(cgs.second.begin() + cpg_ind);
      } else {
        ++cpg_ind;
      }
    }
  }

  int group_size = 0;
  for (auto const& cg : combined_groups) {
    group_size += cg.second.size();
  }
  if (group_size != 35) {
    return;
  }

  std::cout << "Groups in scenario: " << group_size << std::endl;

  config_graph_reduction reduction_config;

  // HALLE #----------------------------#
  std::string scenario_stats_f_name = "heuristics_temp_stats.csv";
  bool scenario_stats_f_existed =
      std::filesystem::exists(scenario_stats_f_name);
  std::ofstream scenario_stats(scenario_stats_f_name, std::ios_base::app);
  if (!scenario_stats_f_existed) {
    scenario_stats
        << "gr_size,AP_obj,NA_obj,greedy_obj,load_based,delay_based\n";
  }
  scenario_stats << group_size << ",";

  double halle_gurobi_obj;
  auto cpg_to_cj_halle =
      cap_ilp_assignment(combined_groups, data, reduction_config.allowed_delay_,
                         sched, te_graph, halle_gurobi_obj, scenario_stats);
  // END HALLE #------------------------#

  auto eg_psg_groups =
      add_psgs_to_te_graph(combined_groups, sched, eg_config, te_graph);
  std::vector<std::vector<bool>> nodes_validity(eg_psg_groups.size());
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    nodes_validity[i] =
        reduce_te_graph(eg_psg_groups[i], te_graph, reduction_config, sched);
  }

  perceived_tt_config perc_tt_config;

  // NODE-ARC
  node_arc_config na_config{1.2, 30, 6, 10000};
  double na_gurobi_obj;
  auto solution =
      node_arc_ilp(eg_psg_groups, nodes_validity, te_graph, na_config,
                   perc_tt_config, sched, na_gurobi_obj, scenario_stats);
  auto cpg_to_cj_node_arc =
      node_arc_solution_to_compact_j(eg_psg_groups, solution, sched);
  // END NODE-ARC

  std::cout << "arc-path: " << halle_gurobi_obj
            << " vs node-arc: " << na_gurobi_obj << std::endl;

  // NOT AS IT IS IN HALLE PAPER FOR INITIALIZATION WITH GREEDY
  // perceived tt for start solution
  auto calc_perc_tt_dist = [&](eg_edge* e, double curr_dist) {
    if (e->capacity_utilization_ >
        perc_tt_config.cost_function_capacity_steps_.back()) {
      return std::numeric_limits<double>::max();
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

  if (halle_gurobi_obj < na_gurobi_obj) {
    std::cout << "Start to check trips of legs" << std::endl;
    for (auto const& p : cpg_to_cj_halle) {
      for (auto const& l : p.second.legs_) {
        if (te_graph.trip_data_.find(to_extern_trip(sched, l.trip_)) ==
            te_graph.trip_data_.end()) {
          std::cout << "TRIP NOT IN TE GRAPH" << std::endl;
        }
      }
    }
    std::cout << "Trips checked" << std::endl;
    std::cout << "Start to compare legs" << std::endl;
    for (auto const& p : cpg_to_cj_halle) {
      auto loc = p.first.localization_.in_trip() ? " in trip " : " at station ";
      std::cout << "inspected group id: " << p.first.id_ << loc
                << " loc time: " << p.first.localization_.current_arrival_time_
                << " dest: " << p.first.destination_station_id_ << " == "
                << sched.stations_[p.first.destination_station_id_]->name_
                << " (planned arr time: "
                << p.first.groups_[0]->planned_arrival_time_ << ")"
                << std::endl;
      std::cout << "halle solution: " << std::endl;
      for (auto const& l : p.second.legs_) {
        std::cout << l.trip_->id_.primary_.train_nr_ << " from "
                  << l.enter_station_id_ << " to " << l.exit_station_id_
                  << " == " << sched.stations_[l.exit_station_id_]->name_
                  << " at " << l.exit_time_ << std::endl;
        std::cout << "----------- greedy to leg target -----------"
                  << std::endl;

        std::vector<eg_event_node*> relevant_nodes =
            utl::all(te_graph.st_to_nodes_[l.exit_station_id_]) |
            utl::remove_if([&](auto const& n) {
              return n->type_ != eg_event_type::WAIT ||
                     n->time_ < l.exit_time_ + 30;
            }) |
            utl::vec();
        if (relevant_nodes.empty()) {
          std::cout << "no wait nodes at the station" << std::endl;
        } else {
          for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
            if (eg_psg_groups[i].cpg_.id_ == p.first.id_) {
              auto greedy_sol = sssd_dijkstra<double>(
                  eg_psg_groups[i].from_, eg_psg_groups[i].to_,
                  eg_psg_groups[i].psg_count_, 0.0,
                  std::numeric_limits<double>::max(), te_graph,
                  nodes_validity[i], 6, calc_perc_tt_dist);
              std::cout << "Greedy solution: " << std::endl;
              for (auto const& e : greedy_sol) {
                std::cout << "e_type: " << e->type_ << std::endl;
                if (e->type_ == eg_edge_type::TRIP) {
                  std::cout << e->trip_->id_.primary_.train_nr_ << std::endl;
                }
              }
              std::cout << " all outgoing arcs from localization node: "
                        << std::endl;
              for (auto const& oe : eg_psg_groups[i].from_->out_edges_) {
                std::cout << "e_type: " << oe->type_ << " to "
                          << oe->to_->station_ << std::endl;
                if (oe->type_ == eg_edge_type::TRIP) {
                  std::cout << oe->trip_->id_.primary_.train_nr_ << std::endl;
                }
              }
            }
          }
        }
      }
      std::cout << "na solution: " << std::endl;
      for (int i = 0; i < cpg_to_cj_node_arc.size(); ++i) {
        if (cpg_to_cj_node_arc[i].first.id_ == p.first.id_) {
          for (auto const& l : cpg_to_cj_node_arc[i].second.legs_) {
            std::cout << l.trip_->id_.primary_.train_nr_ << " from "
                      << l.enter_station_id_ << " to " << l.exit_station_id_
                      << " at " << l.exit_time_ << std::endl;
          }
        }
      }
    }
    std::cout << "Legs checked" << std::endl;

    throw std::runtime_error("to check");
  }

  auto rng = std::mt19937{};

  // Start solution for local search. calc_perc_tt_dist is used
  auto greedy_solution = greedy_assignment(
      te_graph, nodes_validity, eg_config.max_allowed_interchanges_,
      eg_psg_groups, rng, calc_perc_tt_dist);
  double greedy_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, greedy_solution, perc_tt_config);
  std::cout << "manually GREEDY CUMULATIVE: " << greedy_obj << std::endl;
  scenario_stats << greedy_obj << ",";

  {
    scoped_timer alt_timer{"FIND PROBLEMATIC GROUPS"};
    std::vector<int> load_based_order;
    std::vector<int> delay_based_order;
    find_problematic_groups(eg_psg_groups, greedy_solution, load_based_order,
                            delay_based_order);
    auto load_order_solution = greedy_assignment_spec_order(
        te_graph, nodes_validity, eg_config.max_allowed_interchanges_,
        eg_psg_groups, load_based_order, calc_perc_tt_dist);
    double load_order_obj = piecewise_linear_convex_perceived_tt_node_arc(
        eg_psg_groups, load_order_solution, perc_tt_config);
    std::cout << "load_order_obj GREEDY CUMULATIVE: " << load_order_obj
              << std::endl;
    scenario_stats << load_order_obj << ",";

    auto delay_order_solution = greedy_assignment_spec_order(
        te_graph, nodes_validity, eg_config.max_allowed_interchanges_,
        eg_psg_groups, delay_based_order, calc_perc_tt_dist);
    double delay_order_obj = piecewise_linear_convex_perceived_tt_node_arc(
        eg_psg_groups, delay_order_solution, perc_tt_config);
    std::cout << "delay_order_obj GREEDY CUMULATIVE: " << delay_order_obj
              << std::endl;
    scenario_stats << delay_order_obj << "\n";
  }

  /*
  {
    scoped_timer alt_timer{"LOCAL SEARCH"};
    auto ls_solution = local_search(
        eg_psg_groups, greedy_solution, perc_tt_config, 3, rng, te_graph,
        nodes_validity, eg_config.max_allowed_interchanges_, calc_perc_tt_dist);

    final_obj = piecewise_linear_convex_perceived_tt_node_arc(
        eg_psg_groups, ls_solution, perc_tt_config);
    std::cout << "manually LS CUMULATIVE: " << final_obj << std::endl;
    scenario_stats << final_obj << "\n";
  }
  */
  scenario_stats.close();
  // throw std::runtime_error("heuristic algorithms finished");
}

}  // namespace motis::paxassign
