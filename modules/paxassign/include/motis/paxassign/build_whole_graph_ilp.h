#pragma once

#include "motis/paxassign/algorithms_configs.h"
#include "motis/paxassign/reduce_te_graph.h"
#include "motis/paxassign/time_expanded_graph.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

std::vector<std::vector<eg_edge*>> node_arc_ilp(
    std::vector<eg_psg_group>& psg_groups,
    std::vector<std::vector<bool>> const& nodes_validity,
    time_expanded_graph const& te_graph, node_arc_config const& na_config,
    perceived_tt_config const& tt_config, schedule const& sched,
    std::map<std::string, std::tuple<double, double, double, double>>&
        variables_with_values,
    std::ofstream& results_file) {
  try {
    auto start = std::chrono::steady_clock::now();

    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    uint64_t curr_var_nr = 0;

    std::map<eg_edge*, std::vector<GRBVar>> edge_cost_vars;
    {
      logging::scoped_timer var_timer{"ILP: add edge cost variables"};
      for (auto const& n : te_graph.nodes_) {
        bool valid_for_any = false;
        for (auto j = 0u; j < nodes_validity.size(); ++j) {
          if (nodes_validity[j][n->id_]) valid_for_any = true;
        }
        if (!valid_for_any) continue;
        for (auto const& e : n->out_edges_) {
          switch (e->type_) {
            case eg_edge_type::TRIP: {
              uint64_t last_cap_step = 0;
              for (auto const [i, penalty] :
                   utl::enumerate(tt_config.tt_and_waiting_penalties_)) {
                auto curr_cap_step =
                    uint64_t(e->soft_cap_boundary_ *
                             tt_config.cost_function_capacity_steps_[i]);
                uint64_t remaining_cap = 0;
                if (e->passengers_ > curr_cap_step) {
                  remaining_cap = 0;
                } else {
                  if (e->passengers_ > last_cap_step) {
                    remaining_cap = curr_cap_step - e->passengers_;
                  } else {
                    remaining_cap = curr_cap_step - last_cap_step;
                  }
                }
                last_cap_step = curr_cap_step;
                if (remaining_cap == 0) continue;
                edge_cost_vars[e.get()].push_back(model.addVar(
                    0.0, remaining_cap, penalty * e->cost_, GRB_INTEGER,
                    "T_" + std::to_string(e->from_->station_) + "_" +
                        std::to_string(e->to_->station_) + "_" +
                        std::to_string(e->from_->time_) + "_" +
                        std::to_string(e->to_->time_) + "_" +
                        std::to_string(e->trip_->id_.primary_.train_nr_) + "_" +
                        std::to_string(i) + "_" +
                        std::to_string(curr_var_nr++)));
              }
              break;
            }
            case eg_edge_type::WAIT_TRANSPORT: {
              uint32_t last_cap_step = 0;
              for (auto const [i, penalty] :
                   utl::enumerate(tt_config.tt_and_waiting_penalties_)) {
                auto curr_cap_step =
                    uint64_t(e->soft_cap_boundary_ *
                             tt_config.cost_function_capacity_steps_[i]);
                uint64_t remaining_cap = 0;
                if (e->passengers_ > curr_cap_step) {
                  remaining_cap = 0;
                } else {
                  if (e->passengers_ > last_cap_step) {
                    remaining_cap = curr_cap_step - e->passengers_;
                  } else {
                    remaining_cap = curr_cap_step - last_cap_step;
                  }
                }
                last_cap_step = curr_cap_step;
                if (remaining_cap == 0) continue;
                edge_cost_vars[e.get()].push_back(model.addVar(
                    0.0, remaining_cap, penalty * e->cost_, GRB_INTEGER,
                    "W_" + std::to_string(e->from_->station_) + "_" +
                        std::to_string(e->to_->station_) + "_" +
                        std::to_string(e->from_->time_) + "_" +
                        std::to_string(e->to_->time_) + "_" +
                        std::to_string(i) + "_" +
                        std::to_string(curr_var_nr++)));
              }
              break;
            }
            case eg_edge_type::TRAIN_ENTRY: {
              edge_cost_vars[e.get()].push_back(model.addVar(
                  0.0, std::numeric_limits<double>::max(),
                  tt_config.transfer_penalty_ + e->cost_, GRB_INTEGER,
                  "ENTR_" + std::to_string(e->from_->station_) + "_" +
                      std::to_string(e->to_->station_) + "_" +
                      std::to_string(e->from_->time_) + "_" +
                      std::to_string(e->to_->time_) + "_" +
                      std::to_string(curr_var_nr++)));
              break;
            }
            case eg_edge_type::NO_ROUTE: {
              edge_cost_vars[e.get()].push_back(
                  model.addVar(0.0, std::numeric_limits<double>::max(),
                               tt_config.no_route_cost_, GRB_INTEGER,
                               "NR_" + std::to_string(e->from_->station_) +
                                   "_" + std::to_string(e->to_->station_) +
                                   "_" + std::to_string(curr_var_nr++)));
              break;
            }
            case eg_edge_type::WAIT_STATION: {
              edge_cost_vars[e.get()].push_back(
                  model.addVar(0.0, std::numeric_limits<double>::max(),
                               e->cost_, GRB_INTEGER,
                               "WS_" + std::to_string(e->from_->station_) +
                                   "_" + std::to_string(e->to_->station_) +
                                   "_" + std::to_string(e->from_->time_) + "_" +
                                   std::to_string(e->to_->time_) + "_" +
                                   std::to_string(curr_var_nr++)));
              break;
            }
            case eg_edge_type::FINISH: {
              edge_cost_vars[e.get()].push_back(
                  model.addVar(0.0, std::numeric_limits<double>::max(),
                               e->cost_, GRB_INTEGER,
                               "FIN_" + std::to_string(e->from_->station_) +
                                   "_" + std::to_string(e->to_->station_) +
                                   "_" + std::to_string(e->from_->time_) + "_" +
                                   std::to_string(e->to_->time_) + "_" +
                                   std::to_string(curr_var_nr++)));
              break;
            }
            case eg_edge_type::TRAIN_EXIT: {
              edge_cost_vars[e.get()].push_back(
                  model.addVar(0.0, std::numeric_limits<double>::max(),
                               e->cost_, GRB_INTEGER,
                               "EXIT_" + std::to_string(e->from_->station_) +
                                   "_" + std::to_string(e->to_->station_) +
                                   "_" + std::to_string(e->from_->time_) + "_" +
                                   std::to_string(e->to_->time_) + "_" +
                                   std::to_string(curr_var_nr++)));
              break;
            }
          }
        }
      }
    }

    // f.e. psg group k and f.e. edge (i,j) add variable x^k_(i,j)
    std::vector<std::map<eg_edge*, GRBVar>> commodities_edge_usage_vars(
        psg_groups.size());

    {
      logging::scoped_timer var_timer{
          "ILP: add commodity edge usage - variables"};
      for (auto i = 0u; i < psg_groups.size(); ++i) {
        for (auto const& n : te_graph.nodes_) {
          if (!nodes_validity[i][n->id_]) continue;
          for (auto const& e : n->out_edges_) {
            if (!nodes_validity[i][e->to_->id_]) continue;
            commodities_edge_usage_vars[i][e.get()] = model.addVar(
                0.0, 1.0, 0.0, GRB_BINARY,
                std::to_string(i) + "_" + eg_edge_type_to_string(e.get()) +
                    "_" + std::to_string(e->from_->id_) + "_" +
                    std::to_string(e->to_->id_) + "_" +
                    std::to_string(curr_var_nr++));
          }
        }
      }
    }

    // CONSTRAINTS
    // conservation constraints
    {
      logging::scoped_timer conserv_cons_timer{
          "ILP: add conservation constraints"};
      for (auto i = 0u; i < psg_groups.size(); ++i) {
        for (auto const& n : te_graph.nodes_) {
          if (n->in_edges_.empty() && n->out_edges_.empty()) {
            continue;
          }
          GRBLinExpr lhs = 0;
          bool lhs_valid = false;
          for (auto const& oe : n->out_edges_) {
            if (commodities_edge_usage_vars[i].find(oe.get()) ==
                commodities_edge_usage_vars[i].end()) {
              continue;
            }
            lhs += commodities_edge_usage_vars[i][oe.get()];
            lhs_valid = true;
          }
          for (auto const& ie : n->in_edges_) {
            if (commodities_edge_usage_vars[i].find(ie) ==
                commodities_edge_usage_vars[i].end()) {
              continue;
            }
            lhs -= commodities_edge_usage_vars[i][ie];
            lhs_valid = true;
          }
          double rhs = 0.0;
          if (n.get() == psg_groups[i].from_) {
            rhs = 1.0;
          }
          if (n.get() == psg_groups[i].to_) {
            rhs = -1.0;
          }
          if (lhs_valid) {
            model.addConstr(lhs, GRB_EQUAL, rhs);
          }
        }
      }
    }

    // interchanges upper bound
    {
      logging::scoped_timer inch_cons_timer{"ILP: interchange constraints"};
      for (auto i = 0u; i < psg_groups.size(); ++i) {
        GRBLinExpr lhs = 0;
        for (auto const& n : te_graph.nodes_) {
          if (!nodes_validity[i][n->id_]) continue;
          for (auto const& e : n->out_edges_) {
            if (!nodes_validity[i][e->to_->id_]) continue;
            if (e->type_ == eg_edge_type::TRAIN_ENTRY) {
              lhs += commodities_edge_usage_vars[i][e.get()];
            }
          }
        }
        model.addConstr(lhs, GRB_LESS_EQUAL,
                        na_config.max_allowed_interchanges_);
      }
    }

    {
      logging::scoped_timer capacity_cost_timer{"capacity + cost constraints"};
      // capacity and cost
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          GRBLinExpr lhs = 0;
          bool lhs_valid = false;
          for (auto i = 0u; i < psg_groups.size(); ++i) {
            if (commodities_edge_usage_vars[i].find(e.get()) ==
                commodities_edge_usage_vars[i].end()) {
              continue;
            }
            lhs += psg_groups[i].psg_count_ *
                   commodities_edge_usage_vars[i][e.get()];
            lhs_valid = true;
          }
          if (lhs_valid) {
            for (auto const& ec_v : edge_cost_vars[e.get()]) {
              lhs -= ec_v;
            }
            model.addConstr(lhs, GRB_EQUAL, 0.0);
          }
        }
      }
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    auto end = std::chrono::steady_clock::now();
    auto time_building_ILP =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    results_file << time_building_ILP << ",";

    model.optimize();

    int status = model.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
      throw std::runtime_error("node-arc-form ILP model: solution not optimal");
    }

    for (auto const& ecv : edge_cost_vars) {
      for (auto const& curr_ecv : ecv.second) {
        variables_with_values[curr_ecv.get(GRB_StringAttr_VarName)] =
            std::make_tuple(curr_ecv.get(GRB_DoubleAttr_X),
                            curr_ecv.get(GRB_DoubleAttr_Obj),
                            curr_ecv.get(GRB_DoubleAttr_X) *
                                curr_ecv.get(GRB_DoubleAttr_Obj),
                            curr_ecv.get(GRB_DoubleAttr_UB));
      }
    }

    std::vector<std::vector<eg_edge*>> solution(psg_groups.size());
    for (auto i = 0u; i < psg_groups.size(); ++i) {
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          if (commodities_edge_usage_vars[i].find(e.get()) ==
              commodities_edge_usage_vars[i].end()) {
            continue;
          }
          if (commodities_edge_usage_vars[i][e.get()].get(GRB_DoubleAttr_X) ==
              1.0) {
            solution[i].push_back(e.get());
          }
        }
      }
      std::sort(std::begin(solution[i]), std::end(solution[i]),
                [](eg_edge const* lhs, eg_edge const* rhs) {
                  return ((lhs->to_->time_ < rhs->to_->time_) ||
                          (lhs->to_->time_ == rhs->to_->time_ &&
                           lhs->from_->time_ < rhs->from_->time_));
                });
    }

    std::cout << "TIME: " << model.get(GRB_DoubleAttr_Runtime) << std::endl;

    results_file << model.get(GRB_DoubleAttr_ObjVal) << ","
                 << model.get(GRB_DoubleAttr_Runtime) << ","
                 << model.get(GRB_IntAttr_NumVars) << ","
                 << model.get(GRB_IntAttr_NumGenConstrs) +
                        model.get(GRB_IntAttr_NumConstrs)
                 << ",";

    // model.write("motis/build/rel/ilp_files/node_arc.sol");
    // model.write("motis/build/rel/ilp_files/node_arc.lp");
    return solution;
  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
  return {};
}

}  // namespace motis::paxassign
