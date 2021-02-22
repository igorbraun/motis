#pragma once

#include <fstream>

#include "motis/paxassign/algorithms_configs.h"
#include "motis/paxassign/capacitaty_aware_structs.h"

#include "utl/enumerate.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

cap_ILP_solution build_ILP_from_scenario_API(
    std::vector<cap_ILP_psg_group> const& passengers,
    perceived_tt_config const& config, std::string const&,
    std::map<std::string, std::tuple<double, double, double, double>>&
        variables_with_values,
    std::ofstream& results_file) {
  try {
    auto start = std::chrono::steady_clock::now();

    std::set<cap_ILP_edge*> handled_edges;
    std::map<cap_ILP_edge*, std::set<cap_ILP_psg_group const*>> edge_to_psgs;

    for (auto const& pg : passengers) {
      for (auto const& a : pg.alternatives_) {
        for (auto const& e : a.edges_) {
          edge_to_psgs[e].insert(&pg);
          handled_edges.insert(e);
        }
      }
    }

    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    uint64_t curr_var_nr = 0;

    // EDGE VARIABLES
    std::map<cap_ILP_edge*, std::vector<GRBVar>> edge_cost_vars;
    for (auto const& e : handled_edges) {
      switch (e->type_) {
        case edge_type::TRIP: {
          uint64_t last_cap_step = 0;
          for (auto const [i, penalty] :
               utl::enumerate(config.tt_and_waiting_penalties_)) {
            auto curr_cap_step =
                uint64_t(e->soft_cap_boundary_ *
                         config.cost_function_capacity_steps_[i]);
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
            edge_cost_vars[e].push_back(model.addVar(
                0.0, remaining_cap, penalty * e->tt_, GRB_INTEGER,
                "T_" + std::to_string(e->from_->station_) + "_" +
                    std::to_string(e->to_->station_) + "_" +
                    std::to_string(e->from_->time_) + "_" +
                    std::to_string(e->to_->time_) + "_" +
                    std::to_string(e->trip_->id_.primary_.train_nr_) + "_" +
                    std::to_string(i) + "_" + std::to_string(curr_var_nr++)));
          }
          break;
        }
        case edge_type::WAIT: {
          uint32_t last_cap_step = 0;
          for (auto const [i, penalty] :
               utl::enumerate(config.tt_and_waiting_penalties_)) {
            auto curr_cap_step =
                uint64_t(e->soft_cap_boundary_ *
                         config.cost_function_capacity_steps_[i]);
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
            edge_cost_vars[e].push_back(model.addVar(
                0.0, remaining_cap, penalty * e->tt_, GRB_INTEGER,
                "W_" + std::to_string(e->from_->station_) + "_" +
                    std::to_string(e->to_->station_) + "_" +
                    std::to_string(e->from_->time_) + "_" +
                    std::to_string(e->to_->time_) + "_" + std::to_string(i) +
                    "_" + std::to_string(curr_var_nr++)));
          }
          break;
        }
        case edge_type::INTERCHANGE: {
          edge_cost_vars[e].push_back(
              model.addVar(0.0, std::numeric_limits<double>::max(),
                           config.transfer_penalty_ + e->tt_, GRB_INTEGER,
                           "I_" + std::to_string(e->from_->station_) + "_" +
                               std::to_string(e->to_->station_) + "_" +
                               std::to_string(e->from_->time_) + "_" +
                               std::to_string(e->to_->time_) + "_" +
                               std::to_string(curr_var_nr++)));
          break;
        }
        case edge_type::NOROUTE: {
          edge_cost_vars[e].push_back(model.addVar(
              0.0, std::numeric_limits<double>::max(), config.no_route_cost_,
              GRB_INTEGER, "NOROUTE_" + std::to_string(curr_var_nr++)));
          break;
        }
      }
    }

    // PSG ALTERNATIVES VARIABLES
    auto max_pg_pr = std::max_element(
        std::begin(passengers), std::end(passengers),
        [](cap_ILP_psg_group const& pg1, cap_ILP_psg_group const& pg2) {
          return pg1.id_ < pg2.id_;
        });
    std::vector<std::vector<GRBVar>> alt_route_vars(max_pg_pr->id_ + 1);
    for (auto const& pg : passengers) {
      for (auto const& a : pg.alternatives_) {
        alt_route_vars[pg.id_].push_back(model.addVar(
            0.0, 1.0, pg.psg_count_ * a.associated_waiting_time_, GRB_BINARY,
            "y_" + std::to_string(pg.id_) + "_" + std::to_string(a.id_)));
      }
    }

    // PSG-EDGE USAGE VARIABLES
    std::map<cap_ILP_edge*, std::vector<std::pair<uint32_t, GRBVar>>>
        edge_usage_vars;
    for (auto const& e : handled_edges) {
      for (auto const& pg : edge_to_psgs[e]) {
        edge_usage_vars[e].push_back(
            {pg->id_, model.addVar(0.0, 1.0, 0.0, GRB_BINARY,
                                   "delta_" + std::to_string(pg->id_) + "_" +
                                       std::to_string(e->id_))});
      }
    }

    // CONSTRAINTS
    // AT LEAST ONE ROUTE F.E. GROUP
    for (auto const& pg : passengers) {
      GRBLinExpr lhs = 0;
      for (auto const& ar : alt_route_vars[pg.id_]) {
        lhs += ar;
      }
      model.addConstr(lhs, GRB_EQUAL, 1.0);
    }

    // ALTERNATIVE -> PSG-EDGE USAGE ACTIVATION
    for (auto const& pg : passengers) {
      uint16_t alt_id = 0;
      for (auto const& a : pg.alternatives_) {
        for (auto const& e : a.edges_) {
          for (auto const& p_to_edge : edge_usage_vars[e]) {
            if (p_to_edge.first == pg.id_) {
              model.addGenConstrIndicator(alt_route_vars[pg.id_][alt_id], 1.0,
                                          p_to_edge.second, GRB_EQUAL, 1.0);
              break;
            }
          }
        }
        alt_id += 1;
      }
    }

    // EDGES CAPACITY
    for (auto const& e : handled_edges) {
      GRBLinExpr lhs = 0;
      for (auto const& p_to_edge : edge_usage_vars[e]) {
        auto pg_it = std::find_if(
            std::begin(passengers), std::end(passengers),
            [&](auto const& p) { return p.id_ == p_to_edge.first; });
        lhs += pg_it->psg_count_ * p_to_edge.second;
      }
      for (auto const& ec_v : edge_cost_vars[e]) {
        lhs -= ec_v;
      }
      model.addConstr(lhs, GRB_EQUAL, 0.0);
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    // model.write("motis/build/rel/ilp_files/" + scenario_id + ".lp");

    auto end = std::chrono::steady_clock::now();
    auto time_building_ILP =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    // results_file << time_building_ILP << ",";

    model.optimize();

    // model.write("motis/build/rel/ilp_files/" + scenario_id + ".sol");
    // model.write("motis/build/rel/ilp_files/halle.sol");

    int status = model.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
      throw std::runtime_error("capacitated ILP model: solution not optimal");
    }

    std::vector<std::pair<ilp_psg_id, alt_idx>> alt_to_use;
    int no_alt = 0;

    for (auto const& pg : passengers) {
      uint32_t i = 0;
      for (auto const& curr_alt : alt_route_vars[pg.id_]) {
        if (curr_alt.get(GRB_DoubleAttr_X) == 1.0) {
          if (i == alt_route_vars[pg.id_].size() - 1) {
            ++no_alt;
          }
          alt_to_use.push_back({pg.id_, i});
          break;
        }
        ++i;
      }
    }

    for (auto const& arv : alt_route_vars) {
      for (auto const& curr_arv : arv) {
        variables_with_values[curr_arv.get(GRB_StringAttr_VarName)] =
            std::make_tuple(curr_arv.get(GRB_DoubleAttr_X),
                            curr_arv.get(GRB_DoubleAttr_Obj),
                            curr_arv.get(GRB_DoubleAttr_X) *
                                curr_arv.get(GRB_DoubleAttr_Obj),
                            curr_arv.get(GRB_DoubleAttr_UB));
      }
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

    cap_ILP_solution solution{
        cap_ILP_stats{passengers.size(), no_alt, model.get(GRB_IntAttr_NumVars),
                      model.get(GRB_IntAttr_NumGenConstrs) +
                          model.get(GRB_IntAttr_NumConstrs),
                      model.get(GRB_DoubleAttr_Runtime),
                      model.get(GRB_DoubleAttr_ObjVal)},
        alt_to_use};
    std::cout << "TIME: " << model.get(GRB_DoubleAttr_Runtime) << std::endl;

    results_file << model.get(GRB_DoubleAttr_ObjVal) << ",";
    /*
    results_file << model.get(GRB_DoubleAttr_ObjVal) << ","
                 << model.get(GRB_DoubleAttr_Runtime) << ","
                 << model.get(GRB_IntAttr_NumVars) << ","
                 << model.get(GRB_IntAttr_NumGenConstrs) +
                        model.get(GRB_IntAttr_NumConstrs)
                 << "\n";
    */
    return solution;

  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
  return {};
}

/*
void build_ILP_from_scenario(std::vector<cap_ILP_psg_group> const& passengers,
                             perceived_tt_config const& config,
                             std::string const& scenario_id) {
  std::ofstream ilp_file("motis/build/rel/ilp_files/" + scenario_id + ".lp");
  // OBJECTIVE PART
  ilp_file << "Minimize\n";
  std::set<cap_ILP_edge const*> handled_edges;
  std::map<cap_ILP_edge*, std::set<cap_ILP_psg_group const*>> edge_to_psgs;
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        edge_to_psgs[e].insert(&pg);
        if (handled_edges.find(e) != handled_edges.end()) {
          continue;
        }
        handled_edges.insert(e);
        switch (e->type_) {
          case edge_type::TRIP:
          case edge_type::WAIT:
            for (auto const [i, penalty] :
                 utl::enumerate(config.tt_and_waiting_penalties_)) {
              ilp_file << "+ " << std::to_string(penalty * e->tt_) << " f_"
                       << std::to_string(e->id_) << "_" << std::to_string(i)
                       << " ";
            }
            break;
          case edge_type::INTERCHANGE:
            ilp_file << std::to_string(config.transfer_penalty_ + e->tt_)
                     << " f_" << std::to_string(e->id_) << " ";
            break;
          case edge_type::NOROUTE:
            ilp_file << std::to_string(config.no_route_cost_) << " f_"
                     << std::to_string(e->id_) << " ";
            break;
        }
      }
    }
  }
  ilp_file << "\n";

  // CONSTRAINTS
  ilp_file << "Subject To\n";
  // AT LEAST ONE ROUTE F.E. GROUP
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      ilp_file << "+ y_" << std::to_string(pg.id_) << "_"
               << std::to_string(a.id_) << " ";
    }
    ilp_file << "= 1\n";
  }

  // INDICATOR CONSTRAINTS
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        // b1 = 1 -> 2.5 x + 2.3 y + 5.3 z <= 8.1
        ilp_file << "y_" << std::to_string(pg.id_) << "_"
                 << std::to_string(a.id_) << " = 1 -> "
                 << "delta_" << std::to_string(pg.id_) << "_"
                 << std::to_string(e->id_) << " = 1\n";
      }
    }
  }

  // EDGES CAPACITY
  for (auto const& e : handled_edges) {
    for (auto const& pg : edge_to_psgs[e->id_]) {
      ilp_file << "+ " << std::to_string(pg->psg_count_) << " delta_"
               << std::to_string(pg->id_) << "_" << std::to_string(e->id_)
               << " ";
    }
    switch (e->type_) {
      case edge_type::TRIP:
      case edge_type::WAIT:
        for (auto const [i, step] :
             utl::enumerate(config.cost_function_capacity_steps_)) {
          ilp_file << "- f_" << std::to_string(e->id_) << "_"
                   << std::to_string(i) << " ";
        }
        break;
      case edge_type::INTERCHANGE:
      case edge_type::NOROUTE:
        ilp_file << "- f_" << std::to_string(e->id_) << " ";
        break;
    }
    ilp_file << "= 0\n";
  }

  // BOUNDS
  ilp_file << "Bounds\n";
  for (auto const& e : handled_edges) {
    switch (e->type_) {
      case edge_type::TRIP:
      case edge_type::WAIT: {
        uint32_t last_cap_step = 0;
        for (auto const [i, step] :
             utl::enumerate(config.cost_function_capacity_steps_)) {
          auto curr_cap_step = uint32_t(e->soft_cap_boundary_ * step);
          ilp_file << "0 <= f_" << std::to_string(e->id_) << "_"
                   << std::to_string(i)
                   << " <= " << std::to_string(curr_cap_step - last_cap_step)
                   << "\n";
          last_cap_step = curr_cap_step;
        }
        break;
      }
      case edge_type::INTERCHANGE:
      case edge_type::NOROUTE:
        ilp_file << "f_" << std::to_string(e->id_) << " >= 0\n";
        break;
    }
  }

  std::cout << "ILP ENDED" << std::endl;
  ilp_file << "End";
  ilp_file.close();
}
 */
}  // namespace motis::paxassign
