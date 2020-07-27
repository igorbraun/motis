#pragma once

#include "motis/paxassign/capacitaty_aware_structs.h"

#include "utl/enumerate.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

std::vector<uint16_t> build_ILP_from_scenario_API(
    std::vector<cap_ILP_psg_group> const& passengers,
    cap_ILP_config const& config, std::string const& scenario_id) {
  try {
    std::set<cap_ILP_edge const*> handled_edges;
    std::map<uint32_t, std::set<cap_ILP_psg_group const*>> edge_to_psgs;

    for (auto const& pg : passengers) {
      for (auto const& a : pg.alternatives_) {
        for (auto const& e : a.edges_) {
          edge_to_psgs[e->id_].insert(&pg);
          handled_edges.insert(e);
        }
      }
    }

    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    // EDGE VARIABLES
    auto pr =
        std::max_element(std::begin(handled_edges), std::end(handled_edges),
                         [](cap_ILP_edge const* e1, cap_ILP_edge const* e2) {
                           return e1->id_ < e2->id_;
                         });
    std::vector<std::vector<GRBVar>> edge_cost_vars((*pr)->id_ + 1);
    for (auto const& e : handled_edges) {
      switch (e->type_) {
        case edge_type::TRIP: {
          uint32_t last_cap_step = 0;
          for (auto const [i, penalty] :
               utl::enumerate(config.tt_and_waiting_penalties_)) {
            auto curr_cap_step = uint32_t(
                e->capacity_ * config.cost_function_capacity_steps_[i]);
            edge_cost_vars[e->id_].push_back(model.addVar(
                0.0, curr_cap_step - last_cap_step, penalty * e->tt_,
                GRB_INTEGER,
                "T_f_" + std::to_string(e->id_) + "_" + std::to_string(i)));
            last_cap_step = curr_cap_step;
          }
          break;
        }
        case edge_type::WAIT: {
          uint32_t last_cap_step = 0;
          for (auto const [i, penalty] :
               utl::enumerate(config.tt_and_waiting_penalties_)) {
            auto curr_cap_step = uint32_t(
                e->capacity_ * config.cost_function_capacity_steps_[i]);
            edge_cost_vars[e->id_].push_back(model.addVar(
                0.0, curr_cap_step - last_cap_step, penalty * e->tt_,
                GRB_INTEGER,
                "W_f_" + std::to_string(e->id_) + "_" + std::to_string(i)));
            last_cap_step = curr_cap_step;
          }
          break;
        }
        case edge_type::INTERCHANGE: {
          edge_cost_vars[e->id_].push_back(
              model.addVar(0.0, std::numeric_limits<double>::max(),
                           config.transfer_penalty_ + e->tt_, GRB_INTEGER,
                           "I_f_" + std::to_string(e->id_)));
          break;
        }
        case edge_type::NOROUTE: {
          edge_cost_vars[e->id_].push_back(
              model.addVar(0.0, std::numeric_limits<double>::max(), 0,
                           GRB_INTEGER, "NR_f_" + std::to_string(e->id_)));
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
            0.0, 1.0, a.associated_waiting_time_, GRB_BINARY,
            "y_" + std::to_string(pg.id_) + "_" + std::to_string(a.id_)));
      }
    }

    // PSG-EDGE USAGE VARIABLES
    std::vector<std::vector<std::pair<uint32_t, GRBVar>>> edge_usage_vars(
        (*pr)->id_ + 1);
    for (auto const& e : handled_edges) {
      for (auto const& pg : edge_to_psgs[e->id_]) {
        edge_usage_vars[e->id_].push_back(
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
          for (auto const& p_to_edge : edge_usage_vars[e->id_]) {
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
      for (auto const& p_to_edge : edge_usage_vars[e->id_]) {
        auto pg_it = std::find_if(
            std::begin(passengers), std::end(passengers),
            [&](auto const& p) { return p.id_ == p_to_edge.first; });
        lhs += pg_it->psg_count_ * p_to_edge.second;
      }
      for (auto const& ec_v : edge_cost_vars[e->id_]) {
        lhs -= ec_v;
      }
      model.addConstr(lhs, GRB_EQUAL, 0.0);
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.write("motis/build/rel/ilp_files/" + scenario_id + ".lp");

    model.optimize();

    model.write("motis/build/rel/ilp_files/" + scenario_id + ".sol");

    int status = model.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
      throw std::runtime_error("capacitated ILP model: solution not optimal");
    }

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

    std::vector<uint16_t> alt_to_use;
    for (auto const& pg : passengers) {
      uint16_t i = 0;
      for (auto const& curr_alt : alt_route_vars[pg.id_]) {
        if (curr_alt.get(GRB_DoubleAttr_X) == 1.0) {
          alt_to_use.push_back(i);
          break;
        }
      }
    }
    return alt_to_use;

  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
  return {};
}

void build_ILP_from_scenario(std::vector<cap_ILP_psg_group> const& passengers,
                             cap_ILP_config const& config,
                             std::string const& scenario_id) {
  std::ofstream ilp_file("motis/build/rel/ilp_files/" + scenario_id + ".lp");
  // OBJECTIVE PART
  ilp_file << "Minimize\n";
  std::set<cap_ILP_edge const*> handled_edges;
  std::map<uint32_t, std::set<cap_ILP_psg_group const*>> edge_to_psgs;
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        edge_to_psgs[e->id_].insert(&pg);
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
          auto curr_cap_step = uint32_t(e->capacity_ * step);
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
}  // namespace motis::paxassign
