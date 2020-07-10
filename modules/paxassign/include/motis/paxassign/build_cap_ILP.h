#pragma once

#include "motis/paxassign/capacitaty_aware_structs.h"

#include "utl/enumerate.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

void try_gurobi() {
  try {

    // Create an environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", "mip1.log");
    env.start();

    // Create an empty model
    GRBModel model = GRBModel(env);

    // Create variables
    GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
    GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
    GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

    // Set objective: maximize x + y + 2 z
    model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

    // Add constraint: x + 2 y + 3 z <= 4
    model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

    // Add constraint: x + y >= 1
    model.addConstr(x + y >= 1, "c1");

    // Optimize model
    model.optimize();

    std::cout << x.get(GRB_StringAttr_VarName) << " " << x.get(GRB_DoubleAttr_X)
              << std::endl;
    std::cout << y.get(GRB_StringAttr_VarName) << " " << y.get(GRB_DoubleAttr_X)
              << std::endl;
    std::cout << z.get(GRB_StringAttr_VarName) << " " << z.get(GRB_DoubleAttr_X)
              << std::endl;

    std::cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
}

void build_ILP_from_scenario(std::vector<cap_ILP_psg_group> const& passengers,
                             cap_ILP_config const& config,
                             std::string const& scenario_id) {
  try_gurobi();
  std::cout << "BUILD ILP" << std::endl;

  std::ofstream ilp_file("motis/build/rel/ilp_files/" + scenario_id + ".lp");
  // OBJECTIVE PART
  ilp_file << "Minimize\n";
  std::set<cap_ILP_edge const*> handled_edges;
  std::map<uint32_t, std::set<cap_ILP_psg_group const*>> edge_to_psgs;
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        edge_to_psgs[e.id_].insert(&pg);
        if (handled_edges.find(&e) != handled_edges.end()) continue;
        handled_edges.insert(&e);
        switch (e.type_) {
          case edge_type::TRIP:
          case edge_type::WAIT:
            for (auto const [i, penalty] :
                 utl::enumerate(config.tt_and_waiting_penalties_)) {
              ilp_file << "+ " << std::to_string(penalty * e.tt_) << " f_"
                       << std::to_string(e.id_) << "_" << std::to_string(i)
                       << " ";
            }
            break;
          case edge_type::INTERCHANGE:
            ilp_file << std::to_string(config.transfer_penalty_ + e.tt_)
                     << " f_" << std::to_string(e.id_) << " ";
            break;
          case edge_type::NOROUTE:
            ilp_file << std::to_string(config.no_route_cost_) << " f_"
                     << std::to_string(e.id_) << " ";
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
                 << std::to_string(e.id_) << " = 1\n";
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
