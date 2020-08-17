#pragma once

#include "motis/paxassign/time_expanded_graph.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

void build_whole_graph_ilp(eg_event_node* from, eg_event_node* to,
                           time_expanded_graph& te_graph) {
  // TODO: no route edges to te_graph in previous function
  // TODO: count used interchange edges

  try {
    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    // f.e. psg group k and f.e. edge (i,j) add variable x^k_(i,j)
    std::map<eg_edge*, GRBVar> commodity_edge_vars;
    for (auto const& n : te_graph.nodes_) {
      for (auto const& e : n->out_edges_) {
        switch (e->type_) {
          case motis::paxassign::eg_edge_type::TRIP:
          case motis::paxassign::eg_edge_type::WAIT: {
            commodity_edge_vars[e.get()] = model.addVar(
                0.0, 1.0, e->to_->time_ - e->from_->time_, GRB_BINARY,
                "TW_" + std::to_string(e->from_->id_) + "_" +
                    std::to_string(e->to_->id_));
            break;
          }
          case eg_edge_type::INTERCHANGE: {
            commodity_edge_vars[e.get()] =
                model.addVar(0.0, 1.0, e->transfer_time_, GRB_BINARY,
                             "I_" + std::to_string(e->from_->id_) + "_" +
                                 std::to_string(e->to_->id_));
            break;
          }
          case eg_edge_type::NO_ROUTE: break;
        }
      }
    }

    // conservation constraints
    for (auto const& n : te_graph.nodes_) {
      if (n->in_edges_.empty() && n->out_edges_.empty()) {
        continue;
      }
      GRBLinExpr lhs = 0;
      for (auto const& oe : n->out_edges_) {
        lhs += commodity_edge_vars[oe.get()];
      }
      for (auto const& ie : n->in_edges_) {
        lhs -= commodity_edge_vars[ie];
      }
      double rhs = 0.0;
      if (n.get() == from) {
        rhs = 1.0;
      }
      if (n.get() == to) {
        rhs = -1.0;
      }
      model.addConstr(lhs, GRB_EQUAL, rhs);
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.optimize();

  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
}

}  // namespace motis::paxassign