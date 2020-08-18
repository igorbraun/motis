#pragma once

#include "motis/paxassign/time_expanded_graph.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

struct node_arc_psg_group {
  eg_event_node* from_;
  eg_event_node* to_;
  int psg_count_;
};

void build_whole_graph_ilp(std::vector<node_arc_psg_group> const& psg_groups,
                           time_expanded_graph const& te_graph) {
  // TODO: capacity awareness
  // TODO: how to penalize interchanges?

  try {
    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    // f.e. psg group k and f.e. edge (i,j) add variable x^k_(i,j)
    std::vector<std::map<eg_edge*, GRBVar>> commodities_edge_vars(
        psg_groups.size());
    for (auto i = 0u; i < psg_groups.size(); ++i) {
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          switch (e->type_) {
            case motis::paxassign::eg_edge_type::TRIP:
            case motis::paxassign::eg_edge_type::WAIT: {
              commodities_edge_vars[i][e.get()] = model.addVar(
                  0.0, 1.0, e->to_->time_ - e->from_->time_, GRB_BINARY,
                  std::to_string(i) + "_TW_" + std::to_string(e->from_->id_) +
                      "_" + std::to_string(e->to_->id_));
              break;
            }
            case eg_edge_type::INTERCHANGE: {
              commodities_edge_vars[i][e.get()] = model.addVar(
                  0.0, 1.0, e->transfer_time_, GRB_BINARY,
                  std::to_string(i) + "_I_" + std::to_string(e->from_->id_) +
                      "_" + std::to_string(e->to_->id_));
              break;
            }
            case eg_edge_type::NO_ROUTE: {
              commodities_edge_vars[i][e.get()] = model.addVar(
                  0.0, 1.0, e->transfer_time_, GRB_BINARY,
                  std::to_string(i) + "_NR_" + std::to_string(e->from_->id_) +
                      "_" + std::to_string(e->to_->id_));
              break;
            }
          }
        }
      }
    }

    // conservation constraints
    for (auto i = 0u; i < psg_groups.size(); ++i) {
      for (auto const& n : te_graph.nodes_) {
        if (n->in_edges_.empty() && n->out_edges_.empty()) {
          continue;
        }
        GRBLinExpr lhs = 0;
        for (auto const& oe : n->out_edges_) {
          lhs += commodities_edge_vars[i][oe.get()];
        }
        for (auto const& ie : n->in_edges_) {
          lhs -= commodities_edge_vars[i][ie];
        }
        double rhs = 0.0;
        if (n.get() == psg_groups[i].from_) {
          rhs = 1.0;
        }
        if (n.get() == psg_groups[i].to_) {
          rhs = -1.0;
        }
        model.addConstr(lhs, GRB_EQUAL, rhs);
      }
    }

    // max 6 interchanges
    for (auto i = 0u; i < psg_groups.size(); ++i) {
      GRBLinExpr lhs = 0;
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          if (e->type_ == eg_edge_type::INTERCHANGE) {
            lhs += commodities_edge_vars[i][e.get()];
          }
        }
      }
      model.addConstr(lhs, GRB_LESS_EQUAL, 6);
    }

    // capacity awareness
    for (auto const& n : te_graph.nodes_) {
      for (auto const& e : n->out_edges_) {
        GRBLinExpr lhs = 0;
        for (auto i = 0u; i < psg_groups.size(); ++i) {
          lhs += psg_groups[i].psg_count_* commodities_edge_vars[i][e.get()];
        }
        model.addConstr(lhs, GRB_LESS_EQUAL, e->capacity_);
      }
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.optimize();

    // model.write("motis/build/rel/ilp_files/node_arc.sol");

  } catch (GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Exception during optimization" << std::endl;
  }
}

}  // namespace motis::paxassign