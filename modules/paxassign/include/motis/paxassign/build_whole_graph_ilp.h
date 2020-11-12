#pragma once

#include "motis/paxassign/node_arc_structs.h"
#include "motis/paxassign/reduce_te_graph.h"
#include "motis/paxassign/time_expanded_graph.h"

#include "gurobi_c++.h"

namespace motis::paxassign {

std::vector<std::vector<eg_edge*>> build_whole_graph_ilp(
    std::vector<node_arc_psg_group>& psg_groups,
    time_expanded_graph const& te_graph, config const& config,
    schedule const& sched) {
  try {
    GRBEnv env = GRBEnv(true);
    // env.set("LogFile", scenario_id + ".log");
    env.start();
    GRBModel model = GRBModel(env);

    // f.e. psg group k and f.e. edge (i,j) add variable x^k_(i,j)
    std::vector<std::map<eg_edge*, GRBVar>> commodities_edge_vars(
        psg_groups.size());
    std::vector<std::vector<bool>> nodes_validity(psg_groups.size());

    {
      logging::scoped_timer reduce_graph_timer{
          "reduce te graph for passengers"};
      config_graph_reduction reduction_config;
      for (auto i = 0u; i < psg_groups.size(); ++i) {
        nodes_validity[i] =
            reduce_te_graph(psg_groups[i], te_graph, reduction_config, sched);
      }
    }

    {
      logging::scoped_timer var_timer{"ILP: add commodity - variables"};
      for (auto i = 0u; i < psg_groups.size(); ++i) {
        for (auto const& n : te_graph.nodes_) {
          if (!nodes_validity[i][n->id_]) continue;
          for (auto const& e : n->out_edges_) {
            if (!nodes_validity[i][e->to_->id_]) continue;
            auto penalty = (e->type_ == eg_edge_type::TRAIN_ENTRY)
                               ? config.interchange_penalty_
                               : 0;
            commodities_edge_vars[i][e.get()] = model.addVar(
                0.0, 1.0, psg_groups[i].psg_count_ * (e->cost_ + penalty),
                GRB_BINARY,
                std::to_string(i) + "_" + eg_edge_type_to_string(e.get()) +
                    "_" + std::to_string(e->from_->id_) + "_" +
                    std::to_string(e->to_->id_));
          }
        }
      }
    }

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
            if (commodities_edge_vars[i].find(oe.get()) ==
                commodities_edge_vars[i].end()) {
              continue;
            }
            lhs += commodities_edge_vars[i][oe.get()];
            lhs_valid = true;
          }
          for (auto const& ie : n->in_edges_) {
            if (commodities_edge_vars[i].find(ie) ==
                commodities_edge_vars[i].end()) {
              continue;
            }
            lhs -= commodities_edge_vars[i][ie];
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
              lhs += commodities_edge_vars[i][e.get()];
            }
          }
        }
        model.addConstr(lhs, GRB_LESS_EQUAL, config.max_allowed_interchanges_);
      }
    }

    // capacity awareness
    {
      logging::scoped_timer cap_cons_timer{"ILP: capacity constraints"};
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          GRBLinExpr lhs = 0;
          bool lhs_valid = false;
          for (auto i = 0u; i < psg_groups.size(); ++i) {
            if (commodities_edge_vars[i].find(e.get()) ==
                commodities_edge_vars[i].end()) {
              continue;
            }
            lhs += psg_groups[i].psg_count_ * commodities_edge_vars[i][e.get()];
            lhs_valid = true;
          }
          if (lhs_valid) {
            model.addConstr(lhs, GRB_LESS_EQUAL, e->capacity_);
          }
        }
      }
    }

    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    model.optimize();

    int status = model.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
      throw std::runtime_error("node-arc-form ILP model: solution not optimal");
    }

    std::vector<std::vector<eg_edge*>> solution(psg_groups.size());
    for (auto i = 0u; i < psg_groups.size(); ++i) {
      for (auto const& n : te_graph.nodes_) {
        for (auto const& e : n->out_edges_) {
          if (commodities_edge_vars[i].find(e.get()) ==
              commodities_edge_vars[i].end()) {
            continue;
          }
          if (commodities_edge_vars[i][e.get()].get(GRB_DoubleAttr_X) == 1.0) {
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