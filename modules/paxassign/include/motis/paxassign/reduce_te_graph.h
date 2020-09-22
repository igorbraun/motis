#pragma once

#include "motis/paxassign/node_arc_structs.h"

namespace motis::paxassign {

struct node_arc_psg_group;

struct node_arc_node_info {
  eg_event_node* corresponding_ev_node_;
  bool visited_;
  bool valid_;
  uint16_t interchanges_until_;
  time time_;
  double max_cap_utilization_;
  std::vector<service_class> classes_history_;
};

void explore_te_graph(eg_event_node* start_node,
                      time_expanded_graph const& te_graph) {

  std::map<eg_event_node*, node_arc_node_info> nodes_info;
  std::stack<node_arc_node_info*> stack;

  nodes_info[start_node] =
      node_arc_node_info{start_node, false, true, 0, start_node->time_, 0, {}};
  stack.push(&nodes_info[start_node]);

  while (!stack.empty()) {
    auto curr_node = stack.top();
    stack.pop();

    // step 1: check whether valid or already visited
    if (curr_node->visited_ || !curr_node->valid_) continue;
    curr_node->visited_ = true;

    // step 2: add all adjacent nodes to stack
    for (auto const& oe : curr_node->corresponding_ev_node_->out_edges_) {
      if (nodes_info.find(oe->to_) != nodes_info.end()) {
        continue;
      }
      uint16_t updated_interchanges = (oe->type_ == eg_edge_type::TRAIN_ENTRY)
                                          ? curr_node->interchanges_until_ + 1
                                          : curr_node->interchanges_until_;
      double updated_cap_util =
          curr_node->max_cap_utilization_ > oe->capacity_utilization_
              ? curr_node->max_cap_utilization_
              : oe->capacity_utilization_;
      std::vector<service_class> updated_classes_history(
          curr_node->classes_history_);
      if (oe->service_class_ != service_class::OTHER &&
          (curr_node->classes_history_.empty() ||
           oe->service_class_ != curr_node->classes_history_.back())) {
        updated_classes_history.push_back(oe->service_class_);
      }
      nodes_info[oe->to_] = node_arc_node_info{oe->to_,
                                               false,
                                               true,
                                               updated_interchanges,
                                               oe->to_->time_,
                                               updated_cap_util,
                                               updated_classes_history};
      stack.push(&nodes_info[oe->to_]);
    }
  }
}

void reduce_te_graph(node_arc_psg_group& psg_group,
                     time_expanded_graph const& te_graph) {
  explore_te_graph(psg_group.from_, te_graph);
  std::cout << "TATATAA" << std::endl;
  // TODO: return only edges, which go from a valid node to a valid node
}

}  // namespace motis::paxassign