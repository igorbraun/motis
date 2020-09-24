#pragma once

#include "motis/paxassign/node_arc_structs.h"

#include <queue>

namespace motis::paxassign {

struct node_arc_node_info {
  eg_event_node* corresponding_ev_node_;
  bool visited_;
  bool valid_;
  uint16_t interchanges_until_;
  time time_;
  double max_cap_utilization_;
  std::vector<service_class> classes_history_;
};

struct config_graph_reduction {
  uint16_t max_interchanges_{2};
  time_t latest_arr_time_{module::get_schedule().schedule_end_};
  double max_cap_utilization_{1.0};
};

template <typename T>
void print_queue(T& q) {
  while (!q.empty()) {
    std::cout << q.top().second << " ";
    q.pop();
  }
  std::cout << '\n';
}

template <typename T>
void filter_nodes(std::vector<bool>& nodes_validity, std::vector<T> const& dist,
                  T const upper_bound) {
  for (auto i = 0u; i < dist.size(); ++i) {
    if (dist[i] > upper_bound) {
      nodes_validity[i] = false;
    }
  }
}

template <typename T, typename F>
std::vector<T> dijkstra(eg_event_node* start_node,
                        time_expanded_graph const& te_graph,
                        std::vector<bool> const& nodes_validity,
                        F calc_new_dist) {
  typedef std::pair<eg_event_node*, T> node_weight;
  auto cmp_min = [](node_weight left, node_weight right) {
    return left.second > right.second;
  };
  std::priority_queue<node_weight, std::vector<node_weight>, decltype(cmp_min)>
      pq(cmp_min);
  std::vector<T> dist(te_graph.nodes_.size(), std::numeric_limits<T>::max());

  pq.push(std::make_pair(start_node, 0));
  dist[start_node->id_] = 0;

  while (!pq.empty()) {
    auto curr_node = pq.top().first;
    auto curr_dist = pq.top().second;
    pq.pop();
    for (auto const& oe : curr_node->out_edges_) {
      if (!nodes_validity[oe->to_->id_]) {
        continue;
      }
      auto new_dist = calc_new_dist(oe.get(), curr_dist);
      if (new_dist < dist[oe->to_->id_]) {
        dist[oe->to_->id_] = new_dist;
        pq.push(std::make_pair(oe->to_, new_dist));
      }
    }
  }
  return dist;
}

std::vector<bool> reduce_te_graph(node_arc_psg_group& psg_group,
                                  time_expanded_graph const& te_graph,
                                  config_graph_reduction const& config) {
  std::vector<bool> nodes_validity(te_graph.nodes_.size(), true);
  {
    logging::scoped_timer interchanges_filter{"interchanges filter"};
    auto calc_dist_interchanges = [](eg_edge* oe, auto curr_dist) {
      return (oe->type_ == eg_edge_type::TRAIN_ENTRY) ? curr_dist + 1
                                                      : curr_dist;
    };
    auto interchanges_filter_res = dijkstra<uint16_t>(
        psg_group.from_, te_graph, nodes_validity, calc_dist_interchanges);
    filter_nodes<uint16_t>(nodes_validity, interchanges_filter_res,
                           config.max_interchanges_);
    auto valid_count =
        std::accumulate(nodes_validity.begin(), nodes_validity.end(), 0);
    std::cout << "result after inch_filter: " << valid_count << std::endl;
  }
  {
    logging::scoped_timer cap_util_filter{"capacity utilization filter"};
    auto calc_dist_cap_util = [](eg_edge* oe, double curr_dist) {
      return std::max(oe->capacity_utilization_, curr_dist);
    };
    auto cap_util_filter_res = dijkstra<double>(
        psg_group.from_, te_graph, nodes_validity, calc_dist_cap_util);
    filter_nodes<double>(nodes_validity, cap_util_filter_res,
                         config.max_cap_utilization_);
    auto valid_count =
        std::accumulate(nodes_validity.begin(), nodes_validity.end(), 0);
    std::cout << "result after cap util: " << valid_count << std::endl;
  }
  return nodes_validity;
}

}  // namespace motis::paxassign