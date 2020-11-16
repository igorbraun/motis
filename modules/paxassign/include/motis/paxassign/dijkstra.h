#pragma once

#include <tuple>

namespace motis::paxassign {

enum class dijkstra_type { FORWARD = 0, BACKWARD = 1 };

template <typename T, typename F>
std::vector<T> dijkstra(dijkstra_type const& type, eg_event_node* start_node,
                        T const start_node_init, T const dist_vec_init,
                        time_expanded_graph const& te_graph,
                        std::vector<bool> const& nodes_validity,
                        F calc_new_dist) {
  typedef std::pair<eg_event_node*, T> node_weight;
  auto cmp_min = [](node_weight left, node_weight right) {
    return left.second > right.second;
  };
  std::priority_queue<node_weight, std::vector<node_weight>, decltype(cmp_min)>
      pq(cmp_min);
  std::vector<T> dist(te_graph.nodes_.size(), dist_vec_init);

  pq.push(std::make_pair(start_node, start_node_init));
  dist[start_node->id_] = start_node_init;

  while (!pq.empty()) {
    auto curr_node = pq.top().first;
    auto curr_dist = pq.top().second;
    pq.pop();
    if (type == dijkstra_type::FORWARD) {
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
    } else {
      for (auto const& ie : curr_node->in_edges_) {
        if (!nodes_validity[ie->from_->id_]) {
          continue;
        }
        auto new_dist = calc_new_dist(ie, curr_dist);
        if (new_dist < dist[ie->from_->id_]) {
          dist[ie->from_->id_] = new_dist;
          pq.push(std::make_pair(ie->from_, new_dist));
        }
      }
    }
  }
  return dist;
}

template <typename T, typename F>
std::vector<eg_edge*> sssd_dijkstra(eg_psg_group const& pg,
                                    T const start_node_init,
                                    T const dist_vec_init,
                                    time_expanded_graph const& te_graph,
                                    F calc_new_dist) {
  std::map<eg_event_node*, eg_event_node*> predecessors;
  typedef std::pair<eg_event_node*, T> node_weight;
  auto cmp_min = [](node_weight left, node_weight right) {
    return left.second > right.second;
  };
  std::priority_queue<node_weight, std::vector<node_weight>, decltype(cmp_min)>
      pq(cmp_min);
  std::vector<T> dist(te_graph.nodes_.size(), dist_vec_init);

  pq.push(std::make_pair(pg.from_, start_node_init));
  dist[pg.from_->id_] = start_node_init;

  while (!pq.empty()) {
    auto curr_node = pq.top().first;
    auto curr_dist = pq.top().second;
    pq.pop();
    if (curr_node == pg.to_) break;
    for (auto const& oe : curr_node->out_edges_) {
      if (oe->capacity_ < pg.psg_count_) continue;
      auto new_dist = calc_new_dist(oe.get(), curr_dist);
      if (new_dist < dist[oe->to_->id_]) {
        dist[oe->to_->id_] = new_dist;
        pq.push(std::make_pair(oe->to_, new_dist));
        //predecessors.at(oe->to_) = curr_node;
      }
    }
  }
  std::vector<eg_edge*> solution;
  // TODO: reconstruct the solution and return it
  return solution;
}
}  // namespace motis::paxassign