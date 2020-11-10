#pragma once

#include "motis/paxassign/node_arc_structs.h"

#include <queue>

namespace motis::paxassign {

enum class transport_category {
  LOCAL = 0,
  REGIONAL = 1,
  LONG_DIST = 2,
  NO_TRANSPORT = 3
};

inline std::ostream& operator<<(std::ostream& out,
                                transport_category const tc) {
  switch (tc) {
    case transport_category::LOCAL: return out << "LOCAL";
    case transport_category::REGIONAL: return out << "REGIONAL";
    case transport_category::LONG_DIST: return out << "LONG_DIST";
    case transport_category::NO_TRANSPORT: return out << "NO_TRANSPORT";
  }
  return out;
}

struct config_graph_reduction {
  uint16_t max_interchanges_{2};
  time_t latest_arr_time_{module::get_schedule().schedule_end_};
  double max_cap_utilization_{1.0};
  std::map<service_class, transport_category> class_to_cat_{
      {service_class::AIR, transport_category::LONG_DIST},
      {service_class::ICE, transport_category::LONG_DIST},
      {service_class::IC, transport_category::LONG_DIST},
      {service_class::COACH, transport_category::REGIONAL},
      {service_class::N, transport_category::REGIONAL},
      {service_class::RE, transport_category::REGIONAL},
      {service_class::RB, transport_category::REGIONAL},
      {service_class::S, transport_category::LOCAL},
      {service_class::U, transport_category::LOCAL},
      {service_class::STR, transport_category::LOCAL},
      {service_class::BUS, transport_category::LOCAL},
      {service_class::SHIP, transport_category::LOCAL},
      {service_class::OTHER, transport_category::NO_TRANSPORT}};
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

std::vector<bool> reduce_te_graph(node_arc_psg_group& psg_group,
                                  time_expanded_graph const& te_graph,
                                  config_graph_reduction const& config) {
  std::vector<bool> nodes_validity(te_graph.nodes_.size(), true);

  /*
  // INTERCHANGE FILTER
  {
    logging::scoped_timer interchanges_filter{"interchanges filter"};
    auto calc_dist_interchanges = [](eg_edge* e, auto curr_dist) {
      return (e->type_ == eg_edge_type::TRAIN_ENTRY) ? curr_dist + 1
                                                      : curr_dist;
    };
    auto interchanges_filter_res = dijkstra<uint16_t>(
        psg_group.from_, 0, std::numeric_limits<uint16_t>::max(), te_graph,
        nodes_validity, calc_dist_interchanges);
    filter_nodes<uint16_t>(nodes_validity, interchanges_filter_res,
                           config.max_interchanges_);
    auto valid_count =
        std::accumulate(nodes_validity.begin(), nodes_validity.end(), 0);
    std::cout << "result after inch_filter: " << valid_count << " from "
              << te_graph.nodes_.size() << std::endl;
  }
   */
  /*
 // CAPACITY UTILIZATION FILTER
 {
   logging::scoped_timer cap_util_filter{"capacity utilization filter"};
   auto calc_dist_cap_util = [](eg_edge* e, double curr_dist) {
     return std::max(e->capacity_utilization_, curr_dist);
   };
   auto cap_util_filter_res = dijkstra<double>(
       psg_group.from_, te_graph, nodes_validity, calc_dist_cap_util);
   filter_nodes<double>(nodes_validity, cap_util_filter_res,
                        config.max_cap_utilization_);
   auto valid_count =
       std::accumulate(nodes_validity.begin(), nodes_validity.end(), 0);
   std::cout << "result after cap util: " << valid_count << " from "
             << te_graph.nodes_.size() << std::endl;
 }*/

  // CLASSES FILTER, NEEDS RESULTS OF BOTH, FORWARD AND BACKWARD SEARCHES
  {
    logging::scoped_timer classes_filter{"classes filter"};
    auto calc_dist_classes = [&](eg_edge* e, transport_category curr_cat) {
      if (e->service_class_ == service_class::OTHER) return curr_cat;
      auto e_class = config.class_to_cat_.at(e->service_class_);
      if (curr_cat <= e_class) return e_class;
      return transport_category::NO_TRANSPORT;
    };
    auto classes_forward_res = dijkstra<transport_category>(
        dijkstra_type::FORWARD, psg_group.from_, transport_category::LOCAL,
        transport_category::NO_TRANSPORT, te_graph, nodes_validity,
        calc_dist_classes);
    auto classes_backward_res = dijkstra<transport_category>(
        dijkstra_type::BACKWARD, psg_group.to_, transport_category::LOCAL,
        transport_category::NO_TRANSPORT, te_graph, nodes_validity,
        calc_dist_classes);
    std::vector<transport_category> classes_result;
    std::transform(
        classes_forward_res.begin(), classes_forward_res.end(),
        classes_backward_res.begin(), std::back_inserter(classes_result),
        [](transport_category const& lhs, transport_category const& rhs) {
          return (lhs < rhs) ? lhs : rhs;
        });
    filter_nodes<transport_category>(nodes_validity, classes_result,
                                     transport_category::LONG_DIST);
    auto valid_count =
        std::accumulate(nodes_validity.begin(), nodes_validity.end(), 0);
    std::cout << "result after classes filter: " << valid_count << " from "
              << te_graph.nodes_.size() << std::endl;
  }

  return nodes_validity;
}

}  // namespace motis::paxassign