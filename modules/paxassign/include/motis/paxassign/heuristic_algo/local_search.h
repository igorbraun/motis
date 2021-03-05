#pragma once

#include "motis/paxassign/perceived_tt.h"

namespace motis::paxassign {

std::vector<eg_event_node*> BFS(eg_edge* previous_edge,
                                uint8_t const max_depth) {
  typedef std::pair<eg_event_node*, int8_t> node_depth;
  std::set<eg_event_node*> result;

  std::stack<node_depth> stack;
  stack.push(std::make_pair(previous_edge->from_, 0));

  while (!stack.empty()) {
    auto curr_node = stack.top().first;
    auto depth = stack.top().second;
    stack.pop();

    for (auto const& oe : curr_node->out_edges_) {
      if (oe.get() == previous_edge) continue;
      if (depth < max_depth) {
        result.insert(oe->to_);
        stack.push(std::make_pair(oe->to_, depth + 1));
      }
    }
  }
  return std::vector<eg_event_node*>(result.begin(), result.end());
}

struct arc_load_stat {
  std::map<eg_edge*, std::set<int>> arc_to_psgs_idx;
  std::vector<std::pair<eg_edge*, double>> arc_to_util;
};

arc_load_stat e_to_util_and_groups(
    std::vector<eg_psg_group> const& psg_groups,
    std::vector<std::vector<eg_edge*>> const& solution) {
  arc_load_stat arcs_stat;

  std::map<eg_edge*, double> assgmts;
  for (auto i = 0u; i < solution.size(); ++i) {
    for (auto const& e : solution[i]) {
      if (assgmts.find(e) == assgmts.end()) {
        assgmts[e] = psg_groups[i].psg_count_;
      } else {
        assgmts[e] += psg_groups[i].psg_count_;
      }
      arcs_stat.arc_to_psgs_idx[e].insert(i);
    }
  }

  for (auto const& map_entry : assgmts) {
    double new_util =
        (double)(map_entry.first->passengers_ + map_entry.second) /
        map_entry.first->soft_cap_boundary_;
    arcs_stat.arc_to_util.push_back({map_entry.first, new_util});
  }

  std::sort(std::begin(arcs_stat.arc_to_util), std::end(arcs_stat.arc_to_util),
            [](std::pair<eg_edge*, double>& left,
               std::pair<eg_edge*, double>& right) {
              return left.second > right.second;
            });

  return arcs_stat;
}

std::vector<int> sort_by_max_load(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& solution) {
  // 1. use arc with highest utilization
  std::vector<int> result;
  auto load_stats = e_to_util_and_groups(eg_psg_groups, solution);
  for (auto const& p : load_stats.arc_to_util) {
    for (auto const gr_id : load_stats.arc_to_psgs_idx.at(p.first)) {
      if (std::find(result.begin(), result.end(), gr_id) == result.end()) {
        result.push_back(gr_id);
      }
    }
  }
  return result;
}

std::vector<int> no_routes_pgs(
    std::vector<std::vector<eg_edge*>> const& solution) {
  // 2. passengers with no-route
  std::vector<int> no_route_ids;
  for (auto i = 0u; i < solution.size(); ++i) {
    auto no_route_edge = std::find_if(
        solution[i].begin(), solution[i].end(),
        [](eg_edge* e) { return e->type_ == eg_edge_type::NO_ROUTE; });
    if (no_route_edge != solution[i].end()) {
      no_route_ids.push_back(i);
    }
  }
  return no_route_ids;
}

std::vector<int> sorted_by_delays(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& solution) {
  // 3. has highest delays
  std::vector<std::pair<int, int>> delay_to_id;
  for (auto i = 0u; i < solution.size(); ++i) {
    auto no_route_edge = std::find_if(
        solution[i].begin(), solution[i].end(),
        [](eg_edge* e) { return e->type_ == eg_edge_type::NO_ROUTE; });
    if (no_route_edge != solution[i].end()) {
      continue;
    }
    delay_to_id.push_back(
        {solution[i].back()->from_->time_ -
             eg_psg_groups[i].cpg_.groups_[0]->planned_arrival_time_,
         i});
  }
  std::sort(std::begin(delay_to_id), std::end(delay_to_id),
            [](std::pair<int, int>& left, std::pair<int, int>& right) {
              return left.first > right.first;
            });
  return utl::to_vec(delay_to_id,
                     [&](std::pair<int, int> const& p) { return p.second; });
}

void find_problematic_groups(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& start_solution,
    std::vector<int>& load_based_order, std::vector<int>& delay_based_order) {
  auto load_descending = sort_by_max_load(eg_psg_groups, start_solution);
  auto no_route_pgs = no_routes_pgs(start_solution);
  auto delay_descending = sorted_by_delays(eg_psg_groups, start_solution);

  // CASE I: loads based
  load_based_order.insert(load_based_order.end(), no_route_pgs.begin(),
                          no_route_pgs.end());
  for (auto const gr_id : load_descending) {
    if (std::find(load_based_order.begin(), load_based_order.end(), gr_id) ==
        load_based_order.end()) {
      load_based_order.push_back(gr_id);
    }
  }
  if (load_based_order.size() != eg_psg_groups.size()) {
    throw std::runtime_error(
        "load based order: size of vec not equal to number of groups");
  }

  // CASE 2: delay based
  delay_based_order.insert(delay_based_order.end(), no_route_pgs.begin(),
                           no_route_pgs.end());
  for (auto const gr_id : delay_descending) {
    if (std::find(delay_based_order.begin(), delay_based_order.end(), gr_id) ==
        delay_based_order.end()) {
      delay_based_order.push_back(gr_id);
    }
  }
  if (delay_based_order.size() != eg_psg_groups.size()) {
    throw std::runtime_error(
        "delay based order: size of vec not equal to number of groups");
  }
}

template <typename F>
std::vector<std::vector<eg_edge*>> local_search(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& start_solution,
    perceived_tt_config const& config, uint8_t const max_steps,
    std::mt19937& rng, time_expanded_graph const& te_graph,
    int const max_interchanges, uint16_t const allowed_delay, F obj_f) {

  // General description of the local search approach:
  // 1. Randomly choose a group G from the scenario. Take its connection C
  // 2. In C randomly choose a station X (besides the last station and maybe
  // some stations before)
  // 3. Go a couple of edges (parameter) from X in an arbitrary direction. You
  // will result in a station Z (in the current implementation all nodes
  // within a certain depth will be checked)
  // 4. Do two shortest path searches with perceived_tt as objective. Route 1:
  // current station -> Z. Route 2: Z -> end station.
  // 5. Check the objective function of the complete scenario. Better? Take
  // the new connection into the current solution. Otherwise, throw it away
  // 6. Goto 1 until the termination criterion is not reached

  auto curr_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, start_solution, config);

  if (eg_psg_groups.size() == 1)
    return std::vector<std::vector<eg_edge*>>{start_solution};

  std::vector<std::vector<eg_edge*>> solution{start_solution};
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    add_psgs_to_edges(solution[i], eg_psg_groups[i]);
  }

  // Initialization
  std::uniform_int_distribution<> groups_distr(0, eg_psg_groups.size() - 1);
  std::uniform_int_distribution<> steps_to_go(1, max_steps);
  std::uniform_real_distribution<> stops_distr(0.0, 1.0);

  int const max_iterations = eg_psg_groups.size();
  for (auto i = 0u; i < max_iterations; ++i) {
    // Step 1
    auto gr_idx = groups_distr(rng);
    time latest_time =
        eg_psg_groups[gr_idx].cpg_.groups_.back()->planned_arrival_time_ +
        allowed_delay;
    // Step 2
    int edge_idx = (int)((solution[gr_idx].size() - 1) * stops_distr(rng));
    // Step 3
    auto nodes_to_check = BFS(solution[gr_idx][edge_idx], max_steps);
    std::shuffle(std::begin(nodes_to_check), std::end(nodes_to_check), rng);
    // Step 4
    remove_psgs_from_edges(solution[gr_idx], eg_psg_groups[gr_idx]);
    int handled_nodes = 0;
    for (auto const& n : nodes_to_check) {
      ++handled_nodes;
      auto route_part_one = sssd_dijkstra<double>(
          eg_psg_groups[gr_idx].from_, n, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, max_interchanges,
          latest_time, obj_f);
      if (route_part_one.empty()) continue;
      auto route_part_two = sssd_dijkstra<double>(
          n, eg_psg_groups[gr_idx].to_, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, max_interchanges,
          latest_time, obj_f);
      if (route_part_two.empty()) continue;
      route_part_one.insert(route_part_one.end(), route_part_two.begin(),
                            route_part_two.end());
      auto const num_interchanges = std::count_if(
          route_part_one.begin(), route_part_one.end(),
          [](eg_edge* e) { return e->type_ == eg_edge_type::TRAIN_ENTRY; });
      if (num_interchanges > max_interchanges) {
        continue;
      }

      // Step 5
      auto previous_solution{solution[gr_idx]};
      solution[gr_idx] = route_part_one;

      add_psgs_to_edges(route_part_one, eg_psg_groups[gr_idx]);
      for (auto rem_idx = 0u; rem_idx < eg_psg_groups.size(); ++rem_idx) {
        remove_psgs_from_edges(solution[rem_idx], eg_psg_groups[rem_idx]);
      }
      auto new_obj = piecewise_linear_convex_perceived_tt_node_arc(
          eg_psg_groups, solution, config);
      for (auto add_idx = 0u; add_idx < eg_psg_groups.size(); ++add_idx) {
        add_psgs_to_edges(solution[add_idx], eg_psg_groups[add_idx]);
      }
      remove_psgs_from_edges(route_part_one, eg_psg_groups[gr_idx]);

      if (new_obj < curr_obj) {
        curr_obj = new_obj;
      } else {
        solution[gr_idx] = previous_solution;
      }
      if (handled_nodes >= 10) break;
    }
    add_psgs_to_edges(solution[gr_idx], eg_psg_groups[gr_idx]);
  }

  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    remove_psgs_from_edges(solution[i], eg_psg_groups[i]);
  }
  return solution;
}
}  // namespace motis::paxassign