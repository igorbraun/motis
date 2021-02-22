#pragma once

#include "motis/paxassign/perceived_tt.h"

namespace motis::paxassign {

std::set<eg_event_node*> BFS(eg_edge* previous_edge, uint8_t const max_depth) {
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
  return result;
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

void find_problematic_groups(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& start_solution,
    perceived_tt_config const& config, time_expanded_graph const& te_graph,
    std::vector<std::vector<bool>> const& nodes_validity) {
  std::cout << "Find problematic groups start" << std::endl;

  auto curr_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, start_solution, config);

  std::vector<std::vector<eg_edge*>> solution{start_solution};

  // 1. uses arc with highest utilization
  auto load_stats = e_to_util_and_groups(eg_psg_groups, start_solution);

  int max_arcs_to_test = 1;
  for (auto const& p : load_stats.arc_to_util) {
    ++max_arcs_to_test;
    std::cout << "Max load edge: " << p.second << " for groups: " << std::endl;
    for (auto const val : load_stats.arc_to_psgs_idx.at(p.first)) {
      std::cout << val << std::endl;
    }
    if (max_arcs_to_test > 10) break;
  }

  // 2. has highest delay (assign with greedy but change the order wrt highest
  // delay)
}

template <typename F>
std::vector<std::vector<eg_edge*>> local_search(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& start_solution,
    perceived_tt_config const& config, uint8_t const max_steps,
    std::mt19937& rng, time_expanded_graph const& te_graph,
    std::vector<std::vector<bool>> const& nodes_validity,
    int const max_interchanges, F obj_f) {

  // General description of the local search approach:
  // 1. Randomly choose a group G from the scenario. Take its connection C
  // 2. In C randomly choose a station X (besides the last station and maybe
  // some stations before)
  // 3. Go a couple of edges (parameter) from X in an arbitrary direction. You
  // will result in a station Z (in the current implementation all nodes within
  // a certain depth will be checked)
  // 4. Do two shortest path searches with perceived_tt as objective. Route 1:
  // current station -> Z. Route 2: Z -> end station.
  // 5. Check the objective function of the complete scenario. Better? Take the
  // new connection into the current solution. Otherwise, throw it away
  // 6. Goto 1 until the termination criterion is not reached

  // Possible improvements:
  // 1. Consider the capacity utilization of edges by choosing the group in 1 or
  // the station in 2
  // 2. In step 3 go not only in the future, but also in the past
  // 3. In step 2 do it not only with 1 station X, select 2 random stations and
  // do 3 dijkstra's

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

  int const max_iterations = 100;
  for (auto i = 0u; i < max_iterations; ++i) {
    // Step 1
    auto gr_idx = groups_distr(rng);
    // Step 2
    std::uniform_int_distribution<> curr_psg_distr(0,
                                                   solution[gr_idx].size() - 1);
    auto edge_idx = curr_psg_distr(rng);
    // Step 3
    auto nodes_to_check = BFS(solution[gr_idx][edge_idx], max_steps);
    // Step 4

    remove_psgs_from_edges(solution[gr_idx], eg_psg_groups[gr_idx]);
    for (auto const& n : nodes_to_check) {
      auto route_part_one = sssd_dijkstra<double>(
          eg_psg_groups[gr_idx].from_, n, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, nodes_validity[gr_idx],
          max_interchanges, obj_f);
      auto route_part_two = sssd_dijkstra<double>(
          n, eg_psg_groups[gr_idx].to_, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, nodes_validity[gr_idx],
          max_interchanges, obj_f);
      if (route_part_one.empty() || route_part_two.empty()) {
        continue;
      }
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

      // std::cout << curr_obj << " vs " << new_obj << std::endl;
      if (new_obj < curr_obj) {
        curr_obj = new_obj;
        curr_psg_distr =
            std::uniform_int_distribution<>(0, solution[gr_idx].size() - 1);
      } else {
        solution[gr_idx] = previous_solution;
      }
    }
    add_psgs_to_edges(solution[gr_idx], eg_psg_groups[gr_idx]);
  }

  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    remove_psgs_from_edges(solution[i], eg_psg_groups[i]);
  }
  return solution;
}
}  // namespace motis::paxassign