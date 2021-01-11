#pragma once

#include "motis/paxassign/perceived_tt.h"

namespace motis::paxassign {

std::set<eg_event_node*> BFS(eg_event_node* start_node, eg_edge* previous_edge,
                             uint8_t const max_depth) {
  typedef std::pair<eg_event_node*, int8_t> node_depth;
  std::set<eg_event_node*> result;

  std::stack<node_depth> stack;
  stack.push(std::make_pair(start_node, 0));

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

template <typename F>
std::vector<std::vector<eg_edge*>> local_search(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& start_solution,
    perceived_tt_config const& config, uint8_t const max_steps,
    std::mt19937& rng, time_expanded_graph const& te_graph,
    std::vector<std::vector<bool>> const& nodes_validity,
    int const max_interchanges, F obj_f) {
  if (eg_psg_groups.size() == 1) return std::vector<std::vector<eg_edge*>>();
  std::vector<std::vector<eg_edge*>> solution{start_solution};
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    add_psgs_to_edges(solution[i], eg_psg_groups[i]);
  }
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

  // Initialization
  std::uniform_int_distribution<> groups_distr(0, eg_psg_groups.size() - 1);
  std::vector<std::uniform_int_distribution<>> edges_distr;
  std::uniform_int_distribution<> steps_to_go(1, max_steps);

  auto curr_obj = piecewise_linear_convex_perceived_tt_node_arc(
      eg_psg_groups, solution, config);

  for (auto const& sol : solution) {
    // at this point all the edges can be chosen
    edges_distr.emplace_back(0, sol.size() - 1);
  }

  int const max_iterations = 1000;
  for (auto i = 0u; i < max_iterations; ++i) {
    // Step 1
    auto gr_idx = groups_distr(rng);
    // Step 2
    auto edge_idx = edges_distr[gr_idx](rng);
    // Step 3
    auto nodes_to_check = BFS(solution[gr_idx][edge_idx]->from_,
                              solution[gr_idx][edge_idx], max_steps);
    // Step 4
    for (auto const& n : nodes_to_check) {
      auto route_part_one = sssd_dijkstra<double>(
          eg_psg_groups[gr_idx].from_, n, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, nodes_validity[gr_idx],
          max_interchanges, obj_f);
      auto route_part_two = sssd_dijkstra<double>(
          n, eg_psg_groups[gr_idx].to_, eg_psg_groups[gr_idx].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph, nodes_validity[gr_idx],
          max_interchanges, obj_f);
      if (route_part_one.empty() || route_part_two.empty()) continue;
      route_part_one.insert(route_part_one.end(), route_part_two.begin(),
                            route_part_two.end());
      // TODO: check for cumulative interchanges, max driving time and co

      // re-assign psgs to new edges before calculating objective
      auto previous_solution{solution[gr_idx]};
      reassign_psgs(previous_solution, route_part_one, eg_psg_groups[gr_idx]);
      // Step 5
      solution[gr_idx] = route_part_one;
      auto new_obj = piecewise_linear_convex_perceived_tt_node_arc(
          eg_psg_groups, solution, config);
      std::cout << curr_obj << " vs " << new_obj << std::endl;
      if (new_obj < curr_obj) {
        curr_obj = new_obj;
      } else {
        reassign_psgs(route_part_one, previous_solution, eg_psg_groups[gr_idx]);
        solution[gr_idx] = previous_solution;
      }
    }
  }

  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    remove_psgs_from_edges(solution[i], eg_psg_groups[i]);
  }
  return solution;
}
}  // namespace motis::paxassign