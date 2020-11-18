#pragma once

#include <random>

namespace motis::paxassign {

// RE-IMPLEMENTATION OF GREEDY ASSIGNMENT AS IN THE HALLE-PAPER
std::vector<std::vector<eg_edge*>> greedy_assignment(
    time_expanded_graph const& te_graph,
    std::vector<std::vector<bool>> const& nodes_validity,
    std::vector<eg_psg_group> const& eg_psg_groups,
    perceived_tt_config const& config, std::mt19937& rng) {
  std::vector<std::vector<eg_edge*>> solution(eg_psg_groups.size());

  std::vector<int> psgs_indices(eg_psg_groups.size());
  std::iota(std::begin(psgs_indices), std::end(psgs_indices), 0);
  std::shuffle(std::begin(psgs_indices), std::end(psgs_indices), rng);

  auto calc_tt_dist = [&](eg_edge* e, double curr_dist) {
    if (e->capacity_utilization_ >
        config.cost_function_capacity_steps_.back()) {
      return (double)config.no_route_cost_ + curr_dist;
    }
    return e->cost_ + curr_dist;
  };

  for (auto i = 0u; i < psgs_indices.size(); ++i) {
    {
      logging::scoped_timer greedy{"greedy algorithm"};
      solution[psgs_indices[i]] =
          sssd_dijkstra<double>(eg_psg_groups[psgs_indices[i]], 0.0,
                                std::numeric_limits<double>::max(), te_graph,
                                nodes_validity[psgs_indices[i]], calc_tt_dist);
      add_psgs_to_edges(solution[psgs_indices[i]],
                        eg_psg_groups[psgs_indices[i]]);
    }
  }

  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    remove_psgs_from_edges(solution[i], eg_psg_groups[i]);
  }
  return solution;
}
}  // namespace motis::paxassign