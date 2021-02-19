#pragma once

#include <random>

namespace motis::paxassign {

// RE-IMPLEMENTATION OF GREEDY ASSIGNMENT AS IN THE HALLE-PAPER
// TO GET RESULT AS IN THE HALLE-PAPER USE calc_tt_dist as objectve
template <typename F>
std::vector<std::vector<eg_edge*>> greedy_assignment(
    time_expanded_graph const& te_graph,
    std::vector<std::vector<bool>> const& nodes_validity,
    int const max_interchanges, std::vector<eg_psg_group> const& eg_psg_groups,
    std::mt19937& rng, F obj_f) {
  std::vector<std::vector<eg_edge*>> solution(eg_psg_groups.size());

  std::vector<int> psgs_indices(eg_psg_groups.size());
  std::iota(std::begin(psgs_indices), std::end(psgs_indices), 0);
  std::shuffle(std::begin(psgs_indices), std::end(psgs_indices), rng);

  for (auto i = 0u; i < psgs_indices.size(); ++i) {
    {
      logging::scoped_timer greedy{"greedy algorithm"};
      solution[psgs_indices[i]] = sssd_dijkstra<double>(
          eg_psg_groups[psgs_indices[i]].from_,
          eg_psg_groups[psgs_indices[i]].to_,
          eg_psg_groups[psgs_indices[i]].psg_count_, 0.0,
          std::numeric_limits<double>::max(), te_graph,
          nodes_validity[psgs_indices[i]], max_interchanges, obj_f);
      if (solution[psgs_indices[i]].empty()) {
        throw std::runtime_error(
            "GREEDY ASSIGNMENT, DIJKSTRA DIDN'T FIND ANY ROUTE");
      }
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