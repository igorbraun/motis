#pragma once

// RE-IMPLEMENTATION OF GREEDY ASSIGNMENT AS IN THE HALLE-PAPER
namespace motis::paxassign {

std::vector<std::vector<eg_edge*>> greedy_assignment(
    time_expanded_graph const& te_graph,
    std::vector<eg_psg_group> const& eg_psg_groups,
    perceived_tt_config const& config) {
  std::vector<std::vector<eg_edge*>> solution(eg_psg_groups.size());
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    {
      logging::scoped_timer greedy{"greedy algorithm"};
      auto calc_dist_cap_util = [&](eg_edge* e, double curr_dist) {
        if (e->capacity_utilization_ >
            config.cost_function_capacity_steps_.back()) {
          return (double)config.no_route_cost_;
        }
        double transfer_penalty = (e->type_ == eg_edge_type::TRAIN_ENTRY)
                                      ? config.transfer_penalty_
                                      : 0.0;
        auto const it =
            std::lower_bound(config.cost_function_capacity_steps_.begin(),
                             config.cost_function_capacity_steps_.end(),
                             e->capacity_utilization_);
        auto idx =
            std::distance(config.cost_function_capacity_steps_.begin(), it);
        return config.tt_and_waiting_penalties_[idx] * e->cost_ +
               transfer_penalty;
      };
      solution[i] = sssd_dijkstra<double>(eg_psg_groups[i], 0.0,
                                          std::numeric_limits<double>::max(),
                                          te_graph, calc_dist_cap_util);
    }
  }
}
}  // namespace motis::paxassign