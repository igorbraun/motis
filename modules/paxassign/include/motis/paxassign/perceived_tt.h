#pragma once

namespace motis::paxassign {

double calc_perc_tt(std::vector<eg_edge*> const& solution,
                    perceived_tt_config const& perc_tt_config) {
  double perc_tt = 0.0;
  for (auto const& e : solution) {
    if (e->type_ == eg_edge_type::NO_ROUTE)
      return perc_tt_config.no_route_cost_;
    if (e->capacity_utilization_ >
        perc_tt_config.cost_function_capacity_steps_.back()) {
      return perc_tt_config.no_route_cost_;
    }
    double transfer_penalty = (e->type_ == eg_edge_type::TRAIN_ENTRY)
                                  ? perc_tt_config.transfer_penalty_
                                  : 0.0;
    auto const it =
        std::lower_bound(perc_tt_config.cost_function_capacity_steps_.begin(),
                         perc_tt_config.cost_function_capacity_steps_.end(),
                         e->capacity_utilization_);
    auto idx =
        std::distance(perc_tt_config.cost_function_capacity_steps_.begin(), it);
    perc_tt += perc_tt_config.tt_and_waiting_penalties_[idx] * e->cost_ +
               transfer_penalty;
  }
  return perc_tt;
}

double calc_perc_tt_for_scenario(
    std::vector<eg_psg_group> const& eg_psg_groups,
    std::vector<std::vector<eg_edge*>> const& scenario_solution,
    perceived_tt_config const& perc_tt_config) {
  double cumulative_perc_tt = 0;
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    double curr_sol_cost = calc_perc_tt(scenario_solution[i], perc_tt_config);
    cumulative_perc_tt =
        cumulative_perc_tt + eg_psg_groups[i].psg_count_ * curr_sol_cost;
  }
  return cumulative_perc_tt;
}

double get_obj_after_assign(std::vector<eg_psg_group> const& eg_psg_groups,
                            std::vector<std::vector<eg_edge*>> const& solution,
                            perceived_tt_config const& perc_tt_config) {
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    add_psgs_to_edges(solution[i], eg_psg_groups[i]);
  }
  double obj =
      calc_perc_tt_for_scenario(eg_psg_groups, solution, perc_tt_config);
  for (auto i = 0u; i < eg_psg_groups.size(); ++i) {
    remove_psgs_from_edges(solution[i], eg_psg_groups[i]);
  }
  return obj;
}

}  // namespace motis::paxassign