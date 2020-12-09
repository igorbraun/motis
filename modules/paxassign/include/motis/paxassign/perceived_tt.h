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

std::map<cap_ILP_edge*, double> calculate_assignments_to_edges_halle(
    std::vector<cap_ILP_psg_group> const& cap_ILP_scenario,
    std::vector<std::pair<ilp_psg_id, alt_idx>> const& assignment) {
  std::map<cap_ILP_edge*, double> assgmts;
  for (auto const& asg : assignment) {
    auto corresponding_pg = std::find_if(
        cap_ILP_scenario.begin(), cap_ILP_scenario.end(),
        [&asg](cap_ILP_psg_group const& pg) { return pg.id_ == asg.first; });
    auto route = corresponding_pg->alternatives_[asg.second];
    for (auto const& e : route.edges_) {
      if (assgmts.find(e) == assgmts.end()) {
        assgmts[e] = corresponding_pg->psg_count_;
      } else {
        assgmts[e] += corresponding_pg->psg_count_;
      }
    }
  }
  return assgmts;
}

double piecewise_linear_convex_perceived_tt_halle(
    std::vector<cap_ILP_psg_group> const& cap_ILP_scenario,
    std::vector<std::pair<ilp_psg_id, alt_idx>> const& assignment,
    perceived_tt_config const& perc_tt_config,
    std::map<std::string, std::tuple<double, double, double, double>> const&) {
  double result = 0;

  double cum_no_route = 0;
  double cum_trips_waits = 0;

  auto assgmts =
      calculate_assignments_to_edges_halle(cap_ILP_scenario, assignment);
  std::set<cap_ILP_edge*> handled_edges;
  for (auto const& asg : assignment) {
    auto corresponding_pg = std::find_if(
        cap_ILP_scenario.begin(), cap_ILP_scenario.end(),
        [&asg](cap_ILP_psg_group const& pg) { return pg.id_ == asg.first; });
    auto route = corresponding_pg->alternatives_[asg.second];
    result += corresponding_pg->psg_count_ * route.associated_waiting_time_;
  }
  for (auto const& asg : assgmts) {
    auto const& e = asg.first;
    if (e->type_ == edge_type::NOROUTE) {
      result += asg.second * perc_tt_config.no_route_cost_;
      cum_no_route += asg.second * perc_tt_config.no_route_cost_;
      continue;
    }
    if (e->type_ == edge_type::INTERCHANGE) {
      result += asg.second * (perc_tt_config.transfer_penalty_ + e->tt_);
      continue;
    }
    if (e->type_ == edge_type::TRIP || e->type_ == edge_type::WAIT) {
      auto remaining_assgmnts = asg.second;
      int64_t last_cap_step = 0;
      for (auto const [i, penalty] :
           utl::enumerate(perc_tt_config.tt_and_waiting_penalties_)) {
        auto curr_cap_step =
            uint64_t(e->soft_cap_boundary_ *
                     perc_tt_config.cost_function_capacity_steps_[i]);
        uint64_t remaining_cap = 0;
        if (e->passengers_ > curr_cap_step) {
          remaining_cap = 0;
        } else {
          if (e->passengers_ > last_cap_step) {
            remaining_cap = curr_cap_step - e->passengers_;
          } else {
            remaining_cap = curr_cap_step - last_cap_step;
          }
        }
        last_cap_step = curr_cap_step;
        if (remaining_assgmnts > remaining_cap) {
          result += remaining_cap *
                    (perc_tt_config.tt_and_waiting_penalties_[i] * e->tt_);
          cum_trips_waits +=
              remaining_cap *
              (perc_tt_config.tt_and_waiting_penalties_[i] * e->tt_);
          remaining_assgmnts -= remaining_cap;
        } else {
          result += remaining_assgmnts *
                    (perc_tt_config.tt_and_waiting_penalties_[i] * e->tt_);
          cum_trips_waits +=
              remaining_assgmnts *
              (perc_tt_config.tt_and_waiting_penalties_[i] * e->tt_);
          remaining_assgmnts = 0;
        }
      }
    }
  }

  std::cout << "CUM NO ROUTE: " << cum_no_route << std::endl;
  std::cout << "CUM TRIPS AND WAITS: " << cum_trips_waits << std::endl;

  return result;
}

std::map<eg_edge*, double> calculate_assignments_to_edges_node_arc(
    std::vector<eg_psg_group> const& psg_groups,
    std::vector<std::vector<eg_edge*>> const& solution) {
  std::map<eg_edge*, double> assgmts;
  for (auto i = 0u; i < solution.size(); ++i) {
    for (auto const& e : solution[i]) {
      if (assgmts.find(e) == assgmts.end()) {
        assgmts[e] = psg_groups[i].psg_count_;
      } else {
        assgmts[e] += psg_groups[i].psg_count_;
      }
    }
  }
  return assgmts;
}

double piecewise_linear_convex_perceived_tt_node_arc(
    std::vector<eg_psg_group> const& psg_groups,
    std::vector<std::vector<eg_edge*>> const& solution,
    perceived_tt_config const& perc_tt_config) {
  double result = 0;

  double cum_no_route = 0.0;
  double cum_trips_waits = 0.0;

  auto assgmts = calculate_assignments_to_edges_node_arc(psg_groups, solution);

  for (auto const& asg : assgmts) {
    auto const& e = asg.first;
    if (e->type_ == eg_edge_type::NO_ROUTE) {
      result += asg.second * perc_tt_config.no_route_cost_;
      cum_no_route += asg.second * perc_tt_config.no_route_cost_;
      continue;
    }
    if (e->type_ == eg_edge_type::TRAIN_ENTRY) {
      result += asg.second * (perc_tt_config.transfer_penalty_ + e->cost_);
      continue;
    }
    if (e->type_ == eg_edge_type::TRIP ||
        e->type_ == eg_edge_type::WAIT_TRANSPORT) {
      auto remaining_assgmnts = asg.second;
      int64_t last_cap_step = 0;
      for (auto const [i, penalty] :
           utl::enumerate(perc_tt_config.tt_and_waiting_penalties_)) {
        auto curr_cap_step =
            uint64_t(e->soft_cap_boundary_ *
                     perc_tt_config.cost_function_capacity_steps_[i]);
        uint64_t remaining_cap = 0;
        if (e->passengers_ > curr_cap_step) {
          remaining_cap = 0;
        } else {
          if (e->passengers_ > last_cap_step) {
            remaining_cap = curr_cap_step - e->passengers_;
          } else {
            remaining_cap = curr_cap_step - last_cap_step;
          }
        }
        last_cap_step = curr_cap_step;
        if (remaining_assgmnts > remaining_cap) {
          result += remaining_cap *
                    (perc_tt_config.tt_and_waiting_penalties_[i] * e->cost_);
          cum_trips_waits +=
              remaining_cap *
              (perc_tt_config.tt_and_waiting_penalties_[i] * e->cost_);
          remaining_assgmnts -= remaining_cap;
        } else {
          result += remaining_assgmnts *
                    (perc_tt_config.tt_and_waiting_penalties_[i] * e->cost_);
          cum_trips_waits +=
              remaining_assgmnts *
              (perc_tt_config.tt_and_waiting_penalties_[i] * e->cost_);
          remaining_assgmnts = 0;
        }
      }
      continue;
    }
    if (e->type_ == eg_edge_type::WAIT_STATION ||
        e->type_ == eg_edge_type::TRAIN_EXIT ||
        e->type_ == eg_edge_type::FINISH) {
      result += asg.second * e->cost_;
    }
  }

  std::cout << std::fixed << "CUM NO ROUTE: " << cum_no_route << std::endl;
  std::cout << std::fixed << "CUM TRIPS AND WAITS: " << cum_trips_waits
            << std::endl;

  return result;
}

}  // namespace motis::paxassign