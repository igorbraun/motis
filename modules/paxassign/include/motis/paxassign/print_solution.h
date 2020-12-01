#pragma once

namespace motis::paxassign {
void print_solution_routes(std::vector<std::vector<eg_edge*>> const& solution,
                           std::vector<eg_psg_group> const& eg_psg_groups,
                           schedule const& sched) {
  for (auto i = 0u; i < solution.size(); ++i) {
    std::cout << "Psg: " << i << " count: " << eg_psg_groups[i].psg_count_
              << " edges : " << std::endl;
    for (auto const& e : solution[i]) {
      auto trp = (e->trip_ == nullptr)
                     ? "-"
                     : std::to_string(e->trip_->id_.primary_.train_nr_);
      std::cout << "  train " << trp << " type " << e->type_ << " from "
                << sched.stations_[e->from_->station_]->name_ << " to "
                << sched.stations_[e->to_->station_]->name_ << " at "
                << format_time(e->from_->time_) << " - "
                << format_time(e->to_->time_) << " cost " << e->cost_
                << std::endl;
    }
    std::cout << "route cost: "
              << calc_perc_tt(solution[i], perceived_tt_config{}) << std::endl;
  }
}
void print_solution_statistics(
    std::vector<std::vector<eg_edge*>> const& solution,
    std::vector<eg_psg_group> const& eg_psg_groups, schedule const&) {
  for (auto i = 0u; i < solution.size(); ++i) {
    std::cout << "PG id: " << i << ", psg nr: " << eg_psg_groups[i].psg_count_
              << std::endl;
    int entries = 0;
    int exits = 0;
    int no_routes = 0;
    time actual_arrival = 0;
    for (auto const& e : solution[i]) {
      if (e->type_ == eg_edge_type::TRAIN_ENTRY) {
        entries += 1;
      }
      if (e->type_ == eg_edge_type::TRAIN_EXIT) {
        exits += 1;
      }
      if (e->type_ == eg_edge_type::NO_ROUTE) {
        no_routes += 1;
      }
      if (e->to_->time_ != INVALID_TIME) {
        actual_arrival = e->to_->time_;
      }
    }
    auto planned_arrival = eg_psg_groups[i]
                               .cpg_.groups_[0]
                               ->compact_planned_journey_.legs_.back()
                               .exit_time_;
    std::cout << "no routes: " << no_routes << ", entries: " << entries
              << ", exits: " << exits << std::endl;
    std::cout << "planned arr time: " << planned_arrival
              << " actual arrival: " << actual_arrival
              << ", difference : " << actual_arrival - planned_arrival
              << std::endl;
  }
}
}  // namespace motis::paxassign