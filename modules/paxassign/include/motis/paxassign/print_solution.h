#pragma once

namespace motis::paxassign {

void print_solution_routes_mini_halle(
    std::map<std::uint16_t, combined_pg*> const& cpg_id_to_group,
    std::vector<std::pair<ilp_psg_id, alt_idx>> const& alt_to_use,
    schedule const& sched) {
  for (auto& assignment : alt_to_use) {
    auto cpg = cpg_id_to_group.at(assignment.first);
    if (cpg->alternatives_.size() <= assignment.second) {
      std::cout << "NO ROUTE ALTERNATIVE for GROUP(ID) " << cpg->id_ << " with "
                << cpg->passengers_ << " passengers" << std::endl;
    } else {
      auto cj = motis::paxmon::to_compact_journey(
          cpg->alternatives_[assignment.second].journey_, sched);
      std::cout << "ROUTE for GROUP(ID) " << cpg->id_ << " with "
                << cpg->passengers_ << " passengers:" << std::endl;
      for (auto const& leg : cj.legs_) {
        std::cout << leg.trip_->id_.primary_.train_nr_ << std::endl;
      }
    }
  }
}

void print_solution_routes_halle(
    std::vector<cap_ILP_psg_group> const cap_ilp_psg_groups,
    std::vector<std::pair<ilp_psg_id, alt_idx>> const& alt_to_use,
    schedule const& sched) {
  for (auto& assignment : alt_to_use) {
    auto cpg = std::find_if(cap_ilp_psg_groups.begin(), cap_ilp_psg_groups.end(),
                 [&assignment](cap_ILP_psg_group const& curr_gr) {
                   return curr_gr.id_ == assignment.first;
                 });
    assert(cpg != cap_ilp_psg_groups.end());

    if (cpg->alternatives_.size() <= assignment.second) {
      std::cout << "NO ROUTE ALTERNATIVE for GROUP(ID) " << cpg->id_ << " with "
                << cpg->psg_count_ << " passengers" << std::endl;
    } else {
      std::cout << "ROUTE for GROUP(ID) " << cpg->id_ << " with "
                << cpg->psg_count_ << " passengers:" << std::endl;
      for (auto const& e : cpg->alternatives_[assignment.second].edges_) {
        std::string trp = "-";
        if(e->trip_){
          trp = std::to_string(e->trip_->id_.primary_.train_nr_);
        }
        std::cout << "  train " << trp << " type " << e->type_ << " from "
                  << sched.stations_[e->from_->station_]->name_ << " to "
                  << sched.stations_[e->to_->station_]->name_ << " at "
                  << format_time(e->from_->time_) << " - "
                  << format_time(e->to_->time_) << " cost " << e->tt_
                  << std::endl;
      }
    }
  }
}


void print_solution_routes_node_arc(
    std::vector<std::vector<eg_edge*>> const& solution,
    std::vector<eg_psg_group> const& eg_psg_groups, schedule const& sched) {
  for (auto i = 0u; i < solution.size(); ++i) {
    std::cout << "Psg ID: " << eg_psg_groups[i].cpg_.id_
              << " psg count: " << eg_psg_groups[i].psg_count_
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

void print_solution_routes_mini_node_arc(
    std::vector<std::vector<eg_edge*>> const& solution,
    std::vector<eg_psg_group> const& eg_psg_groups, schedule const&) {
  for (auto i = 0u; i < solution.size(); ++i) {
    auto no_route_edge = std::find_if(
        solution[i].begin(), solution[i].end(),
        [](eg_edge* e) { return e->type_ == eg_edge_type::NO_ROUTE; });
    if (no_route_edge != solution[i].end()) {
      std::cout << "NO ROUTE ALTERNATIVE for GROUP(ID) "
                << eg_psg_groups[i].cpg_.id_ << " with "
                << eg_psg_groups[i].cpg_.passengers_ << " passengers"
                << std::endl;
      continue;
    }
    std::vector<uint64_t> driven_transports;
    for (auto const& e : solution[i]) {
      if (!e->trip_) {
        continue;
      }
      if (driven_transports.empty()) {
        driven_transports.push_back(e->trip_->id_.primary_.train_nr_);
      } else {
        if (driven_transports.back() != e->trip_->id_.primary_.train_nr_) {
          driven_transports.push_back(e->trip_->id_.primary_.train_nr_);
        }
      }
    }
    std::cout << "ROUTE for GROUP(ID) " << eg_psg_groups[i].cpg_.id_ << " with "
              << eg_psg_groups[i].cpg_.passengers_
              << " passengers:" << std::endl;
    for (auto e_idx = 0u; e_idx < driven_transports.size(); ++e_idx) {
      std::cout << driven_transports[e_idx] << std::endl;
    }
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
