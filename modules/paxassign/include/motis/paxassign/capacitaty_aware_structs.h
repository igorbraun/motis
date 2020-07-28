#pragma once

namespace motis::paxassign {

enum class edge_type : std::uint8_t { TRIP, INTERCHANGE, WAIT, NOROUTE };

struct cap_ILP_edge {
  uint32_t id_;
  uint32_t tt_;
  ulong capacity_;
  edge_type type_;
};

struct cap_ILP_connection {
  uint32_t id_;
  uint32_t associated_waiting_time_;
  std::vector<cap_ILP_edge*> edges_;
};

struct cap_ILP_psg_group {
  uint32_t id_;
  std::vector<cap_ILP_connection> alternatives_;
  int psg_count_;
};

struct cap_ILP_config {
  std::vector<double> cost_function_capacity_steps_{0.65, 1.0, 1.2};
  std::vector<double> tt_and_waiting_penalties_{1.0, 1.2, 2.0};

  double const transfer_penalty_ = 0;
  uint32_t const no_route_cost_ = 100000;
};

struct cap_ILP_stats {
  unsigned long num_groups_;
  int no_alt_found_;
  int num_vars_;
  int num_constraints_;
  double run_time_;
  double obj_;
};

struct cap_ILP_solution {
  cap_ILP_stats stats_;
  std::vector<uint16_t> alt_to_use_;
};

}  // namespace motis::paxassign
