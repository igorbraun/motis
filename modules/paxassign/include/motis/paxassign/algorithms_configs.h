#pragma once

namespace motis::paxassign {

struct perceived_tt_config {
  std::vector<double> cost_function_capacity_steps_{0.65, 1.0, 1.2};
  std::vector<double> tt_and_waiting_penalties_{1.0, 1.2, 2.0};

  double const transfer_penalty_{30};
  uint32_t const no_route_cost_{100000};
};

struct node_arc_config {
  uint32_t interchange_penalty_{30};  // typical value at DB: 30 min
  uint16_t max_allowed_interchanges_{6};
  uint32_t const no_route_cost_{100000};
};

}  // namespace motis::paxassign