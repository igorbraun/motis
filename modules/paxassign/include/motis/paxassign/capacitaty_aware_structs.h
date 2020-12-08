#pragma once

namespace motis::paxassign {

enum class edge_type : std::uint8_t { TRIP, INTERCHANGE, WAIT, NOROUTE };

struct cap_ILP_edge {
  uint32_t id_;
  uint32_t tt_;
  std::uint64_t soft_cap_boundary_{};
  std::uint64_t hard_cap_boundary_{};
  std::uint64_t passengers_{};
  edge_type type_;
};

struct cap_ILP_connection {
  uint32_t id_;
  uint32_t associated_waiting_time_;
  std::vector<cap_ILP_edge*> edges_;
};

using ilp_psg_id = std::uint16_t;

struct cap_ILP_psg_group {
  std::uint16_t id_;
  std::vector<cap_ILP_connection> alternatives_;
  int psg_count_;
};

struct cap_ILP_stats {
  unsigned long num_groups_;
  int no_alt_found_;
  int num_vars_;
  int num_constraints_;
  double run_time_;
  double obj_;
};

using alt_idx = uint32_t;

struct cap_ILP_solution {
  cap_ILP_stats stats_;
  std::vector<std::pair<ilp_psg_id, alt_idx>> alt_to_use_;
};

}  // namespace motis::paxassign
