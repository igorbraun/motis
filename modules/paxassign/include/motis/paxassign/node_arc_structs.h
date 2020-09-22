#pragma once

#include <unordered_set>

namespace motis::paxassign {

struct config {
  uint32_t interchange_penalty_{0};  // typical value at DB: 30 min
  uint16_t max_allowed_interchanges_{6};
};

struct node_arc_psg_group {
  combined_passenger_group& cpg_;
  eg_event_node* from_;
  eg_event_node* to_;
  int psg_count_;
  std::unordered_set<eg_edge*> valid_edges_;
};

}  // namespace motis::paxassign