#pragma once

#include <fstream>
#include <iostream>

#include "motis/paxassign/algorithms_configs.h"
#include "motis/paxassign/build_cap_ILP.h"
#include "motis/paxassign/capacitaty_aware_structs.h"

#include "motis/paxmon/graph.h"

namespace motis::paxassign {

void build_toy_scenario() {
  std::cout << "BUILD TEST SCENARIO" << std::endl;
  // frist graph
  cap_ILP_edge e1{1, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_1_2{100, 5, 30, edge_type::WAIT};
  cap_ILP_edge e2{2, 10, 30, edge_type::TRIP};
  cap_ILP_connection c1{1, 100, std::vector<cap_ILP_edge*>{&e1, &ew_1_2, &e2}};

  cap_ILP_edge e3{3, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_3_4{101, 5, 30, edge_type::WAIT};
  cap_ILP_edge e4{4, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_4_5{102, 5, 30, edge_type::WAIT};
  cap_ILP_edge e5{5, 10, 30, edge_type::TRIP};
  cap_ILP_connection c2{
      2, 100, std::vector<cap_ILP_edge*>{&e3, &ew_3_4, &e4, &ew_4_5, &e5}};
  cap_ILP_psg_group pg1{1, std::vector<cap_ILP_connection>{c1, c2},
                        10};  // should take c1

  // second graph
  cap_ILP_edge e6{6, 10, 80, edge_type::TRIP};
  cap_ILP_edge ew_6_7{103, 5, 80, edge_type::WAIT};
  cap_ILP_edge e7{7, 10, 80, edge_type::TRIP};
  cap_ILP_edge ew_7_8{104, 5, 80, edge_type::WAIT};
  cap_ILP_edge e8{8, 10, 80, edge_type::TRIP};
  cap_ILP_connection c3{
      3, 100, std::vector<cap_ILP_edge*>{&e6, &ew_6_7, &e7, &ew_7_8, &e8}};

  cap_ILP_edge e9{9, 10, 20, edge_type::TRIP};
  cap_ILP_edge ew_9_10{105, 5, 20, edge_type::WAIT};
  cap_ILP_edge e10{10, 10, 20, edge_type::TRIP};
  cap_ILP_connection c4{4, 100,
                        std::vector<cap_ILP_edge*>{&e9, &ew_9_10, &e10}};
  cap_ILP_psg_group pg2{2, std::vector<cap_ILP_connection>{c3, c4},
                        10};  // should take c4, cost 250
  cap_ILP_psg_group pg3{3, std::vector<cap_ILP_connection>{c3, c4},
                        30};  // should take c3, cost 1200

  // third graph
  cap_ILP_edge e11{11, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_11_12{106, 5, 30, edge_type::WAIT};
  cap_ILP_edge e12{12, 10, 30, edge_type::TRIP};
  cap_ILP_connection c5{5, 100,
                        std::vector<cap_ILP_edge*>{&e11, &ew_11_12, &e12}};

  cap_ILP_edge e13{13, 20, 30, edge_type::TRIP};
  cap_ILP_edge ew_13_14{107, 5, 30, edge_type::WAIT};
  cap_ILP_edge e14{14, 20, 30, edge_type::TRIP};
  cap_ILP_connection c6{6, 100,
                        std::vector<cap_ILP_edge*>{&e13, &ew_13_14, &e14}};
  cap_ILP_psg_group pg4{4, std::vector<cap_ILP_connection>{c5, c6},
                        30};  // should take c5, mixed cost-function

  std::vector<cap_ILP_psg_group> psg_groups{pg1, pg2, pg3, pg4};
  perceived_tt_config config;
  // build_ILP_from_scenario(psg_groups, config, "1");
  auto sol = build_ILP_from_scenario_API(psg_groups, config, "1");
  for (auto i = 0u; i < psg_groups.size(); ++i) {
    std::cout << "pg " << psg_groups[i].id_ << " should use "
              << psg_groups[i].alternatives_[sol.alt_to_use_[i].second].id_
              << std::endl;
  }
  std::cout << "Solution statistics:" << std::endl;
  std::cout << "Obj: " << sol.stats_.obj_ << std::endl;
  std::cout << "Group nr: " << sol.stats_.num_groups_ << std::endl;
  std::cout << "Variables: " << sol.stats_.num_vars_ << std::endl;
  std::cout << "Constraints: " << sol.stats_.num_constraints_ << std::endl;
  std::cout << "Run time: " << sol.stats_.run_time_ << std::endl;
}

}  // namespace motis::paxassign
