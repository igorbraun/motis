#pragma once

#include <fstream>
#include <iostream>

#include "motis/paxmon/graph.h"

#include "utl/enumerate.h"

namespace motis::paxassign {

enum class edge_type : std::uint8_t { TRIP, INTERCHANGE, WAIT, NOROUTE };

struct cap_ILP_edge {
  uint32_t id_;
  double tt_;
  int capacity_;
  edge_type type_;
};

struct cap_ILP_connection {
  uint32_t id_;
  std::vector<cap_ILP_edge> edges_;
};

struct cap_ILP_psg_group {
  uint32_t id_;
  std::vector<cap_ILP_connection> alternatives_;
  int psg_count_;
};

struct cap_ILP_config {
  std::vector<double> cost_function_capacity_steps_{0.65, 1.0, 1.2};
  std::vector<double> tt_and_waiting_penalties_{1.0, 1.2, 2.0};

  double const transfer_penalty_ = 5;
  double const no_route_cost_ = 10000;
};

void build_ILP_from_scenario(std::vector<cap_ILP_psg_group> const& passengers,
                             cap_ILP_config const& config,
                             std::string const& scenario_id) {
  std::cout << "BUILD ILP" << std::endl;

  std::ofstream ilp_file("motis/build/rel/ilp_files/" + scenario_id + ".lp");
  // OBJECTIVE PART
  ilp_file << "Minimize\n";
  std::set<cap_ILP_edge const*> handled_edges;
  std::map<uint32_t, std::set<cap_ILP_psg_group const*>> edge_to_psgs;
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        edge_to_psgs[e.id_].insert(&pg);
        if (handled_edges.find(&e) != handled_edges.end()) continue;
        handled_edges.insert(&e);
        switch (e.type_) {
          case edge_type::TRIP:
          case edge_type::WAIT:
            for (auto const [i, penalty] :
                 utl::enumerate(config.tt_and_waiting_penalties_)) {
              ilp_file << "+ " << std::to_string(penalty * e.tt_) << " f_"
                       << std::to_string(e.id_) << "_" << std::to_string(i)
                       << " ";
            }
            break;
          case edge_type::INTERCHANGE:
            ilp_file << std::to_string(config.transfer_penalty_ + e.tt_)
                     << " f_" << std::to_string(e.id_) << " ";
            break;
          case edge_type::NOROUTE:
            ilp_file << std::to_string(config.no_route_cost_) << " f_"
                     << std::to_string(e.id_) << " ";
            break;
        }
      }
    }
  }
  ilp_file << "\n";

  // CONSTRAINTS
  ilp_file << "Subject To\n";
  // AT LEAST ONE ROUTE F.E. GROUP
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      ilp_file << "+ y_" << std::to_string(pg.id_) << "_"
               << std::to_string(a.id_) << " ";
    }
    ilp_file << "= 1\n";
  }

  // INDICATOR CONSTRAINTS
  for (auto const& pg : passengers) {
    for (auto const& a : pg.alternatives_) {
      for (auto const& e : a.edges_) {
        // b1 = 1 -> 2.5 x + 2.3 y + 5.3 z <= 8.1
        ilp_file << "y_" << std::to_string(pg.id_) << "_"
                 << std::to_string(a.id_) << " = 1 -> "
                 << "delta_" << std::to_string(pg.id_) << "_"
                 << std::to_string(e.id_) << " = 1\n";
      }
    }
  }

  // EDGES CAPACITY
  for (auto const& e : handled_edges) {
    for (auto const& pg : edge_to_psgs[e->id_]) {
      ilp_file << "+ " << std::to_string(pg->psg_count_) << " delta_"
               << std::to_string(pg->id_) << "_" << std::to_string(e->id_)
               << " ";
    }
    switch (e->type_) {
      case edge_type::TRIP:
      case edge_type::WAIT:
        for (auto const [i, step] :
             utl::enumerate(config.cost_function_capacity_steps_)) {
          ilp_file << "- f_" << std::to_string(e->id_) << "_"
                   << std::to_string(i) << " ";
        }
        break;
      case edge_type::INTERCHANGE:
      case edge_type::NOROUTE:
        ilp_file << "- f_" << std::to_string(e->id_) << " ";
        break;
    }
    ilp_file << "= 0\n";
  }

  // BOUNDS
  ilp_file << "Bounds\n";
  for (auto const& e : handled_edges) {
    switch (e->type_) {
      case edge_type::TRIP:
      case edge_type::WAIT: {
        uint32_t last_cap_step = 0;
        for (auto const [i, step] :
             utl::enumerate(config.cost_function_capacity_steps_)) {
          auto curr_cap_step = uint32_t(e->capacity_ * step);
          ilp_file << "0 <= f_" << std::to_string(e->id_) << "_"
                   << std::to_string(i)
                   << " <= " << std::to_string(curr_cap_step - last_cap_step)
                   << "\n";
          last_cap_step = curr_cap_step;
        }
        break;
      }
      case edge_type::INTERCHANGE:
      case edge_type::NOROUTE:
        ilp_file << "f_" << std::to_string(e->id_) << " >= 0\n";
        break;
    }
  }

  std::cout << "ILP ENDED" << std::endl;
  ilp_file << "End";
  ilp_file.close();
}

void build_toy_scenario() {
  std::cout << "BUILD TEST SCENARIOT" << std::endl;

  // frist graph
  cap_ILP_edge e1{1, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_1_2{100, 5, 30, edge_type::WAIT};
  cap_ILP_edge e2{2, 10, 30, edge_type::TRIP};
  cap_ILP_connection c1{1, std::vector<cap_ILP_edge>{e1, ew_1_2, e2}};

  cap_ILP_edge e3{3, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_3_4{101, 5, 30, edge_type::WAIT};
  cap_ILP_edge e4{4, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_4_5{102, 5, 30, edge_type::WAIT};
  cap_ILP_edge e5{5, 10, 30, edge_type::TRIP};
  cap_ILP_connection c2{2,
                        std::vector<cap_ILP_edge>{e3, ew_3_4, e4, ew_4_5, e5}};
  cap_ILP_psg_group pg1{1, std::vector<cap_ILP_connection>{c1, c2},
                        10};  // should take c1

  // second graph
  cap_ILP_edge e6{6, 10, 80, edge_type::TRIP};
  cap_ILP_edge ew_6_7{103, 5, 80, edge_type::WAIT};
  cap_ILP_edge e7{7, 10, 80, edge_type::TRIP};
  cap_ILP_edge ew_7_8{104, 5, 80, edge_type::WAIT};
  cap_ILP_edge e8{8, 10, 80, edge_type::TRIP};
  cap_ILP_connection c3{3,
                        std::vector<cap_ILP_edge>{e6, ew_6_7, e7, ew_7_8, e8}};

  cap_ILP_edge e9{9, 10, 20, edge_type::TRIP};
  cap_ILP_edge ew_9_10{105, 5, 20, edge_type::WAIT};
  cap_ILP_edge e10{10, 10, 20, edge_type::TRIP};
  cap_ILP_connection c4{4, std::vector<cap_ILP_edge>{e9, ew_9_10, e10}};
  cap_ILP_psg_group pg2{2, std::vector<cap_ILP_connection>{c3, c4},
                        10};  // should take c4
  cap_ILP_psg_group pg3{3, std::vector<cap_ILP_connection>{c3, c4},
                        30};  // should take c3

  // third graph
  cap_ILP_edge e11{11, 10, 30, edge_type::TRIP};
  cap_ILP_edge ew_11_12{106, 5, 30, edge_type::WAIT};
  cap_ILP_edge e12{12, 10, 30, edge_type::TRIP};
  cap_ILP_connection c5{5, std::vector<cap_ILP_edge>{e11, ew_11_12, e12}};

  cap_ILP_edge e13{13, 20, 30, edge_type::TRIP};
  cap_ILP_edge ew_13_14{107, 5, 30, edge_type::WAIT};
  cap_ILP_edge e14{14, 20, 30, edge_type::TRIP};
  cap_ILP_connection c6{6, std::vector<cap_ILP_edge>{e13, ew_13_14, e14}};
  cap_ILP_psg_group pg4{4, std::vector<cap_ILP_connection>{c5, c6},
                        30};  // should take c5, mixed cost-function

  std::vector<cap_ILP_psg_group> psg_groups{pg1, pg2, pg3, pg4};
  cap_ILP_config config;
  build_ILP_from_scenario(psg_groups, config, "1");
}

}  // namespace motis::paxassign
