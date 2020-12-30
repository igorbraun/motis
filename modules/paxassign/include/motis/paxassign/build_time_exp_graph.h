#pragma once

#include <algorithm>
#include <numeric>

#include "utl/get_or_create.h"
#include "utl/pipes/all.h"
#include "utl/pipes/remove_if.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

#include "motis/core/common/logging.h"
#include "motis/core/access/realtime_access.h"
#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"
#include "motis/module/context/get_schedule.h"

#include "motis/paxassign/service_time_exp_graph.h"
#include "motis/paxassign/time_expanded_graph.h"

namespace motis::paxassign {

inline eg_edge* add_edge(eg_edge const& e) {
  auto edge_ptr =
      e.from_->out_edges_.emplace_back(std::make_unique<eg_edge>(e)).get();
  e.to_->in_edges_.emplace_back(edge_ptr);
  return edge_ptr;
}

inline eg_edge make_trip_edge(eg_event_node* from, eg_event_node* to,
                              eg_edge_type type, trip const* trp,
                              std::uint16_t const soft_capacity,
                              std::uint16_t const hard_capacity,
                              std::uint16_t const edge_psgs,
                              double const capacity_utilization,
                              service_class const sc) {
  return eg_edge{from,
                 to,
                 type,
                 static_cast<uint32_t>(to->time_ - from->time_),
                 soft_capacity,
                 hard_capacity,
                 edge_psgs,
                 capacity_utilization,
                 sc,
                 trp};
}

inline eg_edge make_not_in_trip_edge(eg_event_node* from, eg_event_node* to,
                                     eg_edge_type type, uint32_t cost) {
  return eg_edge{from,
                 to,
                 type,
                 cost,
                 std::numeric_limits<std::uint16_t>::max(),
                 std::numeric_limits<std::uint16_t>::max(),
                 0,
                 0.0,
                 service_class::OTHER,
                 nullptr};
}

void add_not_in_trip_edge(eg_event_node* from, eg_event_node* to,
                          eg_edge_type type, uint32_t transfer_time,
                          time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == type && e->to_ == to && e->cost_ == transfer_time) {
      return;
    }
  }
  g.not_trip_edges_.emplace_back(
      add_edge(make_not_in_trip_edge(from, to, type, transfer_time)));
}

std::vector<eg_edge*> add_trip(schedule const& sched,
                               node_arc_config const& config,
                               time_expanded_graph& g, extern_trip const& et,
                               paxmon_data const& data) {
  std::vector<eg_edge*> edges;
  auto trp = get_trip(sched, et);
  eg_event_node* prev_node = nullptr;
  for (auto const& section : motis::access::sections(trp)) {
    auto const& lc = section.lcon();
    auto dep_node = g.nodes_
                        .emplace_back(std::make_unique<eg_event_node>(
                            eg_event_node{lc.d_time_,
                                          eg_event_type::DEP,
                                          section.from_station_id(),
                                          {},
                                          {},
                                          g.nodes_.size()}))
                        .get();
    g.st_to_nodes_[dep_node->station_].push_back(dep_node);
    auto arr_node = g.nodes_
                        .emplace_back(std::make_unique<eg_event_node>(
                            eg_event_node{lc.a_time_,
                                          eg_event_type::ARR,
                                          section.to_station_id(),
                                          {},
                                          {},
                                          g.nodes_.size()}))
                        .get();
    g.st_to_nodes_[arr_node->station_].push_back(arr_node);
    auto soft_capacity =
        get_edge_overall_capacity(dep_node, arr_node, et, lc, data, sched);
    auto hard_capacity =
        static_cast<std::uint16_t>(config.hard_capacity_ratio_ * soft_capacity);
    auto capacity_utilization =
        get_edge_capacity_utilization(dep_node, arr_node, et, data, sched);
    auto edge_psgs = get_edge_psgs(dep_node, arr_node, et, data, sched);
    edges.emplace_back(add_edge(make_trip_edge(
        dep_node, arr_node, eg_edge_type::TRIP, trp, soft_capacity,
        hard_capacity, edge_psgs, capacity_utilization, lc.full_con_->clasz_)));
    if (prev_node != nullptr) {
      add_edge(make_trip_edge(prev_node, dep_node, eg_edge_type::WAIT_TRANSPORT,
                              trp, soft_capacity, hard_capacity, edge_psgs,
                              capacity_utilization, lc.full_con_->clasz_));
    }
    prev_node = arr_node;
  }
  return edges;
}

eg_trip_data* get_or_add_trip(schedule const& sched,
                              node_arc_config const& config,
                              time_expanded_graph& g, extern_trip const& et,
                              paxmon_data const& data) {
  return utl::get_or_create(
             g.trip_data_, et,
             [&]() {
               return std::make_unique<eg_trip_data>(
                   eg_trip_data{add_trip(sched, config, g, et, data)});
             })
      .get();
}

void connect_wait_nodes(std::vector<eg_event_node*>& wait_nodes,
                        time_expanded_graph& g) {
  if (wait_nodes.empty()) return;
  std::sort(
      std::begin(wait_nodes), std::end(wait_nodes),
      [](auto const& lhs, auto const& rhs) { return lhs->time_ < rhs->time_; });
  for (auto i = 0u; i < wait_nodes.size() - 1; ++i) {
    g.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
        wait_nodes[i], wait_nodes[i + 1], eg_edge_type::WAIT_STATION,
        wait_nodes[i + 1]->time_ - wait_nodes[i]->time_)));
  }
}

void connect_arrs_to_waits(uint32_t const station_id, time_expanded_graph& g,
                           schedule const& sched) {
  auto const transfer_time = sched.stations_[station_id]->transfer_time_;
  for (auto& curr_arr_n : g.st_to_nodes_[station_id]) {
    if (curr_arr_n->type_ == eg_event_type::ARR) {
      std::vector<eg_event_node*> relevant_wait_nodes =
          utl::all(g.st_to_nodes_[station_id]) |
          utl::remove_if([&](auto const& n) {
            return n->type_ != eg_event_type::WAIT ||
                   n->time_ <= curr_arr_n->time_ + transfer_time;
          }) |
          utl::vec();
      if (relevant_wait_nodes.empty()) {
        continue;
      } else {
        std::sort(std::begin(relevant_wait_nodes),
                  std::end(relevant_wait_nodes),
                  [](eg_event_node const* lhs, eg_event_node const* rhs) {
                    return lhs->time_ < rhs->time_;
                  });
        g.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
            curr_arr_n, relevant_wait_nodes[0], eg_edge_type::TRAIN_EXIT,
            relevant_wait_nodes[0]->time_ - curr_arr_n->time_)));
      }
    }
  }
}

std::vector<eg_event_node*> create_wait_nodes_for_deps(
    std::map<time, std::vector<eg_event_node*>>& time_to_nodes,
    time_expanded_graph& graph) {
  std::vector<eg_event_node*> wait_nodes;
  for (auto& ttn : time_to_nodes) {
    auto wait_node = graph.nodes_
                         .emplace_back(std::make_unique<eg_event_node>(
                             eg_event_node{ttn.first,
                                           eg_event_type::WAIT,
                                           ttn.second[0]->station_,
                                           {},
                                           {},
                                           graph.nodes_.size()}))
                         .get();
    graph.st_to_nodes_[wait_node->station_].push_back(wait_node);
    for (auto& n : ttn.second) {
      graph.not_trip_edges_.emplace_back(add_edge(
          make_not_in_trip_edge(wait_node, n, eg_edge_type::TRAIN_ENTRY, 0)));
    }
    wait_nodes.push_back(wait_node);
  }
  return wait_nodes;
}

std::map<time, std::vector<eg_event_node*>> map_time_to_dep_nodes(
    std::vector<eg_event_node*> const& nodes) {
  std::map<time, std::vector<eg_event_node*>> result;
  for (auto const& n : nodes) {
    if (n->type_ == eg_event_type::DEP) {
      result[n->time_].push_back(n);
    }
  }
  return result;
}

void build_transfers(uint32_t station_id, time_expanded_graph& g,
                     schedule const& sched) {
  auto time_to_dep_nodes = map_time_to_dep_nodes(g.st_to_nodes_[station_id]);
  auto wait_nodes = create_wait_nodes_for_deps(time_to_dep_nodes, g);
  connect_wait_nodes(wait_nodes, g);
  connect_arrs_to_waits(station_id, g, sched);
}

void build_foot_edges(uint32_t station_id, time_expanded_graph& g,
                      schedule const& sched) {
  for (auto const out_fe : sched.stations_[station_id]->outgoing_footpaths_) {
    for (auto& curr_n : g.st_to_nodes_[station_id]) {
      if (curr_n->type_ == eg_event_type::ARR) {
        std::vector<eg_event_node*> rel_wait_nodes_at_to_st =
            utl::all(g.st_to_nodes_[out_fe.to_station_]) |
            utl::remove_if([&](auto const& n) {
              return n->type_ != eg_event_type::WAIT ||
                     n->time_ <= curr_n->time_ + out_fe.duration_;
            }) |
            utl::vec();
        if (rel_wait_nodes_at_to_st.empty()) {
          continue;
        } else {
          std::sort(std::begin(rel_wait_nodes_at_to_st),
                    std::end(rel_wait_nodes_at_to_st),
                    [](eg_event_node const* lhs, eg_event_node const* rhs) {
                      return lhs->time_ < rhs->time_;
                    });
          g.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
              curr_n, rel_wait_nodes_at_to_st[0], eg_edge_type::WAIT_STATION,
              rel_wait_nodes_at_to_st[0]->time_ - curr_n->time_)));
        }
      }
    }
  }
}

time_expanded_graph build_time_expanded_graph(paxmon_data const& data,
                                              schedule const& sched,
                                              node_arc_config const& config) {
  time_expanded_graph te_graph;
  {
    logging::scoped_timer alt_timer{"add trips to te-graph"};
    for (auto const route_trips : sched.expanded_trips_) {
      for (trip const* rt : route_trips) {
        to_extern_trip(sched, rt);
        get_or_add_trip(sched, config, te_graph, to_extern_trip(sched, rt),
                        data);
      }
    }
  }

  {
    logging::scoped_timer alt_timer{"add transfers to te-graph"};
    auto station_count = sched.station_nodes_.size();
    int i = 0;
    for (auto const& sn : sched.station_nodes_) {
      if (i % 1000 == 0) {
        std::cout << "process sn " << i++ << " from " << station_count
                  << std::endl;
      }
      i++;
      if (!te_graph.st_to_nodes_[sn->id_].empty()) {
        build_transfers(sn->id_, te_graph, sched);
      }
    }
    for (auto const& sn : sched.station_nodes_) {
      if (!te_graph.st_to_nodes_[sn->id_].empty()) {
        build_foot_edges(sn->id_, te_graph, sched);
      }
    }
  }
  return te_graph;
}

eg_event_node* get_localization_node(combined_pg const& cpg,
                                     time_expanded_graph& te_graph,
                                     schedule const& sched) {
  // CASE I: Passenger in trip
  if (cpg.localization_.in_trip()) {
    auto tr_data = te_graph.trip_data_.find(
        to_extern_trip(sched, cpg.localization_.in_trip_));
    assert(tr_data != te_graph.trip_data_.end());
    auto at_edge = std::find_if(
        std::begin(tr_data->second->edges_), std::end(tr_data->second->edges_),
        [&cpg](eg_edge* e_ptr) {
          return e_ptr->to_->station_ ==
                     cpg.localization_.at_station_->index_ &&
                 cpg.localization_.current_arrival_time_ == e_ptr->to_->time_;
        });
    assert(at_edge != tr_data->second->edges_.end());
    return (*at_edge)->to_;
  } else {
    // CASE II: Passenger at station, either before journey or at interchange
    auto psg_localization_node =
        te_graph.nodes_
            .emplace_back(std::make_unique<eg_event_node>(
                eg_event_node{cpg.localization_.current_arrival_time_,
                              eg_event_type::WAIT,
                              cpg.localization_.at_station_->index_,
                              {},
                              {},
                              te_graph.nodes_.size()}))
            .get();
    te_graph.st_to_nodes_[cpg.localization_.at_station_->index_].push_back(
        psg_localization_node);
    std::vector<eg_event_node*> relevant_nodes =
        utl::all(te_graph.st_to_nodes_[cpg.localization_.at_station_->index_]) |
        utl::remove_if([&](auto const& n) {
          return n == psg_localization_node ||
                 n->type_ != eg_event_type::WAIT ||
                 n->time_ < cpg.localization_.current_arrival_time_;
        }) |
        utl::vec();
    if (relevant_nodes.empty()) {
      return psg_localization_node;
    } else {
      std::sort(std::begin(relevant_nodes), std::end(relevant_nodes),
                [](eg_event_node const* lhs, eg_event_node const* rhs) {
                  return lhs->time_ < rhs->time_;
                });
      te_graph.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
          psg_localization_node, relevant_nodes[0], eg_edge_type::WAIT_STATION,
          relevant_nodes[0]->time_ - psg_localization_node->time_)));
      return psg_localization_node;
    }
  }
}

std::vector<eg_psg_group> add_psgs_to_te_graph(
    std::map<unsigned, std::vector<combined_pg>>& combined_groups,
    schedule const& sched, node_arc_config const& config,
    time_expanded_graph& te_graph) {
  std::vector<eg_psg_group> eg_psg_groups;
  {
    logging::scoped_timer alt_timer{"add passengers to te graph"};

    for (auto& cgs : combined_groups) {
      for (auto& cpg : cgs.second) {
        std::cout << "1" << std::endl;
        eg_event_node* at_ev_node = get_localization_node(cpg, te_graph, sched);
        std::cout << "2" << std::endl;
        auto target_node = te_graph.nodes_
                               .emplace_back(std::make_unique<eg_event_node>(
                                   eg_event_node{INVALID_TIME,
                                                 eg_event_type::ARR,
                                                 cpg.destination_station_id_,
                                                 {},
                                                 {},
                                                 te_graph.nodes_.size()}))
                               .get();
        std::cout << "3" << std::endl;
        te_graph.st_to_nodes_[cpg.destination_station_id_].push_back(
            target_node);
        std::cout << "4" << std::endl;
        for (auto& n : te_graph.nodes_) {
          if (n.get() != target_node &&
              n->station_ == cpg.destination_station_id_ &&
              n->type_ == eg_event_type::ARR) {
            motis::paxassign::add_not_in_trip_edge(
                n.get(), target_node, eg_edge_type::FINISH, 0, te_graph);
          }
        }
        std::cout << "5" << std::endl;
        motis::paxassign::add_not_in_trip_edge(at_ev_node, target_node,
                                               eg_edge_type::NO_ROUTE,
                                               config.no_route_cost_, te_graph);
        std::cout << "6" << std::endl;
        eg_psg_groups.push_back(
            {cpg, at_ev_node, target_node, cpg.passengers_});
        std::cout << "7" << std::endl;
      }
    }
  }
  return eg_psg_groups;
}

}  // namespace motis::paxassign