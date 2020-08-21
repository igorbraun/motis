#pragma once

#include <algorithm>
#include <numeric>

#include "utl/get_or_create.h"
#include "utl/pipes/all.h"
#include "utl/pipes/remove_if.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

#include "motis/core/access/realtime_access.h"
#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"
#include "motis/module/context/get_schedule.h"

#include "motis/paxassign/time_expanded_graph.h"
#include "motis/paxassign/service_time_exp_graph.h"

namespace motis::paxassign {

inline eg_edge* add_edge(eg_edge const& e) {
  auto edge_ptr =
      e.from_->out_edges_.emplace_back(std::make_unique<eg_edge>(e)).get();
  e.to_->in_edges_.emplace_back(edge_ptr);
  return edge_ptr;
}

inline eg_edge make_trip_edge(eg_event_node* from, eg_event_node* to,
                              eg_edge_type type, trip const* trp,
                              std::uint16_t const capacity) {
  return eg_edge{from,     to,
                 type,     static_cast<uint32_t>(to->time_ - from->time_),
                 capacity, trp};
}

inline eg_edge make_not_in_trip_edge(eg_event_node* from, eg_event_node* to,
                                     eg_edge_type et, uint32_t cost) {
  return eg_edge{
      from, to, et, cost, std::numeric_limits<std::uint16_t>::max(), nullptr};
}

void add_not_in_trip_edge(eg_event_node* from, eg_event_node* to,
                          eg_edge_type et, uint32_t transfer_time,
                          time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == et && e->to_ == to && e->cost_ == transfer_time) {
      return;
    }
  }
  g.not_trip_edges_.emplace_back(
      add_edge(make_not_in_trip_edge(from, to, et, transfer_time)));
}

std::vector<eg_edge*> add_trip(schedule const& sched, time_expanded_graph& g,
                               extern_trip const& et, paxmon_data const& data) {
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
    auto arr_node = g.nodes_
                        .emplace_back(std::make_unique<eg_event_node>(
                            eg_event_node{lc.a_time_,
                                          eg_event_type::ARR,
                                          section.to_station_id(),
                                          {},
                                          {},
                                          g.nodes_.size()}))
                        .get();
    auto capacity = get_edge_capacity(dep_node, arr_node, et, lc, data, sched);
    edges.emplace_back(add_edge(
        make_trip_edge(dep_node, arr_node, eg_edge_type::TRIP, trp, capacity)));
    if (prev_node != nullptr) {
      add_edge(make_trip_edge(prev_node, dep_node, eg_edge_type::WAIT, trp,
                              capacity));
    }
    prev_node = arr_node;
  }
  return edges;
}

eg_trip_data* get_or_add_trip(schedule const& sched, time_expanded_graph& g,
                              extern_trip const& et, paxmon_data const& data) {
  return utl::get_or_create(g.trip_data_, et,
                            [&]() {
                              return std::make_unique<eg_trip_data>(
                                  eg_trip_data{add_trip(sched, g, et, data)});
                            })
      .get();
}

void connect_wait_nodes(std::vector<eg_event_node*>& wait_nodes,
                        time_expanded_graph& g) {
  std::sort(
      std::begin(wait_nodes), std::end(wait_nodes),
      [](auto const& lhs, auto const& rhs) { return lhs->time_ < rhs->time_; });
  for (auto i = 0u; i < wait_nodes.size() - 1; ++i) {
    g.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
        wait_nodes[i], wait_nodes[i + 1], eg_edge_type::WAIT,
        wait_nodes[i + 1]->time_ - wait_nodes[i]->time_)));
  }
}

std::vector<eg_event_node*> create_wait_nodes(
    std::map<time, std::vector<eg_event_node*>>& time_to_nodes,
    time_expanded_graph& graph, uint32_t const transfer_cost) {
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
    for (auto& n : ttn.second) {
      (n->type_ == eg_event_type::DEP)
          ? graph.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
                wait_node, n, eg_edge_type::TRAIN_ENTRY, 0)))
          : graph.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
                n, wait_node, eg_edge_type::TRAIN_EXIT, transfer_cost)));
    }
    wait_nodes.push_back(wait_node);
  }
  return wait_nodes;
}

std::map<time, std::vector<eg_event_node*>> time_to_dep_arr_nodes(
    std::vector<eg_event_node*> const& dep_arr_nodes,
    uint32_t const transfer_cost) {
  std::map<time, std::vector<eg_event_node*>> result;
  for (auto const& n : dep_arr_nodes) {
    time t = (n->type_ == eg_event_type::DEP)
                 ? n->time_
                 : static_cast<time>(n->time_ + transfer_cost);
    result[t].push_back(n);
  }
  return result;
}

void build_transfers(std::vector<eg_event_node*> const& dep_arr_nodes,
                     uint32_t const transfer_cost, time_expanded_graph& g) {
  auto time_to_nodes = time_to_dep_arr_nodes(dep_arr_nodes, transfer_cost);
  auto wait_nodes = create_wait_nodes(time_to_nodes, g, transfer_cost);
  connect_wait_nodes(wait_nodes, g);
}

time_expanded_graph build_time_expanded_graph(paxmon_data const& data,
                                              schedule const& sched) {
  time_expanded_graph graph;

  for (auto const route_trips : sched.expanded_trips_) {
    for (trip const* rt : route_trips) {
      get_or_add_trip(sched, graph, to_extern_trip(sched, rt), data);
    }
  }

  for (auto const& sn : sched.station_nodes_) {
    std::vector<eg_event_node*> relevant_nodes =
        utl::all(graph.nodes_) |
        utl::remove_if([&](auto const& n) { return n->station_ != sn->id_; }) |
        utl::transform([&](auto const& n) { return n.get(); }) | utl::vec();

    if (!relevant_nodes.empty()) {
      build_transfers(relevant_nodes, sched.stations_[sn->id_]->transfer_time_,
                      graph);
    }
  }

  return graph;
}

}  // namespace motis::paxassign