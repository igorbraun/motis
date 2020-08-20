#pragma once

#include <utl/equal_ranges_linear.h>
#include <algorithm>
#include <numeric>

#include "utl/equal_ranges.h"
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

namespace motis::paxassign {

inline eg_edge* add_edge(eg_edge const& e) {
  auto edge_ptr =
      e.from_->out_edges_.emplace_back(std::make_unique<eg_edge>(e)).get();
  e.to_->in_edges_.emplace_back(edge_ptr);
  return edge_ptr;
}

inline eg_edge make_interchange_edge(eg_event_node* from, eg_event_node* to,
                                     uint32_t transfer_time) {
  return eg_edge{from,
                 to,
                 eg_edge_type::INTERCHANGE,
                 transfer_time,
                 std::numeric_limits<std::uint16_t>::max(),
                 nullptr};
}

inline eg_edge make_trip_edge(eg_event_node* from, eg_event_node* to,
                              eg_edge_type type, trip const* trp,
                              std::uint16_t const capacity) {
  return eg_edge{from,     to,
                 type,     static_cast<uint32_t>(to->time_ - from->time_),
                 capacity, trp};
}

inline eg_edge make_not_in_trip_edge(eg_event_node* from, eg_event_node* to,
                                     eg_edge_type type, uint32_t cost) {
  return eg_edge{
      from, to, type, cost, std::numeric_limits<std::uint16_t>::max(), nullptr};
}

inline eg_edge make_no_route_edge(eg_event_node* from, eg_event_node* to,
                                  uint32_t cost) {
  return eg_edge{from,
                 to,
                 eg_edge_type::NO_ROUTE,
                 cost,
                 std::numeric_limits<std::uint16_t>::max(),
                 nullptr};
}

void add_no_route_edge(eg_event_node* from, eg_event_node* to, uint32_t cost,
                       time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == eg_edge_type::NO_ROUTE && e->to_ == to &&
        e->cost_ == cost) {
      return;
    }
  }
  g.no_route_edges_.emplace_back(add_edge(make_no_route_edge(from, to, cost)));
}

void add_interchange(eg_event_node* from, eg_event_node* to,
                     uint32_t transfer_time, time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == eg_edge_type::INTERCHANGE && e->to_ == to &&
        e->cost_ == transfer_time) {
      return;
    }
  }
  g.interchange_edges_.emplace_back(
      add_edge(make_interchange_edge(from, to, transfer_time)));
}

bool event_types_comp(event_type const& et, eg_event_type const& eg_et) {
  return ((et == event_type::ARR && eg_et == eg_event_type::ARR) ||
          (et == event_type::DEP && eg_et == eg_event_type::DEP));
}

std::uint16_t get_edge_capacity(eg_event_node const* from,
                                eg_event_node const* to, extern_trip const& et,
                                light_connection const& lc,
                                paxmon_data const& data,
                                schedule const& sched) {
  if (data.graph_.trip_data_.find(et) != data.graph_.trip_data_.end()) {
    auto td = data.graph_.trip_data_.find(et)->second.get();
    auto edge_it =
        std::find_if(std::begin(td->edges_), std::end(td->edges_),
                     [&](motis::paxmon::edge const* e) {
                       return e->from_->time_ == from->time_ &&
                              e->from_->station_ == from->station_ &&
                              e->to_->time_ == to->time_ &&
                              e->to_->station_ == to->station_ &&
                              event_types_comp(e->from_->type_, from->type_) &&
                              event_types_comp(e->to_->type_, to->type_);
                     });
    assert(edge_it != std::end(td->edges_));
    return ((*edge_it)->capacity() < (*edge_it)->passengers_)
               ? 0
               : (*edge_it)->capacity() - (*edge_it)->passengers_;
  }
  return get_capacity(sched, lc, data.trip_capacity_map_,
                      data.category_capacity_map_, data.default_capacity_)
      .first;
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

std::vector<light_connection const*> get_relevant_l_conns(
    station_node const* from, time const from_time,
    light_connection const& from_lcon) {
  std::vector<light_connection const*> result;
  for (auto const& sn_e : from->edges_) {
    if (!sn_e.to_->is_route_node()) {
      continue;
    }
    for (auto const& e : sn_e.to_->edges_) {
      if (e.type() != motis::edge::type::ROUTE_EDGE) {
        continue;
      }
      for (auto const& c : e.m_.route_edge_.conns_) {
        if (!c.valid_ || from_lcon.trips_ == c.trips_ ||
            c.d_time_ < from_time) {
          continue;
        }
        result.push_back(&c);
      }
    }
  }
  return result;
}

eg_event_node* find_event_node(light_connection const* l_conn,
                               eg_event_type const& ev_type,
                               time_expanded_graph const& g,
                               schedule const& sched) {
  assert(sched.merged_trips_[l_conn->trips_].size() == 1);
  auto const trip_ptr = (*sched.merged_trips_[l_conn->trips_])[0];
  auto et = to_extern_trip(sched, trip_ptr);
  auto et_it = g.trip_data_.find(et);
  if (et_it == g.trip_data_.end()) {
    throw std::runtime_error("extern trip not found in the trip_data map");
  }

  auto it =
      std::find_if(std::begin(et_it->second->edges_),
                   std::end(et_it->second->edges_), [&](eg_edge const* e) {
                     return (e->from_->time_ == l_conn->d_time_ &&
                             e->to_->time_ == l_conn->a_time_);
                   });
  assert(it != std::end(et_it->second->edges_));

  eg_event_node* result;
  switch (ev_type) {
    case eg_event_type::DEP: {
      result = (*it)->from_;
      break;
    }
    case eg_event_type::ARR: {
      result = (*it)->to_;
      break;
    }
    default: break;
  }
  return result;
}

void build_interchange_edges(
    light_connection const* from_l_conn,
    std::vector<light_connection const*> const& to_l_conns,
    schedule const& sched, time_expanded_graph& g) {
  auto from_n = find_event_node(from_l_conn, eg_event_type::ARR, g, sched);
  for (auto const c : to_l_conns) {
    auto to_n = find_event_node(c, eg_event_type::DEP, g, sched);
    uint32_t const inch_duration = to_n->time_ - from_n->time_;
    add_interchange(from_n, to_n, inch_duration, g);
  }
}

auto earliest_latest_node(std::vector<eg_event_node*> const& dep_arr_nodes) {
  return std::minmax_element(
      std::begin(dep_arr_nodes), std::end(dep_arr_nodes),
      [](auto lhs, auto rhs) { return lhs->time_ < rhs->time_; });
}

std::vector<eg_event_node*> filter_nodes(
    std::vector<eg_event_node*> const& dep_arr_nodes,
    eg_event_type const desired_type) {
  return utl::all(dep_arr_nodes) | utl::remove_if([&](auto const& n) {
           return n->type_ != desired_type;
         }) |
         utl::vec();
}

std::vector<eg_event_node*> create_wait_nodes(
    std::vector<eg_event_node*> const& for_nodes, uint32_t const transfer_cost,
    time_expanded_graph& graph) {
  std::vector<eg_event_node*> wait_nodes;
  for (auto& n : for_nodes) {
    auto wait_node =
        graph.nodes_
            .emplace_back(std::make_unique<eg_event_node>(
                eg_event_node{(n->type_ == eg_event_type::DEP)
                                  ? n->time_
                                  : static_cast<time>(n->time_ + transfer_cost),
                              eg_event_type::WAIT,
                              n->station_,
                              {},
                              {},
                              graph.nodes_.size()}))
            .get();
    (n->type_ == eg_event_type::DEP)
        ? graph.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
              wait_node, n, eg_edge_type::INTERCHANGE, 0)))
        : graph.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
              n, wait_node, eg_edge_type::WAIT, transfer_cost)));
    wait_nodes.push_back(wait_node);
  }
  return wait_nodes;
}

void connect_wait_nodes_pair(eg_event_node* fir, eg_event_node* sec,
                             time_expanded_graph& g) {
  g.not_trip_edges_.emplace_back(add_edge(make_not_in_trip_edge(
      fir, sec, eg_edge_type::WAIT, sec->time_ - fir->time_)));
  if (fir->time_ == sec->time_) {
    g.not_trip_edges_.emplace_back(
        add_edge(make_not_in_trip_edge(sec, fir, eg_edge_type::WAIT, 0)));
  }
}

void connect_wait_nodes(std::vector<eg_event_node*>& wait_nodes,
                        time_expanded_graph& g) {
  std::sort(
      std::begin(wait_nodes), std::end(wait_nodes),
      [](auto const& lhs, auto const& rhs) { return lhs->time_ < rhs->time_; });
  for (auto const& n : wait_nodes) {
    std::cout << "node time: " << n->time_ << std::endl;
  }
  std::cout << "Before calling eq_ranges" << std::endl;
  utl::equal_ranges_linear(
      wait_nodes,
      [](auto const& lhs, auto const& rhs) { return lhs->time_ == rhs->time_; },
      [&](auto lb, auto ub) {
        for (std::vector<eg_event_node*>::iterator it = lb; it != ub; ++it) {
          std::cout << (*it)->time_ << std::endl;
        }
//        std::cout << "lb: " << (*lb)->time_ << ", ub: " << (*ub)->time_
//                  << std::endl;
        std::cout << "Distance of equal ranges: " << std::distance(lb, ub)
                  << std::endl;
      });
}

void build_transfers(std::vector<eg_event_node*> const& dep_arr_nodes,
                     uint32_t const transfer_cost, time_expanded_graph& g) {
  /*
  auto const [earliest, latest] = earliest_latest_node(dep_arr_nodes);
  (*earliest)->time_;
  (*latest)->time_;

  auto dep_nodes = filter_nodes(dep_arr_nodes, eg_event_type::DEP);
  auto arr_nodes = filter_nodes(dep_arr_nodes, eg_event_type::ARR);

  auto waits_from_deps = create_wait_nodes(dep_nodes, transfer_cost, g);
  auto waits_from_arrs = create_wait_nodes(arr_nodes, transfer_cost, g);
*/
  auto wait_nodes = create_wait_nodes(dep_arr_nodes, transfer_cost, g);
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

    for (auto const& e : sn->edges_) {
      if (!e.to_->is_route_node()) {
        continue;
      }
      auto curr_route_node = e.to_;
      for (auto const& ie : curr_route_node->incoming_edges_) {
        if (ie->type() != motis::edge::type::ROUTE_EDGE) {
          continue;
        }
        for (auto const& c : ie->m_.route_edge_.conns_) {
          if (!c.valid_) continue;
          auto rel_l_conns = get_relevant_l_conns(
              sn.get(), c.a_time_ + sched.stations_[sn->id_]->transfer_time_,
              c);
          if (!rel_l_conns.empty()) {
            build_interchange_edges(&c, rel_l_conns, sched, graph);
          }
        }
      }
    }
  }

  return graph;
}

}  // namespace motis::paxassign