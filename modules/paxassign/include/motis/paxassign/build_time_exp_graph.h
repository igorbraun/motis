#pragma once

#include <algorithm>
#include <numeric>

#include "utl/get_or_create.h"

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
  return eg_edge{from,          to, eg_edge_type::INTERCHANGE,
                 transfer_time, 0,  nullptr};
}

inline eg_edge make_trip_edge(eg_event_node* from, eg_event_node* to,
                              eg_edge_type type, trip const* trp,
                              std::uint16_t encoded_capacity) {
  return eg_edge{from, to, type, 0, encoded_capacity, trp};
}

inline eg_edge make_no_route_edge(eg_event_node* from, eg_event_node* to,
                                  uint32_t transfer_time) {
  return eg_edge{from, to, eg_edge_type::NO_ROUTE, transfer_time, 0, nullptr};
}

void add_no_route_edge(eg_event_node* from, eg_event_node* to,
                       uint32_t transfer_time, time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == eg_edge_type::NO_ROUTE && e->to_ == to &&
        e->transfer_time_ == transfer_time) {
      return;
    }
  }
  g.no_route_edges_.emplace_back(
      add_edge(make_no_route_edge(from, to, transfer_time)));
}

void add_interchange(eg_event_node* from, eg_event_node* to,
                     uint32_t transfer_time, time_expanded_graph& g) {
  for (auto& e : from->out_edges_) {
    if (e->type_ == eg_edge_type::INTERCHANGE && e->to_ == to &&
        e->transfer_time_ == transfer_time) {
      return;
    }
  }
  g.interchange_edges_.emplace_back(
      add_edge(make_interchange_edge(from, to, transfer_time)));
}

std::vector<eg_edge*> add_trip(
    schedule const& sched, time_expanded_graph& g, extern_trip const& et,
    trip_capacity_map_t const& trip_capacity_map,
    category_capacity_map_t const& category_capacity_map,
    std::uint16_t const default_capacity) {
  std::vector<eg_edge*> edges;

  auto trp = get_trip(sched, et);
  eg_event_node* prev_node = nullptr;
  for (auto const& section : motis::access::sections(trp)) {
    auto const& lc = section.lcon();
    auto dep_node = g.nodes_
                        .emplace_back(std::make_unique<eg_event_node>(
                            eg_event_node{lc.d_time_,
                                          event_type::DEP,
                                          section.from_station_id(),
                                          {},
                                          {},
                                          g.nodes_.size()}))
                        .get();
    auto arr_node = g.nodes_
                        .emplace_back(std::make_unique<eg_event_node>(
                            eg_event_node{lc.a_time_,
                                          event_type::ARR,
                                          section.to_station_id(),
                                          {},
                                          {},
                                          g.nodes_.size()}))
                        .get();
    auto const encoded_capacity = encode_capacity(get_capacity(
        sched, lc, trip_capacity_map, category_capacity_map, default_capacity));
    edges.emplace_back(add_edge(make_trip_edge(
        dep_node, arr_node, eg_edge_type::TRIP, trp, encoded_capacity)));
    if (prev_node != nullptr) {
      add_edge(make_trip_edge(prev_node, dep_node, eg_edge_type::WAIT, trp,
                              encoded_capacity));
    }
    prev_node = arr_node;
  }
  return edges;
}

eg_trip_data* get_or_add_trip(
    schedule const& sched, time_expanded_graph& g, extern_trip const& et,
    trip_capacity_map_t const& trip_capacity_map,
    category_capacity_map_t const& category_capacity_map,
    std::uint16_t const default_capacity) {
  return utl::get_or_create(
             g.trip_data_, et,
             [&]() {
               return std::make_unique<eg_trip_data>(eg_trip_data{
                   add_trip(sched, g, et, trip_capacity_map,
                            category_capacity_map, default_capacity)});
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
                               motis::event_type const& ev_type,
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
    case motis::event_type::DEP: {
      result = (*it)->from_;
      break;
    }
    case motis::event_type::ARR: {
      result = (*it)->to_;
      break;
    }
  }
  return result;
}

void build_interchange_edges(
    light_connection const* from_l_conn,
    std::vector<light_connection const*> const& to_l_conns,
    schedule const& sched, time_expanded_graph& g) {
  auto from_n = find_event_node(from_l_conn, motis::event_type::ARR, g, sched);
  for (auto const c : to_l_conns) {
    auto to_n = find_event_node(c, motis::event_type::DEP, g, sched);
    uint32_t const inch_duration = to_n->time_ - from_n->time_;
    add_interchange(from_n, to_n, inch_duration, g);
  }
}

time_expanded_graph build_time_expanded_graph(paxmon_data const& data,
                                              schedule const& sched) {
  time_expanded_graph graph;

  trip_capacity_map_t trip_capacity_map{data.trip_capacity_map_};
  category_capacity_map_t category_capacity_map{data.category_capacity_map_};
  std::uint16_t default_capacity{data.default_capacity_};

  for (auto const route_trips : sched.expanded_trips_) {
    for (trip const* rt : route_trips) {
      get_or_add_trip(sched, graph, to_extern_trip(sched, rt),
                      trip_capacity_map, category_capacity_map,
                      default_capacity);
    }
  }

  for (auto const& sn : sched.station_nodes_) {
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