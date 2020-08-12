#pragma once

#include <algorithm>
#include <numeric>

#include "utl/get_or_create.h"

#include "motis/core/access/realtime_access.h"
#include "motis/core/access/trip_iterator.h"
#include "motis/core/conv/trip_conv.h"
#include "motis/module/context/get_schedule.h"

#include "motis/paxmon/graph_access.h"

namespace motis::paxassign {

using namespace motis::paxmon;

inline motis::paxmon::edge make_interchange_edge(event_node* from,
                                                 event_node* to,
                                                 duration transfer_time) {
  return motis::paxmon::edge{
      from, to,      edge_type::INTERCHANGE, false, transfer_time, 0,
      0,    nullptr, pax_connection_info{}};
}

void add_interchange_wo_psgs(
    event_node* from, event_node* to, duration transfer_time, graph const& g,
    std::vector<motis::paxmon::edge*>& interchange_edges) {
  for (auto& e : from->outgoing_edges(g)) {
    if (e->type_ == edge_type::INTERCHANGE && e->to(g) == to &&
        e->transfer_time() == transfer_time) {
      std::cout << "already there" << std::endl;
      return;
    }
  }
  interchange_edges.emplace_back(
      add_edge(make_interchange_edge(from, to, transfer_time)));
}

std::vector<motis::paxmon::edge*> add_trip_graph_only(
    schedule const& sched, graph& g, extern_trip const& et,
    trip_capacity_map_t const& trip_capacity_map,
    category_capacity_map_t const& category_capacity_map,
    std::uint16_t const default_capacity) {
  std::vector<motis::paxmon::edge*> edges;

  auto trp = get_trip(sched, et);
  event_node* prev_node = nullptr;
  for (auto const& section : motis::access::sections(trp)) {
    auto const& lc = section.lcon();
    auto dep_node = g.nodes_
                        .emplace_back(std::make_unique<event_node>(event_node{
                            lc.d_time_,
                            get_schedule_time(sched, section.edge(),
                                              trp->lcon_idx_, event_type::DEP),
                            event_type::DEP,
                            true,
                            section.from_station_id(),
                            {},
                            {}}))
                        .get();
    auto arr_node = g.nodes_
                        .emplace_back(std::make_unique<event_node>(event_node{
                            lc.a_time_,
                            get_schedule_time(sched, section.edge(),
                                              trp->lcon_idx_, event_type::ARR),
                            event_type::ARR,
                            true,
                            section.to_station_id(),
                            {},
                            {}}))
                        .get();
    auto const encoded_capacity = encode_capacity(get_capacity(
        sched, lc, trip_capacity_map, category_capacity_map, default_capacity));
    edges.emplace_back(add_edge(make_trip_edge(
        dep_node, arr_node, edge_type::TRIP, trp, encoded_capacity)));
    if (prev_node != nullptr) {
      add_edge(make_trip_edge(prev_node, dep_node, edge_type::WAIT, trp,
                              encoded_capacity));
    }
    prev_node = arr_node;
  }
  return edges;
}

trip_data* get_or_add_trip_graph_only(
    schedule const& sched, graph& g, extern_trip const& et,
    trip_capacity_map_t const& trip_capacity_map,
    category_capacity_map_t const& category_capacity_map,
    std::uint16_t const default_capacity) {
  return utl::get_or_create(
             g.trip_data_, et,
             [&]() {
               return std::make_unique<trip_data>(trip_data{
                   add_trip_graph_only(sched, g, et, trip_capacity_map,
                                       category_capacity_map, default_capacity),
                   {},
                   {}});
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

event_node* find_event_node(light_connection const* l_conn,
                            motis::event_type const& ev_type, graph const& g,
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
                   std::end(et_it->second->edges_), [&](paxmon::edge const* e) {
                     return (e->from_->time_ == l_conn->d_time_ &&
                             e->to_->time_ == l_conn->a_time_);
                   });
  assert(it != std::end(et_it->second->edges_));

  event_node* result;
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
    std::vector<light_connection const*> const to_l_conns,
    schedule const& sched, graph& g,
    std::vector<motis::paxmon::edge*>& interchange_edges) {
  auto from_n = find_event_node(from_l_conn, motis::event_type::ARR, g, sched);
  for (auto const c : to_l_conns) {
    auto to_n = find_event_node(c, motis::event_type::DEP, g, sched);
    duration const inch_duration = to_n->time_ - from_n->time_;
    add_interchange_wo_psgs(from_n, to_n, inch_duration, g, interchange_edges);
  }
}

void build_time_expanded_graph(paxmon_data const& data, schedule const& sched) {
  graph graph;
  std::vector<motis::paxmon::edge*> interchange_edges;

  trip_capacity_map_t trip_capacity_map{data.trip_capacity_map_};
  category_capacity_map_t category_capacity_map{data.category_capacity_map_};
  std::uint16_t default_capacity{data.default_capacity_};

  for (auto const route_trips : sched.expanded_trips_) {
    for (trip const* rt : route_trips) {
      get_or_add_trip_graph_only(sched, graph, to_extern_trip(sched, rt),
                                 trip_capacity_map, category_capacity_map,
                                 default_capacity);
    }
  }

  std::cout << "Edges before interchanges: " << std::endl;
  std::cout << std::accumulate(graph.nodes_.begin(), graph.nodes_.end(), 0.0,
                               [](double sum, auto& n) {
                                 return sum + n->in_edges_.size();
                               })
            << std::endl;

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
            build_interchange_edges(&c, rel_l_conns, sched, graph,
                                    interchange_edges);
          }
        }
      }
    }
  }

  std::cout << "Edges after interchanges: " << std::endl;
  std::cout << std::accumulate(graph.nodes_.begin(), graph.nodes_.end(), 0.0,
                               [](double sum, auto& n) {
                                 return sum + n->in_edges_.size();
                               })
            << std::endl;
  std::for_each(graph.nodes_.begin(), graph.nodes_.end(),
                [](std::unique_ptr<event_node>& n) {
                  for (auto const e : n->in_edges_) {
                    if (e->type_ == motis::paxmon::edge_type::INTERCHANGE) {
                      std::cout << "inch edge from " << e->from_->type_
                                << " to " << e->to_->type_ << ", from time "
                                << e->from_->time_ << " to time "
                                << e->to_->time_
                                << ", transfer time: " << e->transfer_time_
                                << std::endl;
                    }
                  }

                  n->out_edges_;
                });

  throw std::runtime_error("I build time expanded graph");
}

}  // namespace motis::paxassign