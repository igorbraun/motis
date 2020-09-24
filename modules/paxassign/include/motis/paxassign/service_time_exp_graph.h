#pragma once

#include "motis/paxassign/time_expanded_graph.h"

namespace motis::paxassign {

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

double get_edge_capacity_utilization(eg_event_node const* from,
                                     eg_event_node const* to,
                                     extern_trip const& et,
                                     paxmon_data const& data) {
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
    return (double)(*edge_it)->passengers_ / (*edge_it)->capacity();
  }
  return 0.0;
}

eg_event_node* get_localization_node(combined_passenger_group const& cpg,
                                     time_expanded_graph const& te_graph,
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
                 cpg.localization_.arrival_time_ == e_ptr->to_->time_;
        });
    assert(at_edge != tr_data->second->edges_.end());
    return (*at_edge)->to_;
  } else {
    // CASE II: Passenger at station, either before journey or at interchange
    std::vector<eg_event_node*> relevant_nodes =
        utl::all(te_graph.nodes_) | utl::remove_if([&](auto const& n) {
          return n->station_ != cpg.localization_.at_station_->index_ ||
                 n->type_ != eg_event_type::WAIT ||
                 n->time_ < cpg.localization_.arrival_time_;
        }) |
        utl::transform([&](auto const& n) { return n.get(); }) | utl::vec();
    if (relevant_nodes.empty()) {
      return nullptr;
    }
    std::sort(std::begin(relevant_nodes), std::end(relevant_nodes),
              [](eg_event_node const* lhs, eg_event_node const* rhs) {
                return lhs->time_ < rhs->time_;
              });
    return relevant_nodes[0];
  }
}

}  // namespace motis::paxassign