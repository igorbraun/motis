#pragma once

#include <memory>
#include <string>
#include <vector>

#include "conf/date_time.h"

#include "motis/module/module.h"
#include "motis/paxforecast/combined_passenger_group.h"
#include "motis/paxmon/paxmon_data.h"

using namespace motis::paxforecast;
using namespace motis::paxmon;

namespace motis::paxassign {

struct paxassign : public motis::module::module {
  paxassign();
  ~paxassign() override;

  paxassign(paxassign const&) = delete;
  paxassign& operator=(paxassign const&) = delete;

  paxassign(paxassign&&) = delete;
  paxassign& operator=(paxassign&&) = delete;

  void init(motis::module::registry&) override;

private:
  void on_forecast(motis::module::msg_ptr const& msg);
  void on_monitor(const motis::module::msg_ptr& msg);
  void cap_ilp_assignment(
      std::map<unsigned, std::vector<combined_passenger_group>>&
          combined_groups,
      paxmon_data& data, schedule const& sched);
  void whole_graph_ilp_assignment(
      std::map<unsigned, std::vector<combined_passenger_group>>&
          combined_groups,
      paxmon_data& data, schedule const& sched);
  void toy_scenario(const motis::module::msg_ptr&);
};

}  // namespace motis::paxassign
