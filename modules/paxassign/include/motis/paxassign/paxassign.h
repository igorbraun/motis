#pragma once

#include <memory>
#include <string>
#include <vector>

#include "conf/date_time.h"

#include "motis/module/module.h"
#include "motis/paxassign/combined_pg.h"
#include "motis/paxmon/paxmon_data.h"

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
  std::vector<std::pair<std::uint16_t, std::uint16_t>> cap_ilp_assignment(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched,
      std::map<std::string, std::tuple<double, double, double, double>>&
          variables_with_values, std::ofstream& results_file);
  void node_arc_ilp_assignment(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched, std::ofstream& results_file);
  void heuristic_assignments(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, const schedule& sched);
  void toy_scenario(const motis::module::msg_ptr&);
};

}  // namespace motis::paxassign
