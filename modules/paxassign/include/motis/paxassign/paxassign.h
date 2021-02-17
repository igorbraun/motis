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
  void on_monitor(const motis::module::msg_ptr& msg);
  std::vector<std::pair<combined_pg&, motis::paxmon::compact_journey>>
  cap_ilp_assignment(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, uint16_t const allowed_delay, schedule const& sched,
      std::map<std::string, std::tuple<double, double, double, double>>&
          variables_with_values,
      std::ofstream& results_file);
  void node_arc_ilp_assignment(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched);
  void heuristic_assignments(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, const schedule& sched);
  void filter_evaluation(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched);
  void count_scenarios(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched);
  void filter_and_opt_evaluation(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched);
  void find_suspicious_groups(
      std::map<unsigned, std::vector<combined_pg>>& combined_groups,
      paxmon_data& data, schedule const& sched);
  void toy_scenario(const motis::module::msg_ptr&);
};

}  // namespace motis::paxassign
