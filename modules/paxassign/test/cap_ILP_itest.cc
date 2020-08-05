#include "gtest/gtest.h"

#include "motis/core/access/station_access.h"
#include "motis/core/access/trip_access.h"
#include "motis/module/message.h"
#include "motis/loader/loader_options.h"
#include "motis/paxforecast/alternatives.h"
#include "motis/paxmon/messages.h"
#include "motis/ris/risml/risml_parser.h"
#include "motis/test/motis_instance_test.h"

using namespace motis::test;
using namespace motis::module;

static motis::loader::loader_options cap_ilp_dataset{
    .dataset_ = "modules/paxassign/test_resources/schedule",
    .schedule_begin_ = "20190730",
    .apply_rules_ = false};

struct cap_ilp_test : public motis_instance_test {
  cap_ilp_test()
      : motis::test::motis_instance_test(
            cap_ilp_dataset, {"ris", "rt", "routing", "paxassign", "paxmon"},
            {"--ris.input=modules/paxassign/test_resources/rismsg",
             //"--ris.init_time=2019-07-30T12:00:00",
             "--paxmon.journeys=modules/paxassign/test_resources/"
             "simplest_test_jrns.csv",
             "--paxmon.capacity=modules/paxassign/test_resources/"
             "default_capacity.csv"}) {}

  static msg_ptr forward(time_t time) {
    motis::module::message_creator fbb;
    fbb.create_and_finish(
        motis::MsgContent_RISForwardTimeRequest,
        motis::ris::CreateRISForwardTimeRequest(fbb, time).Union(),
        "/ris/forward");
    return make_msg(fbb);
  }
};

TEST_F(cap_ilp_test, ris_test) {
  call(forward(1564480800));
  auto trp = get_trip(sched(), "0000001", 1, 1564473600, "0000008", 1564498800,
                      "", true);
  auto const valid = std::map<std::string, bool>{
      {"0000001", true},  {"0000002", true},  {"0000003", true},
      {"0000004", false}, {"0000005", false}, {"0000006", false},
      {"0000007", false}, {"0000008", false}};
  for (auto const& trp_e : *trp->edges_) {
    motis::edge const* e = trp_e.get_edge();
    auto const dep = motis::ev_key{e, trp->lcon_idx_, motis::event_type::DEP};
    auto const& id = sched().stations_[dep.get_station_idx()]->eva_nr_;
    EXPECT_EQ(valid.at(id.str()), dep.lcon()->valid_);
  }
}

TEST_F(cap_ilp_test, ILP_result_test) {
  using namespace motis::paxmon;
  struct scenario_psg {
    uint32_t secondary_id_;
    compact_journey desired_alternative_;
  };

  std::vector<scenario_psg> expected_result{
      scenario_psg{
          1, compact_journey{std::vector<journey_leg>{
                 journey_leg{motis::extern_trip{"0000004", 2, 1564484700,
                                                "0000006", 1564491300, ""},
                             15, 17, 7865, 7975, transfer_info{}},
                 journey_leg{motis::extern_trip{"0000006", 3, 1564491900,
                                                "0000008", 1564502400, ""},
                             17, 19, 7985, 8160, transfer_info{}}}}},
      scenario_psg{
          2, compact_journey{std::vector<journey_leg>{
                 journey_leg{motis::extern_trip{"0000012", 7, 1564484700,
                                                "0000013", 1564487400, ""},
                             23, 24, 7865, 7910, transfer_info{}},
                 journey_leg{motis::extern_trip{"0000013", 8, 1564488600,
                                                "0000015", 1564494600, ""},
                             24, 26, 7930, 8030, transfer_info{}}}}},
      scenario_psg{3, compact_journey{std::vector<journey_leg>{journey_leg{
                          motis::extern_trip{"0000012", 6, 1564485000,
                                             "0000016", 1564506000, ""},
                          23, 27, 7870, 8220, transfer_info{}}}}},

      scenario_psg{4, compact_journey{std::vector<journey_leg>{journey_leg{
                          motis::extern_trip{"0000020", 10, 1564485000,
                                             "0000024", 1564502400, ""},
                          31, 34, 7870, 8035, transfer_info{}}}}

      },
      scenario_psg{5, compact_journey{}},
  };

  std::vector<msg_ptr> assignment_msgs;
  subscribe("/paxassign/ilp_result", msg_sink(&assignment_msgs));

  call(forward(1564480800));
  ASSERT_EQ(1, assignment_msgs.size());

  using namespace motis::paxmon;
  using namespace motis::paxassign;
  using namespace motis;
  auto const assignments_msg =
      motis_content(ConnAssignments, assignment_msgs[0]);

  for (auto const& assignment : *assignments_msg->assignments()) {
    auto const p_id = assignment->psg_id();
    auto sc_ptr =
        std::find_if(std::begin(expected_result), std::end(expected_result),
                     [&](auto const& sp) { return sp.secondary_id_ == p_id; });
    if (sc_ptr == std::end(expected_result)) {
      throw std::runtime_error("passenger not in scenario");
    }
    auto cj = from_fbs(sched(), assignment->conn());
    ASSERT_TRUE(sc_ptr->desired_alternative_ == cj);
  }
}