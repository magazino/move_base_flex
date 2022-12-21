#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_msgs/GetPathAction.h>

#include <mbf_abstract_nav/planner_action.h>
#include <mbf_abstract_nav/abstract_planner_execution.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <string>

using namespace mbf_abstract_nav;

// in this test we will use an action server and an action client, since
// we cannot mock the goal-handle.

using geometry_msgs::PoseStamped;
using mbf_abstract_core::AbstractPlanner;

// a mocked planner, allowing to control the outcome
struct MockPlanner : public AbstractPlanner
{
  MOCK_METHOD6(makePlan, uint32_t(const PoseStamped&, const PoseStamped&, double, std::vector<PoseStamped>&, double&,
                                  std::string&));

  // the cancel call is for now uninteresting. if this changes, we can also mock it.
  bool cancel()
  {
    return true;
  }
};

using mbf_abstract_nav::AbstractPlannerExecution;
using mbf_msgs::GetPathAction;

using testing::_;
using testing::DoAll;
using testing::Eq;
using testing::Field;
using testing::Return;
using testing::SetArgReferee;
using testing::Test;

// a mocked action server
struct MockedActionServer : public actionlib::ActionServerBase<GetPathAction>
{
  // define the types we are using
  typedef actionlib::ServerGoalHandle<GetPathAction> GoalHandle;
  typedef actionlib::ActionServerBase<GetPathAction> ActionServerBase;

  MockedActionServer(boost::function<void(GoalHandle)> goal_cb, boost::function<void(GoalHandle)> cancel_cb)
    : ActionServerBase(goal_cb, cancel_cb, true)
  {
  }

  // the mocked method
  MOCK_METHOD2(publishResult, void(const actionlib_msgs::GoalStatus&, const Result&));

  // methods below are not required for now but might be mocked in the future
  virtual void initialize()
  {
  }

  void publishFeedback(const actionlib_msgs::GoalStatus&, const Feedback&)
  {
  }

  void publishStatus()
  {
  }
};

// action which will trigger our condition variable, so we can wait until the runImpl thread reaches makePlan
ACTION_P(Notify, cv)
{
  cv->notify_all();
}

// test-fixture
struct PlannerActionFixture : public Test
{
  boost::shared_ptr<MockPlanner> planner;     ///< the mocked planner
  boost::condition_variable done_condition_;  ///< cv triggered on makePlan
  MoveBaseFlexConfig config;                  ///< config for the mbf

  std::string action_name;
  TFPtr tf;

  // todo change this from const ref
  std::string global_frame;
  std::string local_frame;
  ros::Duration dur;
  mbf_utility::RobotInformation robot_info;

  PlannerAction planner_action;
  ros::NodeHandle nh;

  MockedActionServer action_server;

  mbf_msgs::GetPathResult expected_result;
  mbf_msgs::GetPathActionGoalPtr goal;

  PlannerActionFixture()
    : planner(new MockPlanner())
    , action_name("action_name")
    , tf(new TF())
    , global_frame("global_frame")
    , local_frame("local_frame")
    , dur(0)
    , goal(new mbf_msgs::GetPathActionGoal())
    , robot_info(*tf, global_frame, local_frame, dur)
    , planner_action(action_name, robot_info)
    , action_server(boost::bind(&PlannerActionFixture::callAction, this, _1),
                    boost::bind(&PlannerActionFixture::cancelAction, this, _1))
  {
    tf->setUsingDedicatedThread(true);
    config.planner_patience = 0;
  }

  void TearDown()
  {
    // call the server - this is where the actual test call happens
    action_server.goalCallback(goal);

    // wait until the cv gets triggered
    boost::mutex m;
    boost::unique_lock<boost::mutex> lock(m);
    done_condition_.wait_for(lock, boost::chrono::seconds(1));
  }

  void callAction(MockedActionServer::GoalHandle goal_handle)
  {
    planner_action.start(goal_handle,
                         boost::make_shared<AbstractPlannerExecution>("plugin", planner, robot_info, config));
  }

  void cancelAction(MockedActionServer::GoalHandle goal_handle)
  {
    planner_action.cancel(goal_handle);
  }
};

TEST_F(PlannerActionFixture, emptyPath)
{
  // goal with frames, so we can pass tf-lookup
  goal->goal.use_start_pose = true;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;

  // setup the expectation
  // we dont return anything here, so the outcome should be empty path
  EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(Return(0));
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::EMPTY_PATH))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));
}

#if !ROS_VERSION_MINIMUM(1, 14, 0)
// disable the test on kinetic and lunar, until I figure out how to transform
// from global_frame into global_frame in the tf framework
TEST_F(PlannerActionFixture, DISABLED_success)
#else
TEST_F(PlannerActionFixture, success)
#endif
{
  // create a dummy path
  std::vector<geometry_msgs::PoseStamped> path(10);
  // set the frame such that we can skip tf
  for (size_t ii = 0; ii != path.size(); ++ii)
  {
    path[ii].header.frame_id = global_frame;
    path[ii].pose.orientation.w = 1;
  }

  // goal with frames, so we can pass tf-lookup
  goal->goal.use_start_pose = true;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;

  // setup the expectation
  EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(DoAll(SetArgReferee<3>(path), Return(0)));
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::SUCCESS))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));
}

TEST_F(PlannerActionFixture, tfError)
{
  // create a dummy path
  std::vector<geometry_msgs::PoseStamped> path(10);
  // set the frame such that we fail at the tf
  for (size_t ii = 0; ii != path.size(); ++ii)
    path[ii].header.frame_id = "unknown";

  // setup the expectation - we succeed here
  EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(DoAll(SetArgReferee<3>(path), Return(0)));
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::TF_ERROR))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));

  // goal with frames, so we can pass tf-lookup
  goal->goal.use_start_pose = true;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;
}

TEST_F(PlannerActionFixture, noRobotPose)
{
  // test case where we fail to get a valid robot pose.
  // in this case we will receive a TF_ERROR from the server.

  // make us use the robot pose
  goal->goal.use_start_pose = false;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;

  // set the expectation
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::TF_ERROR))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));
}

TEST_F(PlannerActionFixture, noPathFound)
{
  // test case where the planner fails.
  // in this case we will receive NO_PLAN_FOUND from the server.
  config.planner_max_retries = 0;

  // valid goal
  goal->goal.use_start_pose = true;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;

  // set the expectation: the planner returns a failure
  EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(Return(mbf_msgs::GetPathResult::NO_PATH_FOUND));
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::NO_PATH_FOUND))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));
}

ACTION(SleepAndFail)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  return 11;
}

TEST_F(PlannerActionFixture, patExceeded)
{
  // test case where the planner fails multiple times and we are out of patience

  // setup the config; this will be passed to the execution
  config.planner_max_retries = 5;
  config.planner_patience = 0.001;

  // valid goal
  goal->goal.use_start_pose = true;
  goal->goal.start_pose.header.frame_id = goal->goal.target_pose.header.frame_id = global_frame;

  // set the expectation: the planner returns a failure
  EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillRepeatedly(SleepAndFail());
  EXPECT_CALL(action_server,
              publishResult(_, Field(&mbf_msgs::GetPathResult::outcome, Eq(mbf_msgs::GetPathResult::PAT_EXCEEDED))))
      .Times(1)
      .WillOnce(Notify(&done_condition_));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_types");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
