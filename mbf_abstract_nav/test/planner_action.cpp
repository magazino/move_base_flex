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

using namespace mbf_abstract_nav;

// in this test we will use an action server and an action client, since
// we cannot mock the goal-handle.

using geometry_msgs::PoseStamped;
using mbf_abstract_core::AbstractPlanner;

// a mocked planner, allowing to control the outcome
struct MockPlanner : public AbstractPlanner
{
    MOCK_METHOD6(makePlan,
                 uint32_t(const PoseStamped &, const PoseStamped &, double,
                          std::vector<PoseStamped> &, double &, std::string &));

    MOCK_METHOD0(cancel, bool());
};

// mocked client, so we can check the outcome
struct MockClient
{
    MOCK_METHOD0(call, void());
};

using mbf_abstract_nav::AbstractPlannerExecution;
using testing::Test;

// test-fixture putting clinet and server together
struct PlannerActionFixture : public Test
{
    boost::shared_ptr<MockPlanner> planner;
    MoveBaseFlexConfig config;

    std::string action_name;
    TF tf;

    // todo change this from const ref
    std::string global_frame;
    std::string local_frame;
    ros::Duration dur;
    mbf_utility::RobotInformation robot_info;

    PlannerAction planner_action;
    ros::NodeHandle nh;

    typedef actionlib::ActionServer<mbf_msgs::GetPathAction> ActionServer;
    typedef actionlib::SimpleActionClient<mbf_msgs::GetPathAction> ActionClient;

    ActionServer action_server;
    ActionClient action_client;

    MockClient mock_client;
    mbf_msgs::GetPathResult expected_result;
    mbf_msgs::GetPathGoal goal;

    PlannerActionFixture() : planner(new MockPlanner()),
                             action_name("action_name"),
                             global_frame("global_frame"),
                             local_frame("local_frame"),
                             dur(0),
                             robot_info(tf, global_frame, local_frame, dur),
                             planner_action(action_name, robot_info),
                             action_server(nh, "get_path", boost::bind(&PlannerActionFixture::callAction, this, _1),
                                           boost::bind(&PlannerActionFixture::cancelAction, this, _1), false),
                             action_client("get_path", true)
    {
        action_server.start();
        tf.setUsingDedicatedThread(true);
        config.planner_patience = 0;
    }

    void SetUp()
    {
        // wait for server
        ASSERT_TRUE(action_client.waitForServer(ros::Duration(1)));
    }

    void TearDown()
    {
        // here we fire up the request and wait for the result
        // setup the client expectation - we always want to hear something back
        EXPECT_CALL(mock_client, call()).Times(1);

        // send goal
        action_client.sendGoal(goal, boost::bind(&PlannerActionFixture::doneCallback, this, _1, _2));

        // wait here
        ASSERT_TRUE(action_client.waitForResult(ros::Duration(1)));
    }

    void callAction(ActionServer::GoalHandle goal_handle)
    {
        planner_action.start(goal_handle, boost::make_shared<AbstractPlannerExecution>("plugin", planner, config));
    }

    void cancelAction(ActionServer::GoalHandle goal_handle)
    {
        planner_action.cancel(goal_handle);
    }

    void doneCallback(const actionlib::SimpleClientGoalState &state,
                      const mbf_msgs::GetPathResultConstPtr &result)
    {
        mock_client.call();
        EXPECT_EQ(result->outcome, expected_result.outcome);
    }
};

using testing::_;
using testing::Return;

TEST_F(PlannerActionFixture, emptyPath)
{
    // setup the expectation
    // we dont return anything here, so the outcome should be empty path
    EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(Return(0));
    expected_result.outcome = mbf_msgs::GetPathResult::EMPTY_PATH;

    // goal with frames, so we can pass tf-lookup
    goal.use_start_pose = true;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;
}

using testing::DoAll;
using testing::SetArgReferee;

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
        path[ii].header.frame_id = global_frame;

    // setup the expectation
    EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(DoAll(SetArgReferee<3>(path), Return(0)));
    expected_result.outcome = mbf_msgs::GetPathResult::SUCCESS;

    // goal with frames, so we can pass tf-lookup
    goal.use_start_pose = true;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;
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
    expected_result.outcome = mbf_msgs::GetPathResult::TF_ERROR;

    // goal with frames, so we can pass tf-lookup
    goal.use_start_pose = true;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;
}

TEST_F(PlannerActionFixture, noRobotPose)
{
    // test case where we fail to get a valid robot pose.
    // in this case we will receive a TF_ERROR from the server.

    // make us use the robot pose
    goal.use_start_pose = false;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;

    // set the expectation
    expected_result.outcome = mbf_msgs::GetPathResult::TF_ERROR;
}

TEST_F(PlannerActionFixture, noPlanFound)
{
    // test case where the planner fails.
    // in this case we will receive NO_PLAN_FOUND from the server.
    config.planner_max_retries = 0;

    // valid goal
    goal.use_start_pose = true;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;

    // set the expectation: the planner retuns a failure
    EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillOnce(Return(mbf_msgs::GetPathResult::NO_PATH_FOUND));
    expected_result.outcome = mbf_msgs::GetPathResult::NO_PATH_FOUND;
}

ACTION(SleepAndFail)
{
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    return 11;
}

TEST_F(PlannerActionFixture, patExceeded)
{
    // test case where the planner fails multple times and we are out of patience

    // setup the config; this will be passed to the execution
    config.planner_max_retries = 5;
    config.planner_patience = 0.001;

    // valid goal
    goal.use_start_pose = true;
    goal.start_pose.header.frame_id = goal.target_pose.header.frame_id = global_frame;

    // set the expectation: the planner retuns a failure
    EXPECT_CALL(*planner, makePlan(_, _, _, _, _, _)).WillRepeatedly(SleepAndFail());
    expected_result.outcome = mbf_msgs::GetPathResult::PAT_EXCEEDED;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_types");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);

    testing::InitGoogleTest(&argc, argv);
    spinner.start();
    int result = RUN_ALL_TESTS();
    spinner.stop();
    return result;
}
