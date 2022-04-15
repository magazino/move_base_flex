#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <ros/ros.h>

#include <mbf_abstract_core/abstract_controller.h>
#include <mbf_abstract_nav/MoveBaseFlexConfig.h>
#include <mbf_abstract_nav/abstract_controller_execution.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <vector>

using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::Twist;
using geometry_msgs::TwistStamped;
using mbf_abstract_core::AbstractController;
using mbf_abstract_nav::AbstractControllerExecution;
using mbf_abstract_nav::MoveBaseFlexConfig;
using testing::_;
using testing::Return;
using testing::Test;
// for kinetic
using tf::StampedTransform;

// the plan as a vector of poses
typedef std::vector<PoseStamped> plan_t;

// mocked planner so we can control its output
struct AbstractControllerMock : public AbstractController
{
  // the mocked pure virtual members
  MOCK_METHOD4(computeVelocityCommands, uint32_t(const PoseStamped&, const TwistStamped&, TwistStamped&, std::string&));
  MOCK_METHOD2(isGoalReached, bool(double, double));
  MOCK_METHOD1(setPlan, bool(const plan_t&));
  MOCK_METHOD0(cancel, bool());
};

ros::Publisher VEL_PUB, GOAL_PUB;
TFPtr TF_PTR;
mbf_utility::RobotInformation::Ptr ROBOT_INFO_PTR;

// fixture for our tests
struct AbstractControllerExecutionFixture : public Test, public AbstractControllerExecution
{
  AbstractControllerExecutionFixture()
    : AbstractControllerExecution("a name", AbstractController::Ptr(new AbstractControllerMock()), *ROBOT_INFO_PTR,
                                  VEL_PUB, GOAL_PUB, MoveBaseFlexConfig{})
  {
  }

  void TearDown() override
  {
    // after every test we expect that moving_ is set to false
    // we don't expect this for the case NO_LOCAL_CMD
    if (getState() != NO_LOCAL_CMD)
      EXPECT_FALSE(isMoving());

    // we have to stop the thread when the test is done
    join();
  }
};

TEST_F(AbstractControllerExecutionFixture, noPlan)
{
  // test checks the case where we call start() without setting a plan.
  // the expected output is NO_PLAN.

  // start the controller
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), NO_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, emptyPlan)
{
  // test checks the case where we pass an empty path to the controller.
  // the expected output is EMPTY_PLAN

  // set an empty plan
  setNewPlan(plan_t{}, true, 1, 1);

  // start the controller
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), EMPTY_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, invalidPlan)
{
  // test checks the case where the controller recjets the plan.
  // the expected output is INVALID_PLAN

  // setup the expectation: the controller rejects the plan
  AbstractControllerMock& mock = dynamic_cast<AbstractControllerMock&>(*controller_);
  EXPECT_CALL(mock, setPlan(_)).WillOnce(Return(false));
  // set a plan
  plan_t plan(10);
  setNewPlan(plan, true, 1, 1);

  // start the controller
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), INVALID_PLAN);
}

TEST_F(AbstractControllerExecutionFixture, internalError)
{
  // test checks the case where we cannot compute the current robot pose
  // the expected output is INTERNAL_ERROR

  // setup the expectation: the controller accepts the plan
  AbstractControllerMock& mock = dynamic_cast<AbstractControllerMock&>(*controller_);
  EXPECT_CALL(mock, setPlan(_)).WillOnce(Return(true));

  // set a plan
  plan_t plan(10);
  setNewPlan(plan, true, 1, 1);

  // set the robot frame to some thing else then the global frame (for our test case)
  global_frame_ = "global_frame";
  robot_frame_ = "not_global_frame";

  // start the controller
  ASSERT_TRUE(start());

  // wait for the status update
  // note: this timeout must be longer than the default tf-timeout
  waitForStateUpdate(boost::chrono::seconds(2));
  ASSERT_EQ(getState(), INTERNAL_ERROR);
}

// fixture making us pass computeRobotPose()
struct ComputeRobotPoseFixture : public AbstractControllerExecutionFixture
{
  void SetUp()
  {
    // setup the transform.
    global_frame_ = "global_frame";
    robot_frame_ = "robot_frame";
#ifdef USE_OLD_TF
    StampedTransform transform;
    transform.child_frame_id_ = robot_frame_;
    transform.frame_id_ = global_frame_;
    transform.stamp_ = ros::Time::now();
#else
    TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = global_frame_;
    transform.child_frame_id = robot_frame_;
    transform.transform.rotation.w = 1;
#endif
    // todo right now the mbf_utility checks on the transform age - but this does not work for static transforms
    TF_PTR->setTransform(transform, "mama");
  }
};

TEST_F(ComputeRobotPoseFixture, arrivedGoal)
{
  // test checks the case where we reach the goal.
  // the expected output is ARRIVED_GOAL

  // setup the expectation: the controller accepts the plan and says we are arrived
  AbstractControllerMock& mock = dynamic_cast<AbstractControllerMock&>(*controller_);
  EXPECT_CALL(mock, setPlan(_)).WillOnce(Return(true));
  EXPECT_CALL(mock, isGoalReached(_, _)).WillOnce(Return(true));

  // we compare against the back-pose of the plan
  plan_t plan(10);
  plan.back().header.frame_id = global_frame_;
  plan.back().pose.orientation.w = 1;

  // make the tolerances small
  setNewPlan(plan, true, 1e-3, 1e-3);

  // call start
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), ARRIVED_GOAL);
}

ACTION(ControllerException)
{
  throw std::runtime_error("Oh no! Controller throws an Exception");
}

TEST_F(ComputeRobotPoseFixture, controllerException)
{
  // setup the expectation: the controller accepts the plan and says we are not arrived.
  // the controller throws then an exception
  AbstractControllerMock& mock = dynamic_cast<AbstractControllerMock&>(*controller_);
  EXPECT_CALL(mock, setPlan(_)).WillOnce(Return(true));
  EXPECT_CALL(mock, isGoalReached(_, _)).WillOnce(Return(false));
  EXPECT_CALL(mock, computeVelocityCommands(_, _, _, _)).WillOnce(ControllerException());

  // setup the plan
  plan_t plan(10);
  setNewPlan(plan, true, 1e-3, 1e-3);

  // call start
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), INTERNAL_ERROR);
}

// fixture which will setup the mock such that we generate a controller failure
struct FailureFixture : public ComputeRobotPoseFixture
{
  void SetUp()
  {
    // setup the expectation: the controller accepts the plan and says we are not arrived.
    // it furhermore returns an error code
    AbstractControllerMock& mock = dynamic_cast<AbstractControllerMock&>(*controller_);
    EXPECT_CALL(mock, setPlan(_)).WillOnce(Return(true));
    EXPECT_CALL(mock, isGoalReached(_, _)).WillRepeatedly(Return(false));
    EXPECT_CALL(mock, computeVelocityCommands(_, _, _, _)).WillRepeatedly(Return(11));

    // setup the plan
    plan_t plan(10);
    setNewPlan(plan, true, 1e-3, 1e-3);

    // call the parent method for the computeRobotPose call
    ComputeRobotPoseFixture::SetUp();
  }
};

TEST_F(FailureFixture, maxRetries)
{
  // test verifies the case where we exceed the max-retries.
  // the expected output is MAX_RETRIES

  // enable the retries logic (max_retries > 0)
  max_retries_ = 1;

  // call start
  ASSERT_TRUE(start());

  // wait for the status update: in first iteration NO_LOCAL_CMD
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), NO_LOCAL_CMD);

  // wait for the status update: in second iteration MAX_RETRIES
  // bcs max_retries_ > 0 && ++retries > max_retries_
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), MAX_RETRIES);
}

TEST_F(FailureFixture, noValidCmd)
{
  // test verifies the case where we don't exceed the patience or max-retries conditions
  // the expected output is NO_VALID_CMD

  // disable the retries logic
  max_retries_ = -1;
  // call start
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), NO_LOCAL_CMD);
}

TEST_F(FailureFixture, patExceeded)
{
  // test verifies the case where we exceed the patience
  // the expected output is PAT_EXCEEDED

  // disable the retries logic and enable the patience logic: we cheat by setting it to a negative duration.
  max_retries_ = -1;
  patience_ = ros::Duration(-1e-3);

  // call start
  ASSERT_TRUE(start());

  // wait for the status update
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), PAT_EXCEEDED);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_types");
  ros::NodeHandle nh;
  // setup the pubs as global objects
  VEL_PUB = nh.advertise<Twist>("vel", 1);
  GOAL_PUB = nh.advertise<PoseStamped>("pose", 1);

  // setup the tf-publisher and robot info as a global objects
  TF_PTR.reset(new TF());
  TF_PTR->setUsingDedicatedThread(true);
  ros::Duration TF_TIMEOUT(1.0);
  ROBOT_INFO_PTR.reset(new mbf_utility::RobotInformation(*TF_PTR, "global_frame", "robot_frame", TF_TIMEOUT, ""));

  // suppress the logging since we don't want warnings to pollute the test-outcome
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
