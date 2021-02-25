#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_abstract_nav/abstract_planner_execution.h>

// too long namespaces...
using geometry_msgs::PoseStamped;
using mbf_abstract_core::AbstractPlanner;

// mocked version of a planner
// we will control the output of it
struct AbstractPlannerMock : public AbstractPlanner
{
  MOCK_METHOD6(makePlan, uint32_t(const PoseStamped&, const PoseStamped&, double, std::vector<PoseStamped>&, double&,
                                  std::string&));

  MOCK_METHOD0(cancel, bool());
};

using mbf_abstract_nav::AbstractPlannerExecution;
using mbf_abstract_nav::MoveBaseFlexConfig;
using testing::_;
using testing::Return;
using testing::Test;

// setup the test-fixture
struct AbstractPlannerExecutionFixture : public Test, public AbstractPlannerExecution
{
  PoseStamped pose;  // dummy pose to call start

  AbstractPlannerExecutionFixture()
    : AbstractPlannerExecution("foo", AbstractPlanner::Ptr{ new AbstractPlannerMock() }, TFPtr(), MoveBaseFlexConfig{})
  {
  }

  void TearDown() override
  {
    // we have to stop the thread when the test is done
    join();
  }
};

TEST_F(AbstractPlannerExecutionFixture, success)
{
  // the good case - we succeed
  // setup the expectation
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _)).WillOnce(Return(0));

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // check result
  ASSERT_EQ(waitForStateUpdate(boost::chrono::seconds(1)), boost::cv_status::no_timeout);
  ASSERT_EQ(getState(), FOUND_PLAN);
}

ACTION_P(Wait, cv)
{
  boost::mutex m;
  boost::unique_lock<boost::mutex> lock(m);
  cv->wait(lock);
  return 11;
}

TEST_F(AbstractPlannerExecutionFixture, cancel)
{
  // the cancel case. we simulate that we cancel the execution
  // setup the expectation
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  boost::condition_variable cv;
  // makePlan may or may not be called
  ON_CALL(mock, makePlan(_, _, _, _, _, _)).WillByDefault(Wait(&cv));
  EXPECT_CALL(mock, cancel()).Times(1).WillOnce(Return(true));

  // now call the method
  ASSERT_TRUE(start(pose, pose, 0));
  ASSERT_TRUE(cancel());

  // wake up run-thread
  cv.notify_all();

  // check result
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), CANCELED);
}

TEST_F(AbstractPlannerExecutionFixture, max_retries)
{
  // we expect that if the planner fails for max_retries times, that
  // the class returns MAX_RETRIES

  // configure the class
  MoveBaseFlexConfig config;
  config.planner_max_retries = 5;
  config.planner_patience = 100;  // set a high patience
  reconfigure(config);

  // setup the expectations
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);

  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _)).Times(config.planner_max_retries + 1).WillRepeatedly(Return(11));

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // check result
  ASSERT_EQ(waitForStateUpdate(boost::chrono::seconds(1)), boost::cv_status::no_timeout);
  ASSERT_EQ(getState(), MAX_RETRIES);
}

TEST_F(AbstractPlannerExecutionFixture, no_plan_found)
{
  // if no retries and no patience are configured, we return NO_PLAN_FOUND on
  // planner failure

  // configure the class
  MoveBaseFlexConfig config;
  config.planner_max_retries = 0;
  config.planner_patience = 0;
  reconfigure(config);

  // setup the expectations
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Return(11));

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // check result
  ASSERT_EQ(waitForStateUpdate(boost::chrono::seconds(1)), boost::cv_status::no_timeout);
  ASSERT_EQ(getState(), NO_PLAN_FOUND);
}

using testing::DoAll;
using testing::SetArgReferee;

TEST_F(AbstractPlannerExecutionFixture, sumDist)
{
  // simulate the case when the planner returns zero cost
  std::vector<geometry_msgs::PoseStamped> plan(4);
  for (size_t ii = 0; ii != plan.size(); ++ii)
    plan.at(ii).pose.position.x = ii;
  double cost = 0;

  // call the planner
  // the good case - we succeed
  // setup the expectation
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _))
      .WillOnce(DoAll(SetArgReferee<3>(plan), SetArgReferee<4>(cost), Return(0)));

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // check result
  ASSERT_EQ(waitForStateUpdate(boost::chrono::seconds(1)), boost::cv_status::no_timeout);
  ASSERT_EQ(getState(), FOUND_PLAN);
  ASSERT_EQ(getCost(), 3);
}

TEST_F(AbstractPlannerExecutionFixture, patience_exceeded)
{
  // if no retries and no patience are configured, we return NO_PLAN_FOUND on
  // planner failure

  // configure the class
  MoveBaseFlexConfig config;
  config.planner_max_retries = 0;
  config.planner_patience = 0.1;
  reconfigure(config);

  // setup the expectations
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  boost::condition_variable cv;
  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(Wait(&cv));

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // wait for the patience to elapse
  boost::this_thread::sleep_for(boost::chrono::milliseconds{ 200 });
  cv.notify_all();

  // check result
  waitForStateUpdate(boost::chrono::seconds(1));
  ASSERT_EQ(getState(), PAT_EXCEEDED);
}

ACTION(ThrowException)
{
  throw std::runtime_error("bad planner");
}

TEST_F(AbstractPlannerExecutionFixture, exception)
{
  // if we throw an exception, we expect that we can recover from it
  // setup the expectations
  AbstractPlannerMock& mock = dynamic_cast<AbstractPlannerMock&>(*planner_);
  EXPECT_CALL(mock, makePlan(_, _, _, _, _, _)).Times(1).WillOnce(ThrowException());

  // call and wait
  ASSERT_TRUE(start(pose, pose, 0));

  // check result
  ASSERT_EQ(waitForStateUpdate(boost::chrono::seconds(1)), boost::cv_status::no_timeout);
  ASSERT_EQ(getState(), INTERNAL_ERROR);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "read_types");
  ros::NodeHandle nh;
  // suppress the logging since we don't want warnings to polute the test-outcome
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
