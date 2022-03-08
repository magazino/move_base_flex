#include <gtest/gtest.h>
#include <mbf_abstract_nav/abstract_execution_base.h>

#include <boost/chrono.hpp>

using namespace mbf_abstract_nav;

// our dummy implementation of the AbstractExecutionBase
// it basically runs until we cancel it.
struct DummyExecutionBase : public AbstractExecutionBase
{
  DummyExecutionBase(const std::string& _name, const mbf_utility::RobotInformation& ri)
    : AbstractExecutionBase(_name, ri)
  {
  }

  // implement the required interfaces
  bool cancel()
  {
    cancel_ = true;
    condition_.notify_all();
    return true;
  }

protected:
  void run()
  {
    boost::mutex mutex;
    boost::unique_lock<boost::mutex> lock(mutex);

    // wait until someone says we are done (== cancel or stop)
    // we set a timeout, since we might miss the cancel call (especially if we
    // run on an environment with high CPU load)
    condition_.wait_for(lock, boost::chrono::seconds(1));
    outcome_ = 0;
  }
};

// shortcuts...
using testing::Test;

// the fixture owning the instance of the DummyExecutionBase
struct AbstractExecutionFixture : public Test
{
  TF tf_;
  DummyExecutionBase impl_;
  mbf_utility::RobotInformation ri_;

  AbstractExecutionFixture() : ri_(tf_, "global_frame", "local_frame", ros::Duration(), ""), impl_("foo", ri_)
  {
  }
};

TEST_F(AbstractExecutionFixture, timeout)
{
  // start the thread
  impl_.start();

  // make sure that we timeout and don't alter the outcome
  EXPECT_EQ(impl_.waitForStateUpdate(boost::chrono::microseconds(60)), boost::cv_status::timeout);
  EXPECT_EQ(impl_.getOutcome(), 255);
}

TEST_F(AbstractExecutionFixture, success)
{
  // start the thread
  impl_.start();
  EXPECT_EQ(impl_.waitForStateUpdate(boost::chrono::microseconds(60)), boost::cv_status::timeout);

  // cancel, so we set the outcome to 0
  impl_.cancel();
  impl_.join();
  EXPECT_EQ(impl_.getOutcome(), 0);
}

TEST_F(AbstractExecutionFixture, restart)
{
  // call start multiple times without waiting for its termination
  for (size_t ii = 0; ii != 10; ++ii)
    impl_.start();
}

TEST_F(AbstractExecutionFixture, stop)
{
  // call stop/terminate multiple times. this should be a noop
  for (size_t ii = 0; ii != 10; ++ii)
  {
    impl_.stop();
    impl_.join();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}