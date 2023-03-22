// ros
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/testing_helper.h>

// mbf
#include "mbf_costmap_nav/search_helper.h"

namespace mbf_costmap_nav::test
{
class SearchHelperTest : public ::testing::Test
{
protected:
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{ tf_buffer };
  costmap_2d::Costmap2D* costmap;

  void SetUp() override
  {
    tf_buffer.setUsingDedicatedThread(true);
    if (!tf_buffer.canTransform("base_link", "map", ros::Time::now(), ros::Duration(5.0)))
    {
      FAIL() << "Cannot transform from base_link to map";
    }
    costmap = new costmap_2d::Costmap2D(10, 10, 1.0, 0.0, 0.0, 0);
  }

  geometry_msgs::Point toPoint(double x, double y)
  {
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    return point;
  }

  geometry_msgs::Pose2D toPose2D(double x, double y, double theta)
  {
    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    return pose;
  }

  void addObstacle(costmap_2d::Costmap2DROS* cm, double x, double y)
  {
    boost::shared_ptr<costmap_2d::ObstacleLayer> olayer;
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = cm->getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin();
         pluginp != plugins->end(); ++pluginp)
    {
      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
      if (plugin->getName().find("obstacles") != std::string::npos)
      {
        olayer = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
        addObservation(&(*olayer), x, y, MAX_Z / 2, 0, 0, MAX_Z / 2);
      }
    }

    cm->updateMap();
    printMap(*(cm->getCostmap()));
  }
};

TEST_F(SearchHelperTest, getNeighbors)
{
  Cell cell{ 0, 0 };
  auto neighbors = SearchHelper::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 3);
  EXPECT_EQ(neighbors[0].x, 0);
  EXPECT_EQ(neighbors[0].y, 1);
  EXPECT_EQ(neighbors[1].x, 1);
  EXPECT_EQ(neighbors[1].y, 0);
  EXPECT_EQ(neighbors[2].x, 1);
  EXPECT_EQ(neighbors[2].y, 1);

  cell = { 9, 9 };
  neighbors = SearchHelper::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 3);
  EXPECT_EQ(neighbors[0].x, 8);
  EXPECT_EQ(neighbors[0].y, 8);
  EXPECT_EQ(neighbors[1].x, 8);
  EXPECT_EQ(neighbors[1].y, 9);
  EXPECT_EQ(neighbors[2].x, 9);
  EXPECT_EQ(neighbors[2].y, 8);

  cell = { 5, 5 };
  neighbors = SearchHelper::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 8);
  EXPECT_EQ(neighbors[0].x, 4);
  EXPECT_EQ(neighbors[0].y, 4);
  EXPECT_EQ(neighbors[1].x, 4);
  EXPECT_EQ(neighbors[1].y, 5);
  EXPECT_EQ(neighbors[2].x, 4);
  EXPECT_EQ(neighbors[2].y, 6);
  EXPECT_EQ(neighbors[3].x, 5);
  EXPECT_EQ(neighbors[3].y, 4);
  EXPECT_EQ(neighbors[4].x, 5);
  EXPECT_EQ(neighbors[4].y, 6);
  EXPECT_EQ(neighbors[5].x, 6);
  EXPECT_EQ(neighbors[5].y, 4);
  EXPECT_EQ(neighbors[6].x, 6);
  EXPECT_EQ(neighbors[6].y, 5);
  EXPECT_EQ(neighbors[7].x, 6);
  EXPECT_EQ(neighbors[7].y, 6);

  cell = { 0, 5 };
  neighbors = SearchHelper::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 5);
  EXPECT_EQ(neighbors[0].x, 0);
  EXPECT_EQ(neighbors[0].y, 4);
  EXPECT_EQ(neighbors[1].x, 0);
  EXPECT_EQ(neighbors[1].y, 6);
  EXPECT_EQ(neighbors[2].x, 1);
  EXPECT_EQ(neighbors[2].y, 4);
  EXPECT_EQ(neighbors[3].x, 1);
  EXPECT_EQ(neighbors[3].y, 5);
  EXPECT_EQ(neighbors[4].x, 1);
  EXPECT_EQ(neighbors[4].y, 6);

  cell = { 5, 0 };
  neighbors = SearchHelper::getNeighbors(costmap, cell);
  ASSERT_EQ(neighbors.size(), 5);
  EXPECT_EQ(neighbors[0].x, 4);
  EXPECT_EQ(neighbors[0].y, 0);
  EXPECT_EQ(neighbors[1].x, 4);
  EXPECT_EQ(neighbors[1].y, 1);
  EXPECT_EQ(neighbors[2].x, 5);
  EXPECT_EQ(neighbors[2].y, 1);
  EXPECT_EQ(neighbors[3].x, 6);
  EXPECT_EQ(neighbors[3].y, 0);
  EXPECT_EQ(neighbors[4].x, 6);
  EXPECT_EQ(neighbors[4].y, 1);
}

TEST_F(SearchHelperTest, safetyPadding)
{
  costmap_2d::Costmap2DROS* cm = new costmap_2d::Costmap2DROS("unpadded", tf_buffer);

  auto footprint = SearchHelper::safetyPadding(cm, true, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);

  footprint = SearchHelper::safetyPadding(cm, false, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);

  cm = new costmap_2d::Costmap2DROS("padded", tf_buffer);
  footprint = SearchHelper::safetyPadding(cm, true, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.6f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.6f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.6f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.6f, footprint[2].y, 1e-6);

  footprint = SearchHelper::safetyPadding(cm, false, 0.1);
  EXPECT_EQ(3, footprint.size());
  EXPECT_NEAR(1.1f, footprint[0].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[0].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[1].x, 1e-6);
  EXPECT_NEAR(1.1f, footprint[1].y, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].x, 1e-6);
  EXPECT_NEAR(-1.1f, footprint[2].y, 1e-6);
}

TEST_F(SearchHelperTest, isPoseValid)
{
  std::vector<geometry_msgs::Point> footprint = { toPoint(-0.5, -0.5), toPoint(0.5, -0.5), toPoint(0.5, 0.5),
                                                  toPoint(-0.5, 0.5) };

  auto test = [&]() {
    EXPECT_FALSE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(5, 5, 0)));
    EXPECT_FALSE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.6, 5, 0)));
    EXPECT_FALSE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.6, 4.6, 0)));
    EXPECT_FALSE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.6, 4.6, M_PI)));
    EXPECT_FALSE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.5, 4.5, 0)));
    EXPECT_TRUE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.4, 4.4, 0)));
    EXPECT_TRUE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(4.5, 4.5, M_PI_4)));
    EXPECT_TRUE(SearchHelper::isPoseValid(costmap, footprint, toPose2D(3, 5, 0)));
  };

  costmap->setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
  test();

  costmap->setCost(5, 5, costmap_2d::NO_INFORMATION);
  test();
}

TEST_F(SearchHelperTest, findValidOrientation)
{
  costmap->setCost(5, 5, costmap_2d::LETHAL_OBSTACLE);
  // square
  std::vector<geometry_msgs::Point> footprint = { toPoint(-0.5, -0.5), toPoint(0.5, -0.5), toPoint(0.5, 0.5),
                                                  toPoint(-0.5, 0.5) };
  auto sol = SearchHelper::findValidOrientation(costmap, footprint, toPose2D(4.6, 4.6, 0), { M_PI_4 / 2, M_PI });
  ASSERT_TRUE(sol);
  EXPECT_NEAR(sol->theta, M_PI_4 / 2, 1e-6);

  sol = SearchHelper::findValidOrientation(costmap, footprint, toPose2D(4.6, 4.6, 0), { M_PI, 2 * M_PI });
  ASSERT_FALSE(sol);

  // rectangle
  footprint = { toPoint(-0.5, -0.4), toPoint(1.0, -0.4), toPoint(1.0, 0.4), toPoint(-0.5, 0.4) };
  sol = SearchHelper::findValidOrientation(costmap, footprint, toPose2D(4.5, 5.5, 0), { M_PI_4, M_PI });
  ASSERT_TRUE(sol);
  EXPECT_NEAR(sol->theta, M_PI_2, 1e-6);

  sol = SearchHelper::findValidOrientation(costmap, footprint, toPose2D(4.5, 5.5, -M_PI_2), { M_PI_4, M_PI });
  ASSERT_TRUE(sol);
  EXPECT_NEAR(sol->theta, -M_PI_2, 1e-6);
}

TEST_F(SearchHelperTest, search)
{
  costmap_2d::Costmap2DROS* cm = new costmap_2d::Costmap2DROS("search/global", tf_buffer);
  printMap(*(cm->getCostmap()));
  addObstacle(cm, 5.5, 5.5);

  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0     0     0    0     0     0     0
  4.5 |  0    0    0     0     0     0    x     0     0     0
  5.5 |  0    0    0     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     0    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */
  SearchConfig config{ M_PI_4, M_PI, 5.0, false, 0.0, toPose2D(5.5, 5.5, 0) };
  SearchHelper sh(cm, config);

  geometry_msgs::Pose2D sol;
  ASSERT_TRUE(sh.search(sol));
  EXPECT_NEAR(sol.x, 6.5, 1e-6);
  EXPECT_NEAR(sol.y, 4.5, 1e-6);
  EXPECT_NEAR(sol.theta, -M_PI_4, 1e-6);

  addObstacle(cm, 6.5, 4.5);
  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0     0     0     0
  4.5 |  0    0    0     0     0     0   254    0     0     0
  5.5 |  0    0    0     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0     x    0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */
  ASSERT_TRUE(sh.search(sol));
  EXPECT_NEAR(sol.x, 5.5, 1e-6);
  EXPECT_NEAR(sol.y, 7.5, 1e-6);
  EXPECT_NEAR(sol.theta, M_PI_4, 1e-6);

  addObstacle(cm, 5.5, 7.5);
  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0     0     0     0
  4.5 |  0    0    0     x     0     0   254    0     0     0
  5.5 |  0    0    0     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     0     0    254   0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */
  ASSERT_TRUE(sh.search(sol));
  EXPECT_NEAR(sol.x, 3.5, 1e-6);
  EXPECT_NEAR(sol.y, 4.5, 1e-6);
  EXPECT_NEAR(sol.theta, 3 * M_PI_4, 1e-6);

  addObstacle(cm, 3.5, 4.5);
  /*
  y/x   0.5  1.5  2.5   3.5   4.5   5.5  6.5   7.5   8.5   9.5
  ------------------------------------------------------------
  0.5 |  0    0    0     0     0     0    0    254   254   254
  1.5 |  0    0    0     0     0     0    0    254   254   254
  2.5 |  0    0    0    254   254   254   0     0     0     0
  3.5 |  0    0    0     0    254    0    0     0     0     0
  4.5 |  0    0    0    254    0     0   254    0     0     0
  5.5 |  0    0    0     0    254    G    0    254   254   254
  6.5 |  0    0    0     0    254    0    0    254   254   254
  7.5 |  0    0    0     x     0    254   0    254   254   254
  8.5 |  0    0    0     0     0     0    0     0     0     0
  9.5 |  0    0    0     0     0     0    0     0     0     0
  */
  ASSERT_TRUE(sh.search(sol));
  EXPECT_NEAR(sol.x, 3.5, 1e-6);
  EXPECT_NEAR(sol.y, 7.5, 1e-6);
  EXPECT_NEAR(sol.theta, M_PI_4, 1e-6);

  delete cm;
}

}  // namespace mbf_costmap_nav::test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "search_helper_test");
  // enable debug logging for tests
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  ros::AsyncSpinner spinner(0);
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
