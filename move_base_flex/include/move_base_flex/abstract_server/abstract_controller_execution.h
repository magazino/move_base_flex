/*
 *  Copyright 2017, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_controller_execution.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_
#define MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_

#include <pluginlib/class_loader.h>
#include <boost/chrono/thread_clock.hpp>
#include <boost/chrono/duration.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/abstract_local_planner.h>

#include "move_base_flex/navigation_utility.h"
#include "move_base_flex/MoveBaseFlexConfig.h"

namespace move_base_flex
{

template<typename LOCAL_PLANNER_BASE>
  class AbstractControllerExecution
  {
  public:

    typedef boost::shared_ptr<AbstractControllerExecution<LOCAL_PLANNER_BASE> > Ptr;

    AbstractControllerExecution(boost::condition_variable &condition,
                                const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr,
                                std::string package, std::string class_name);

    virtual ~AbstractControllerExecution();

    bool startMoving();

    void stopMoving();

    void setNewPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

    enum ControllerState
    {
      INITIALIZED,
      STARTED,
      PLANNING,
      NO_PLAN,
      MAX_RETRIES,
      PAT_EXCEEDED,
      EMPTY_PLAN,
      NO_LOCAL_CMD,
      GOT_LOCAL_CMD,
      ARRIVED_GOAL,
      STOPPED,
    };

    ControllerState getState();

    uint8_t getErrorCode();

    const std::string &getErrorMessage();

    ros::Time getLastCycleStartTime();

    ros::Time getLastValidCmdVelTime();

    bool isPatienceExceeded();

    void getVelocityCmd(geometry_msgs::TwistStamped &vel_cmd_stamped);

    void initialize();

    void reconfigure(move_base_flex::MoveBaseFlexConfig &config);

  protected:

    std::string plugin_name_;

    // class loader, to load the local planner plugin
    pluginlib::ClassLoader<LOCAL_PLANNER_BASE> class_loader_local_planner_;

    // the local planer to calculate the velocity command
    boost::shared_ptr<LOCAL_PLANNER_BASE> local_planner_;

    const boost::shared_ptr<tf::TransformListener> &tf_listener_ptr;

    void setVelocityCmd(const geometry_msgs::TwistStamped &vel_cmd_stamped);

    // main thread function -> moving the robot
    virtual void run();

    ros::Time cycle_start_time;

    int max_retries_;

    ros::Duration patience_;

  private:

    virtual void initMovingPlugin() = 0;

    // stops the robot
    void publishZeroVelocity();

    boost::mutex state_mtx_;

    void setState(ControllerState state);

    boost::mutex plan_mtx_;
    boost::mutex vel_cmd_mtx_;
    boost::mutex lct_mtx_;

    bool new_plan_;

    bool hasNewPlan();

    void getNewPlan(std::vector<geometry_msgs::PoseStamped> &plan);

    geometry_msgs::TwistStamped vel_cmd_stamped_;

    // current global path
    std::vector<geometry_msgs::PoseStamped> plan_;

    // condition to wake up control thread
    boost::condition_variable &condition_;

    // thread for moving
    boost::thread thread_;

    // timing of the moving thread
    boost::chrono::microseconds calling_duration_;

    // frames to get the current global robot pose
    std::string robot_frame_;
    std::string global_frame_;

    // subscribers and publishers
    ros::Publisher vel_pub_;

    // internal state
    AbstractControllerExecution::ControllerState state_;
    uint8_t error_code_;
    std::string error_message_;
    double tf_timeout_;

    // dynamic reconfigure attributes and methods
    boost::recursive_mutex configuration_mutex_;

    // is the moving thread running
    bool moving_;
  };

} /* namespace move_base_flex */

#include "move_base_flex/abstract_server/impl/abstract_controller_execution.tcc"

#endif /* MOVE_BASE_FLEX__ABSTRACT_CONTROLLER_EXECUTION_H_ */
