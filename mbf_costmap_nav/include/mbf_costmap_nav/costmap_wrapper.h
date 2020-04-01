/*
 *  Copyright 2019, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
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
 *  costmap_wrapper.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_
#define MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_

#include <costmap_2d/costmap_2d_ros.h>

#include <mbf_utility/types.h>


namespace mbf_costmap_nav
{
/**
 * @defgroup move_base_server Move Base Server
 * @brief Classes belonging to the Move Base Server level.
 */


/**
 * @brief The CostmapWrapper class manages access to a costmap by locking/unlocking its mutex and handles
 * (de)activation.
 *
 * @ingroup navigation_server move_base_server
 */
class CostmapWrapper : public costmap_2d::Costmap2DROS
{
public:
  typedef boost::shared_ptr<CostmapWrapper> Ptr;

  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  CostmapWrapper(const std::string &name, const TFPtr &tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~CostmapWrapper();

  /**
   * @brief Reconfiguration method called by dynamic reconfigure.
   * @param shutdown_costmap Determines whether or not to shutdown the costmap when move_base_flex is inactive.
   * @param shutdown_costmap_delay How long in seconds to wait after last action before shutting down the costmap.
   */
  void reconfigure(double shutdown_costmap, double shutdown_costmap_delay);

  /**
   * @brief Clear costmap.
   */
  void clear();

  /**
   * @brief Check whether the costmap should be activated.
   */
  void checkActivate();

  /**
   * @brief Check whether the costmap should and could be deactivated.
   */
  void checkDeactivate();

private:
  /**
   * @brief Timer-triggered deactivation of the costmap.
   */
  void deactivate(const ros::TimerEvent &event);

  //! Private node handle
  ros::NodeHandle private_nh_;

  boost::mutex check_costmap_mutex_;     //!< Start/stop costmap mutex; concurrent calls to start can lead to segfault
  bool shutdown_costmap_;                //!< don't update costmap when not using it
  bool clear_on_shutdown_;               //!< clear the costmap, when shutting down
  int16_t costmap_users_;                //!< keep track of plugins using costmap
  ros::Timer shutdown_costmap_timer_;    //!< costmap delayed shutdown timer
  ros::Duration shutdown_costmap_delay_; //!< costmap delayed shutdown delay
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_ */
