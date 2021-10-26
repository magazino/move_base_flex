^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_costmap_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2021-10-26)
------------------
* make the costmaps used for planning and controlling configurable, see #278
* return empty footprint if partly outside of the map, see #272
* add tf_transform_ptr to the abstract planner execution, see #256
* transform pose to global frame before calling planner in costmap planner execution, see #256

0.3.4 (2020-12-02)
------------------
* fix blind driving, see #243

0.3.3 (2020-11-05)
------------------
* Fix controller fails if robot pose gets older than tf_timeout, see #231
* clear the costmap before deactivating it, see #220
* Use `catkin_install_python` to install legacy relay. see #219
* For move_base_legacy_relay, keep configured base local and global planners to send to MBF, see #209
* Fix deallocation on shutdown by discarding all plugins and resetting action server pointers, see #199
* Make reference symbol position consistent across the project, see #198
* Move RobotInformation to mbf_utility, as it can be used generaly, see #196
* Prevent unrelated type casts for Cell, see #197

0.3.2 (2020-05-25)
------------------
* Remove dependency on base_local_planner and move FootprintHelper class to mbf_costmap_nav and make it static

0.3.1 (2020-04-07)
------------------
* Ensure that check_costmap_mutex is destroyed after timer.
* Avoid crash on shutdown by stop shutdown_costmap_timer on destructor
  and explicitly call the costmap_nav_srv destructor

0.3.0 (2020-03-31)
------------------
* add output for cancel method if nav_core plugin is wrapped
* unify license declaration to BSD-3

0.2.5 (2019-10-11)
------------------
* Add clear_on_shutdown functionality
* Do not pass boost functions to abstract server to (de)activate costmaps.
  Run instead abstract methods (possibly) overridden in the costmap server,
  all costmap-related handling refactored to a new CostmapWrapper class
* On controller execution, check that local costmap is current

0.2.4 (2019-06-16)
------------------
* Add check_point_cost service
* Lock costmaps on clear_costmaps service
* Replace recursive mutexes with normal ones when not needed

0.2.3 (2018-11-14)
------------------
* single publisher for controller execution objects

0.2.2 (2018-10-10)
------------------
* Do not use MultiThreadedSpinner, as costmap updates can crash when combining laser scans and point clouds
* Make start/stop costmaps mutexed, since concurrent calls to start can lead to segfaults

0.2.1 (2018-10-03)
------------------
* Make MBF melodic and indigo compatible
* Fix GoalHandle references bug in callbacks

0.2.0 (2018-09-11)
------------------
* Update copyright and 3-clause-BSD license
* Concurrency for planners, controllers and recovery behaviors

0.1.0 (2018-03-22)
------------------
* First release of move_base_flex for kinetic and lunar
