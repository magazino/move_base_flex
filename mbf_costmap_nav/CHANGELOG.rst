^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_costmap_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
