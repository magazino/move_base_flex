^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mbf_costmap_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
