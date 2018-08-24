# Move Base Flex Simple Navigation Server {#mainpage}


The mbf_simple_nav package contains a simple navigation server implementation of Move Base Flex (MBF). The simple navigation server is bound to no map representation. It provides actions for planning, controlling and recovering. MBF loads all defined plugins which are defined in the lists *planners*, *controllers* and *recovery_behaviors*. Each list holds a pair of a *name* and a *type*. The *type* defines which kind of plugin to load. The *name* defines under which name the plugin should be callable by the actions. 

It tries to load the defined plugins which implements the defined interfaces in [mbf_abstract_core](wiki.ros.org/mbf_abstract_core).