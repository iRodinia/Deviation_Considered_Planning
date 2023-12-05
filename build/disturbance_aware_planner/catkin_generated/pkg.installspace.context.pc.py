# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;grid_map;grid_map_planner".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldisturbance_aware_planner;-lreference_governor".split(';') if "-ldisturbance_aware_planner;-lreference_governor" != "" else []
PROJECT_NAME = "disturbance_aware_planner"
PROJECT_SPACE_DIR = "/home/cz_linux/Documents/Deviation_Considered_Planning/install"
PROJECT_VERSION = "0.0.0"
