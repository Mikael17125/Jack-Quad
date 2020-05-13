# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;roscpp;std_msgs;jack_framework_common;cmake_modules;jack_controller".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbezier_walk".split(';') if "-lbezier_walk" != "" else []
PROJECT_NAME = "bezier_walk"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
