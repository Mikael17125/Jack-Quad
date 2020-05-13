# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;roslib;jack_framework_common;cmake_modules".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ljack_controller".split(';') if "-ljack_controller" != "" else []
PROJECT_NAME = "jack_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
