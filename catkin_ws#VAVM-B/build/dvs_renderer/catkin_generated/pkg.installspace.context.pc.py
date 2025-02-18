# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sensor_msgs;dvs_msgs;cv_bridge;image_transport;nodelet".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldvs_renderer_nodelet".split(';') if "-ldvs_renderer_nodelet" != "" else []
PROJECT_NAME = "dvs_renderer"
PROJECT_SPACE_DIR = "/home/julia/catkin_ws/install"
PROJECT_VERSION = "0.0.0"
