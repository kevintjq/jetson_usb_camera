cmake_minimum_required(VERSION 3.0.2)

project(jetson_gst_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  sensor_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
)

add_definitions(
  ${GSTREAMER_CFLAGS_OTHER}
)

add_executable(jetson_gst_camera_node
  src/jetson_gst_camera.cpp
)

target_link_libraries(jetson_gst_camera_node
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

install(TARGETS jetson_gst_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
