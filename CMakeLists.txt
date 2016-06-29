cmake_minimum_required(VERSION 2.8.3)
project(rgbd)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geolib2
  image_geometry
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  tue_serialization
  message_generation
)

add_message_files(
  FILES
    RGBDMsg.msg
)

add_service_files(
  FILES
    GetRGBD.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rgbd
  CATKIN_DEPENDS cv_bridge geolib2 image_geometry message_filters pcl_conversions pcl_ros roscpp tue_serialization
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# to show header files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(rgbd    src/Server.cpp
                    src/Client.cpp
                    src/ClientRaw.cpp
                    src/Image.cpp
                    src/View.cpp
                    src/serialization.cpp
                    src/ros/conversions.cpp

                    src/shared_mem_server.cpp
                    src/shared_mem_client.cpp

                    ${HEADER_FILES})
target_link_libraries(rgbd ${catkin_LIBRARIES} rt)

# - - - - - - - - - - - - - - - - - TOOLS - - - - - - - - - - - - - - - - -

add_executable(rgbd_server src/server.cpp)
target_link_libraries(rgbd_server rgbd)

add_executable(rgbd_viewer src/viewer.cpp)
target_link_libraries(rgbd_viewer rgbd)

add_executable(rgbd_to_ros src/rgbd_to_ros.cpp)
target_link_libraries(rgbd_to_ros rgbd)

add_executable(rgbd_save src/save.cpp)
target_link_libraries(rgbd_save rgbd)

add_executable(rgbd_view src/view.cpp)
target_link_libraries(rgbd_view rgbd)

# compile the library after messages have been generated
add_dependencies(rgbd ${PROJECT_NAME}_generate_messages_cpp)

add_executable(record_to_video tools/record_to_video.cpp)
target_link_libraries(record_to_video rgbd)

add_executable(multitool tools/multitool.cpp)
target_link_libraries(multitool rgbd)

add_executable(presenter tools/presenter.cpp)
target_link_libraries(presenter rgbd)

add_executable(rgbd_to_rgb_png tools/rgbd_to_rgb_png.cpp)
target_link_libraries(rgbd_to_rgb_png rgbd)

# - - - - - - - - - - - - - - - - - TESTS - - - - - - - - - - - - - - - - -

add_executable(rgbd_test_server test/test_server.cpp)
target_link_libraries(rgbd_test_server rgbd)

add_executable(rgbd_test_client test/test_client.cpp)
target_link_libraries(rgbd_test_client rgbd)

add_executable(rgbd_test_serialization test/test_serialization.cpp)
target_link_libraries(rgbd_test_serialization rgbd)

add_executable(amigo_full_screen src/amigo_full_screen.cpp)
target_link_libraries(amigo_full_screen rgbd)

