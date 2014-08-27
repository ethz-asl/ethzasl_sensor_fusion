cmake_minimum_required(VERSION 2.8.3)
project(ssf_core)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs dynamic_reconfigure cmake_modules sensor_fusion_comm)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters

#add_message_files(
#  FILES
  # TODO: List your msg files here
#)

add_definitions (-Wall -O3)

# get eigen
find_package(Eigen REQUIRED)
include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/SSF_Core.cfg)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure sensor_fusion_comm
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
    LIBRARIES ssf_core
)

add_library(ssf_core src/SSF_Core.cpp src/measurement.cpp src/state.cpp)
add_dependencies(ssf_core ${PROJECT_NAME}_gencfg)
target_link_libraries(ssf_core ${catkin_LIBRRIES})

