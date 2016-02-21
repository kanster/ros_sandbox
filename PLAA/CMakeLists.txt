cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_gensrv()

find_package(OpenCV 2.4 REQUIRED)
message(STATUS "OPENCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OPENCV LIBS REQUIRED ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

rosbuild_add_boost_directories()
rosbuild_add_executable(pf_localization
    src/pf_localization.cpp
    include/pf_localization.h
)
rosbuild_link_boost(pf_localization thread)
target_link_libraries(pf_localization ${OpenCV_LIBS})

rosbuild_add_executable(astar
    src/astar.cpp
    include/astar.h
)
rosbuild_link_boost(astar thread)
target_link_libraries(astar ${OpenCV_LIBS})
