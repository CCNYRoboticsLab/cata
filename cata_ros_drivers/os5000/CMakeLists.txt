cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set (ROS_BUILD_TYPE RelWithDebInfo)

# Initialize the ROS build system.
rosbuild_init ()

# Set the default path for built executables to the "bin" directory.
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Set the name to use for the executable.
set (BINNAME oscompass)

# Set the source files to use with the executable.
set (SRCS ${SRCS} src/compassCore_SR.cpp)
set (SRCS ${SRCS} src/oscompassROS_SR.cpp)
set (SRCS ${SRCS} src/serial_SR.cpp)
set (SRCS ${SRCS} src/timing_SR.cpp)

# Have ROS autogenerate code used in messages.
rosbuild_genmsg ()

# Set the directories where include files can be found.
include_directories (include)

# Build the executable that will be used to run this node.
rosbuild_add_executable (${BINNAME} ${SRCS})

# Add dynamic reconfigure API.
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()
