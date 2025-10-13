cmake_minimum_required(VERSION 3.20)

project(ros_dds_bridge)

# Find system dependencies first (Fast DDS / Fast CDR)
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()

if(NOT fastdds_FOUND)
    find_package(fastdds 3 REQUIRED)
endif()

# Catkin (ROS1) setup
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

# Set C++11 (compatible with Melodic toolchains)
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

file(GLOB_RECURSE DDS_MESSAGE_SOURCES_CXX CONFIGURE_DEPENDS "src/*.cxx")

message(STATUS "Found generated IDL sources: ${USER_IDL_GENERATED_SOURCES}")

# Declare the catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

# Bridge executable that composes translators
#add_executable(RosDdsBridge src/RosDdsBridge.cpp src/ros_to_dds/RosToDdsPoseBridge.cpp src/BridgeManager.cpp ${DDS_MESSAGE_SOURCES_CXX})
add_executable(RosDdsBridge src/RosDdsBridge.cpp ${DDS_MESSAGE_SOURCES_CXX})
target_include_directories(RosDdsBridge PRIVATE src)
target_link_libraries(RosDdsBridge fastdds fastcdr ${catkin_LIBRARIES})

# Install rules so this builds as a ROS package and the node is discoverable
install(TARGETS RosDdsBridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.idl"
)