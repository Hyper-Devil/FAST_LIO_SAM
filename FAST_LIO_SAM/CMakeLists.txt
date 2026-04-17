cmake_minimum_required(VERSION 3.0.2)
project(fast_lio_sam)

# SET(CMAKE_BUILD_TYPE "Debug")  
SET(CMAKE_BUILD_TYPE "Realese")  

ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread timer chrono filesystem system date_time)

# Force system libusb for this project only to avoid /opt/MVS libusb conflicts.
find_library(SYSTEM_LIBUSB_LIBRARY
  NAMES usb-1.0
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib
  NO_DEFAULT_PATH)
if(NOT SYSTEM_LIBUSB_LIBRARY)
  find_library(SYSTEM_LIBUSB_LIBRARY NAMES usb-1.0)
endif()
if(NOT SYSTEM_LIBUSB_LIBRARY)
  message(FATAL_ERROR "System libusb (usb-1.0) not found")
endif()
get_filename_component(SYSTEM_LIBUSB_DIR ${SYSTEM_LIBUSB_LIBRARY} DIRECTORY)
message(STATUS "Using system libusb: ${SYSTEM_LIBUSB_LIBRARY}")
message(STATUS "Using system libusb dir: ${SYSTEM_LIBUSB_DIR}")

find_package(GeographicLib QUIET)
if(NOT GeographicLib_FOUND)
  find_package(GeographicLib QUIET
    PATHS /usr/local /usr
    PATH_SUFFIXES lib/cmake/GeographicLib cmake/GeographicLib)
endif()

if(GeographicLib_FOUND)
  message(STATUS "Found GeographicLib via CMake package")
else()
  message(STATUS "GeographicLib CMake package not found, trying manual search")
  find_path(GeographicLib_INCLUDE_DIRS
    NAMES GeographicLib/LocalCartesian.hpp
    PATHS /usr/include /usr/local/include)

  find_library(GeographicLib_LIBRARY
    NAMES Geographic GeographicLib
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib /usr/local/lib64)

  if(GeographicLib_INCLUDE_DIRS AND GeographicLib_LIBRARY)
    set(GeographicLib_LIBRARIES ${GeographicLib_LIBRARY})
    set(GeographicLib_FOUND TRUE)
    message(STATUS "Found GeographicLib manually: ${GeographicLib_LIBRARY}")
  else()
    message(FATAL_ERROR
      "GeographicLib not found. Please install development package (e.g. libgeographic-dev) "
      "or set GeographicLib_DIR to the directory containing geographiclib-config.cmake")
  endif()
endif()

find_package( GTSAMCMakeTools )
find_package(GTSAM REQUIRED QUIET)

if(NOT GTSAM_INCLUDE_DIRS AND GTSAM_INCLUDE_DIR)
  set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})
endif()

if(NOT GTSAM_LIBRARIES)
  set(GTSAM_LIBRARIES gtsam)
endif()

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  include)

  # link directories
# link_directories(
#   ${GTSAM_LIBRARY_DIRS}
# )

add_message_files(
  FILES
  Pose6D.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
  save_pose.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL GTSAM
  INCLUDE_DIRS
)

# add_executable(fastlio_sam_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
# target_link_libraries(fastlio_sam_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
# target_include_directories(fastlio_sam_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_library(ikd_Tree include/ikd-Tree/ikd_Tree.cpp)
add_library(preprocess  src/preprocess.cpp)
add_executable(fastlio_sam_mapping src/laserMapping.cpp)
target_link_libraries(fastlio_sam_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${GeographicLib_LIBRARIES} ikd_Tree preprocess ${Boost_LIBRARIES} ${GTSAM_LIBRARIES} ${SYSTEM_LIBUSB_LIBRARY})
set_target_properties(fastlio_sam_mapping PROPERTIES
  BUILD_RPATH "${SYSTEM_LIBUSB_DIR}"
  INSTALL_RPATH "${SYSTEM_LIBUSB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  LINK_FLAGS "-Wl,--disable-new-dtags")
target_include_directories(fastlio_sam_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
